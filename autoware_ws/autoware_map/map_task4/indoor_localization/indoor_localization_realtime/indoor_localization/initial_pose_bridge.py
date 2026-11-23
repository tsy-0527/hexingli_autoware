"""Forward a global-localization result to Autoware's pose_initializer exactly once.

/room_global_localizer/pose (PoseWithCovarianceStamped)
   -> /localization/initialize (AUTO, with pose)  -> NDT align -> EKF reset
A pose is only forwarded when localization is NOT initialized, unless the bridge
was armed via ~/arm, so a repeatedly published pose never keeps resetting the EKF.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:  # newer Autoware (component interface uses autoware_localization_msgs)
    from autoware_localization_msgs.srv import InitializeLocalization as InitSrv
except ImportError:  # older Autoware
    from autoware_adapi_v1_msgs.srv import InitializeLocalization as InitSrv
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState as InitState


class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__('initial_pose_bridge')
        d = self.declare_parameter
        d('input_topic', '/room_global_localizer/pose')
        d('initialize_service', '/localization/initialize')
        d('state_topic', '/localization/initialization_state')
        d('map_frame', 'map')
        d('min_interval_sec', 5.0)
        d('reinitialize_when_initialized', False)
        g = lambda k: self.get_parameter(k).value  # noqa
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.cli = self.create_client(InitSrv, g('initialize_service'))
        self.create_subscription(PoseWithCovarianceStamped, g('input_topic'), self.on_pose, 10)
        self.create_subscription(InitState, g('state_topic'), self.on_state, latched)
        self.pub = self.create_publisher(String, '~/status', latched)
        self.create_service(Trigger, '~/arm', self.on_arm)
        self.state = None
        self.armed = False
        self.busy = False
        self.last_call = 0.0
        self.status('IDLE', 'waiting for pose')

    def status(self, s, msg, **kw):
        self.pub.publish(String(data=json.dumps(dict(state=s, message=msg, **kw), ensure_ascii=False)))
        self.get_logger().info(f'[{s}] {msg}')

    def on_state(self, msg):
        self.state = msg.state

    def on_arm(self, req, res):
        self.armed = True
        res.success, res.message = True, 'next pose will (re)initialize localization'
        self.status('ARMED', res.message)
        return res

    def on_pose(self, msg):
        g = lambda k: self.get_parameter(k).value  # noqa
        if self.busy:
            return
        initialized = self.state == InitState.INITIALIZED
        if initialized and not (self.armed or g('reinitialize_when_initialized')):
            return  # already localized: ignore repeated poses
        if time.time() - self.last_call < g('min_interval_sec'):
            return
        if msg.header.frame_id and msg.header.frame_id != g('map_frame'):
            self.status('REJECTED', f'frame_id {msg.header.frame_id} != {g("map_frame")}')
            return
        if not self.cli.service_is_ready():
            self.status('WAITING', f'{g("initialize_service")} not available', )
            return
        req = InitSrv.Request()
        if hasattr(req, 'pose_with_covariance'):
            req.pose_with_covariance = [msg]
            if hasattr(req, 'method'):
                req.method = InitSrv.Request.AUTO
        else:
            req.pose = [msg]
        self.busy = True
        self.armed = False
        self.last_call = time.time()
        p = msg.pose.pose.position
        self.status('CALLING', f'initialize at x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}')
        fut = self.cli.call_async(req)
        fut.add_done_callback(self.on_done)

    def on_done(self, fut):
        self.busy = False
        try:
            res = fut.result()
        except Exception as e:  # noqa
            self.status('FAILED', f'service call failed: {e}')
            return
        if res.status.success:
            self.status('DONE', 'localization initialized (NDT aligned, EKF reset)')
        else:
            self.status('FAILED', f'code={res.status.code} {res.status.message}')


def main():
    rclpy.init()
    node = InitialPoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
