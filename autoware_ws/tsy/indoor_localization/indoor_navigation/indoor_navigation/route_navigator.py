"""Station-level interface for the UI / Agent (rosbridge friendly, no custom msgs).

Commands  : ~/command  (std_msgs/String JSON)
              {"cmd": "goto", "station": "hall_station_1"}
              {"cmd": "cancel"}
            ~/cancel   (std_srvs/Trigger)  - same as cancel, for a one-click stop
Status    : ~/status   (std_msgs/String JSON, transient local)
              state: IDLE / NAVIGATING / SUCCEEDED / FAILED / CANCELED / REJECTED
Stations  : config/stations.yaml (route graph)

A "goto" is rejected while localization health is ERROR. The route is sent to Nav2
as NavigateThroughPoses; Hybrid-A* plans between the waypoints.

修正：融合导航时，只有 hybrid_mode_manager 报告 mode=indoor 且 phase=active（室内已拿到
底盘权限）才接受 goto；权限被收回（切换开始或状态断流）时取消正在执行的 Nav2 任务。
原来在室外也能下发室内目标，Nav2 控制器空转、进度检查失败后会触发倒车等恢复行为。
"""
import time
import json
import math

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .route_graph import RouteGraph


class RouteNavigator(Node):
    def __init__(self):
        super().__init__('route_navigator')
        d = self.declare_parameter
        d('stations_file', '')
        d('odom_topic', '/localization/kinematic_state')
        d('health_topic', '/indoor_localization/health')
        d('map_frame', 'map')
        d('action_name', 'navigate_through_poses')
        d('behavior_tree', '')          # empty = bt_navigator default
        d('reject_when_health_error', True)
        d('require_indoor_authority', True)
        d('hybrid_state_topic', '/hybrid_mode_manager/state')
        d('hybrid_state_timeout', 1.0)
        g = lambda k: self.get_parameter(k).value  # noqa
        self.g = g
        with open(g('stations_file')) as f:
            self.graph = RouteGraph.from_dict(yaml.safe_load(f))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '~/status', latched)
        self.create_subscription(String, '~/command', self.on_command, 10)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        self.create_subscription(String, g('health_topic'), self.on_health, 10)
        self.create_service(Trigger, '~/cancel', self.on_cancel_srv)
        self.hybrid = None          # (wall time, dict)
        if g('require_indoor_authority'):
            self.create_subscription(String, g('hybrid_state_topic'), self.on_hybrid, 10)
            self.create_timer(0.2, self.check_authority)
        self.client = ActionClient(self, NavigateThroughPoses, g('action_name'))
        self.pose = None
        self.health = None
        self.goal_handle = None
        self.state = dict(state='IDLE', stations=sorted(self.graph.stations))
        self.publish()

    # ---------------- inputs ----------------
    def on_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_health(self, m):
        try:
            self.health = json.loads(m.data)
        except ValueError:
            pass

    def on_hybrid(self, m):
        try:
            self.hybrid = (time.monotonic(), json.loads(m.data))
        except ValueError:
            pass

    def has_authority(self):
        """(ok, reason)"""
        if not self.g('require_indoor_authority'):
            return True, ''
        if self.hybrid is None or time.monotonic() - self.hybrid[0] > self.g('hybrid_state_timeout'):
            return False, 'no state from hybrid_mode_manager'
        h = self.hybrid[1]
        if h.get('mode') != 'indoor' or h.get('phase') != 'active':
            return False, (f'no indoor control authority (mode={h.get("mode")}, phase={h.get("phase")}); '
                           'call /hybrid_mode_manager/set_indoor true and wait for phase=active')
        return True, ''

    def check_authority(self):
        if self.goal_handle is None or self.state.get('state') == 'CANCELING':
            return
        ok, why = self.has_authority()
        if not ok:
            self.get_logger().warn(f'cancel Nav2 goal: {why}')
            self.goal_handle.cancel_goal_async()
            self.update('CANCELING', reason=why)

    def on_command(self, m):
        try:
            c = json.loads(m.data)
        except ValueError:
            return self.update('REJECTED', reason='command is not JSON')
        if c.get('cmd') == 'goto':
            self.goto(str(c.get('station', '')))
        elif c.get('cmd') == 'cancel':
            self.cancel()
        else:
            self.update('REJECTED', reason=f'unknown cmd {c.get("cmd")}')

    def on_cancel_srv(self, req, res):
        res.success = self.cancel()
        res.message = 'cancel requested' if res.success else 'nothing to cancel'
        return res

    # ---------------- actions ----------------
    def goto(self, station):
        if station not in self.graph.stations:
            return self.update('REJECTED', station=station, reason='unknown station')
        if self.pose is None:
            return self.update('REJECTED', station=station, reason='no localization')
        ok, why = self.has_authority()
        if not ok:
            return self.update('REJECTED', station=station, reason=why)
        if self.g('reject_when_health_error') and self.health and self.health.get('level') == 'ERROR':
            return self.update('REJECTED', station=station,
                               reason='localization ERROR: ' + ', '.join(self.health.get('reasons', [])))
        if self.goal_handle is not None:
            return self.update('REJECTED', station=station, reason='already navigating, cancel first')
        if not self.client.wait_for_server(timeout_sec=2.0):
            return self.update('REJECTED', station=station, reason='Nav2 navigate_through_poses not available')
        nodes, wps = self.graph.route(self.pose[0], self.pose[1], station)
        if not wps:
            return self.update('REJECTED', station=station, reason='no route in station graph')
        goal = NavigateThroughPoses.Goal()
        now = self.get_clock().now().to_msg()
        for x, y, yaw in wps:
            p = PoseStamped()
            p.header.frame_id = self.g('map_frame')
            p.header.stamp = now
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            goal.poses.append(p)
        goal.behavior_tree = self.g('behavior_tree')
        self.update('NAVIGATING', station=station, route=nodes, reason='')
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.goal_handle = None
            return self.update('REJECTED', reason='Nav2 rejected the goal')
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, fb):
        f = fb.feedback
        self.state['distance_remaining'] = round(float(f.distance_remaining), 2)
        self.state['poses_remaining'] = int(f.number_of_poses_remaining)
        self.state['recoveries'] = int(f.number_of_recoveries)
        self.publish()

    def on_result(self, fut):
        status = fut.result().status
        self.goal_handle = None
        name = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_ABORTED: 'FAILED'}.get(status, 'FAILED')
        self.update(name, reason='' if name != 'FAILED' else f'nav2 status {status}')

    def cancel(self):
        if self.goal_handle is None:
            return False
        self.goal_handle.cancel_goal_async()
        self.update('CANCELING')
        return True

    # ---------------- status ----------------
    def update(self, state, **kw):
        self.state.update(state=state, **kw)
        self.publish()

    def publish(self):
        self.status_pub.publish(String(data=json.dumps(self.state, ensure_ascii=False)))
        self.get_logger().info(json.dumps(self.state, ensure_ascii=False))


def main():
    rclpy.init()
    node = RouteNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
