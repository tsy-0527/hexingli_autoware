"""Fault injector: sits between Autoware prediction output and the existence filter.

  input  (param input_topic,  default /perception/object_recognition/objects_raw)  PredictedObjects
  output (param output_topic, default /fj/objects_injected)                         PredictedObjects
  debug  /fj/injector_debug  std_msgs/String (JSON per frame)

Disturbances (all seeded by `seed`, so a run is reproducible):
  dropout_mode = none | burst | iid
      burst: Gilbert-Elliott chain per object (p_enter, p_exit)   -> measured "成段漏检"
      iid:   independent per-frame miss with probability p_miss   -> "独立漏检"
  sigma, rho: AR(1) position jitter (m) added to x and y of every real object
  fp_rate:    probability per frame of spawning a ghost object ahead of ego ("虚警"),
              each ghost lives a random number of frames in [fp_len_min, fp_len_max]
"""
import copy
import json
import math
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import (PredictedObjects, PredictedObject, ObjectClassification,
                                          PredictedPath, Shape)
from builtin_interfaces.msg import Duration

from .common import GilbertElliott, IidDropout, Ar1, seeded_rng, yaw_from_quat


def uuid_hex(obj):
    return bytes(obj.object_id.uuid).hex()


def make_ghost(x, y, yaw, stamp_ghost_id):
    o = PredictedObject()
    o.object_id.uuid = list(uuid.UUID(stamp_ghost_id).bytes)
    o.existence_probability = 1.0
    c = ObjectClassification()
    c.label = ObjectClassification.CAR
    c.probability = 1.0
    o.classification = [c]
    p = o.kinematics.initial_pose_with_covariance.pose
    p.position.x, p.position.y = x, y
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    path = PredictedPath()
    path.time_step = Duration(sec=0, nanosec=500_000_000)
    path.confidence = 1.0
    path.path = [copy.deepcopy(p) for _ in range(10)]
    o.kinematics.predicted_paths = [path]
    o.shape.type = Shape.BOUNDING_BOX
    o.shape.dimensions.x, o.shape.dimensions.y, o.shape.dimensions.z = 4.5, 1.9, 1.6
    return o


class FaultInjector(Node):
    def __init__(self):
        super().__init__('fj_fault_injector')
        P = self.declare_parameter
        self.input_topic = P('input_topic', '/perception/object_recognition/objects_raw').value
        self.output_topic = P('output_topic', '/fj/objects_injected').value
        self.seed = int(P('seed', 0).value)
        self.mode = P('dropout_mode', 'none').value
        self.p_enter = float(P('p_enter', 0.066).value)
        self.p_exit = float(P('p_exit', 0.154).value)
        self.p_miss = float(P('p_miss', 0.3).value)
        self.sigma = float(P('sigma', 0.0).value)
        self.rho = float(P('rho', 0.65).value)
        self.fp_rate = float(P('fp_rate', 0.0).value)
        self.fp_len_min = int(P('fp_len_min', 2).value)
        self.fp_len_max = int(P('fp_len_max', 6).value)
        self.fp_dist_min = float(P('fp_dist_min', 15.0).value)
        self.fp_dist_max = float(P('fp_dist_max', 35.0).value)
        self.fp_lat = float(P('fp_lat', 1.0).value)

        self.rng = seeded_rng(self.seed, 'injector')
        self.state = {}      # uuid hex -> (dropout model, ar1 x, ar1 y)
        self.ghosts = []     # dicts: id, x, y, yaw, left
        self.ego = None

        # publisher RELIABLE, subscription BEST_EFFORT: compatible with either publisher policy upstream
        pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PredictedObjects, self.output_topic, pub_qos)
        self.dbg = self.create_publisher(String, '/fj/injector_debug', 10)
        self.create_subscription(PredictedObjects, self.input_topic, self.cb, sub_qos)
        self.create_subscription(Odometry, '/localization/kinematic_state', self.cb_odom, 10)
        self.get_logger().info(
            f'fault_injector: {self.input_topic} -> {self.output_topic}, mode={self.mode}, sigma={self.sigma}, '
            f'fp_rate={self.fp_rate}, seed={self.seed}')

    def cb_odom(self, msg):
        p = msg.pose.pose
        self.ego = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def model_for(self, key):
        if key not in self.state:
            # seed by order of first appearance (not by the upstream UUID, which changes every run), so the
            # same seed gives the same dropout sequence to every method
            r = seeded_rng(self.seed, f'obj{len(self.state)}')
            if self.mode == 'burst':
                d = GilbertElliott(self.p_enter, self.p_exit, r)
            elif self.mode == 'iid':
                d = IidDropout(self.p_miss, r)
            else:
                d = None
            self.state[key] = (d, Ar1(self.sigma, self.rho, r), Ar1(self.sigma, self.rho, r))
        return self.state[key]

    def cb(self, msg):
        out = PredictedObjects()
        out.header = msg.header
        dropped = []
        for o in msg.objects:
            key = uuid_hex(o)
            d, ex, ey = self.model_for(key)
            detected = True if d is None else d.step()
            nx, ny = ex.step(), ey.step()
            if not detected:
                dropped.append(key[:8])
                continue
            o2 = copy.deepcopy(o)
            if nx or ny:
                o2.kinematics.initial_pose_with_covariance.pose.position.x += nx
                o2.kinematics.initial_pose_with_covariance.pose.position.y += ny
                for path in o2.kinematics.predicted_paths:
                    for ps in path.path:
                        ps.position.x += nx
                        ps.position.y += ny
            out.objects.append(o2)

        # ghosts (false positives)
        if self.fp_rate > 0 and self.ego is not None and self.rng.random() < self.fp_rate:
            ex_, ey_, eyaw = self.ego
            dist = self.rng.uniform(self.fp_dist_min, self.fp_dist_max)
            lat = self.rng.uniform(-self.fp_lat, self.fp_lat)
            gx = ex_ + math.cos(eyaw) * dist - math.sin(eyaw) * lat
            gy = ey_ + math.sin(eyaw) * dist + math.cos(eyaw) * lat
            gid = str(uuid.UUID(int=self.rng.getrandbits(128)))
            self.ghosts.append(dict(id=gid, x=gx, y=gy, yaw=eyaw,
                                    left=self.rng.randint(self.fp_len_min, self.fp_len_max)))
        alive = []
        for g in self.ghosts:
            out.objects.append(make_ghost(g['x'], g['y'], g['yaw'], g['id']))
            g['left'] -= 1
            if g['left'] > 0:
                alive.append(g)
        ghost_ids = [g['id'][:8] for g in self.ghosts]
        self.ghosts = alive

        self.pub.publish(out)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.dbg.publish(String(data=json.dumps(dict(t=t, n_in=len(msg.objects), dropped=dropped, ghosts=ghost_ids))))


def main():
    rclpy.init()
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
