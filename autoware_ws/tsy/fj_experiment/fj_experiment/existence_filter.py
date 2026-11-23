"""Existence filter: improvement 1 (existence CUSUM) and improvement 2 (graded response).

  input  (param input_topic,  default /fj/objects_injected)                   PredictedObjects
  output (param output_topic, default /perception/object_recognition/objects) PredictedObjects -> planning
  debug  /fj/filter_debug   std_msgs/String (JSON per frame)
  speed  (param vlim_topic,  default /planning/scenario_planning/max_velocity_candidates)  VelocityLimit (mode=graded)

mode:
  passthrough  M0  output = input (Autoware default behaviour)
  khold        M1  a confirmed object is kept for `hold_frames` missed frames, then deleted
  cusum        M2  a confirmed object is kept until its existence CUSUM S >= h            (improvement 1)
  graded       M3  S < s_soft: hard (published);  s_soft <= S < h: soft (not published, speed limited);
                   S >= h: deleted                                                       (improvement 1 + 2)

Tentative objects (seen < confirm_hits frames) are published only on frames where they are detected and
are dropped after `tentative_drop` consecutive misses, in every mode, so short-lived ghosts are not kept.
Association is by position (gate `gate` m), not by upstream UUID, because a long dropout can make the
upstream tracker re-issue a new UUID for the same obstacle.
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
from autoware_perception_msgs.msg import PredictedObjects

from .common import ExistenceCusum, yaw_from_quat

try:  # message moved between Autoware versions
    from autoware_internal_planning_msgs.msg import VelocityLimit
except ImportError:  # pragma: no cover
    try:
        from tier4_planning_msgs.msg import VelocityLimit
    except ImportError:
        VelocityLimit = None


def obj_xy(o):
    p = o.kinematics.initial_pose_with_covariance.pose.position
    return p.x, p.y


class Track:
    def __init__(self, obj, cusum_kw, rng_uuid):
        self.id = rng_uuid
        self.obj = obj
        self.x, self.y = obj_xy(obj)
        self.hits, self.miss, self.confirmed = 1, 0, False
        self.cusum = ExistenceCusum(**cusum_kw)
        self.matched = True
        self.deleted = False


class ExistenceFilter(Node):
    def __init__(self):
        super().__init__('fj_existence_filter')
        P = self.declare_parameter
        self.input_topic = P('input_topic', '/fj/objects_injected').value
        self.output_topic = P('output_topic', '/perception/object_recognition/objects').value
        self.mode = P('mode', 'passthrough').value
        self.gate = float(P('gate', 2.0).value)
        self.confirm_hits = int(P('confirm_hits', 3).value)
        self.tentative_drop = int(P('tentative_drop', 2).value)
        self.hold_frames = int(P('hold_frames', 10).value)
        self.cusum_kw = dict(p_fa=float(P('p_fa', 0.05).value), pd_good=float(P('pd_good', 0.99).value),
                             pd_bad=float(P('pd_bad', 0.0).value), p_enter=float(P('p_enter', 0.066).value),
                             p_exit=float(P('p_exit', 0.154).value), h=float(P('h', 8.0).value))
        self.s_soft = float(P('s_soft', 3.5).value)
        self.a_soft = float(P('a_soft', 3.0).value)       # m/s^2, braking the soft limit must allow
        self.v_floor = float(P('v_floor', 1.0).value)     # m/s, never limit below this
        self.d_stop = float(P('d_stop', 4.0).value)       # m, stop margin in front of the object
        self.ego_front = float(P('ego_front', 3.79).value)  # base_link -> front bumper (sample_vehicle)
        self.v_release = float(P('v_release', 100.0).value)
        # soft_publish=false: a soft object is NOT sent to planning (only the speed limit acts) -- the design
        # in the report.  true: it is still sent (path keeps avoiding) AND the speed limit acts (ablation).
        self.soft_publish = bool(P('soft_publish', False).value)
        self.vlim_topic = P('vlim_topic', '/planning/scenario_planning/max_velocity_candidates').value

        self.tracks = []
        self.ego = None
        self._n = 0
        pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PredictedObjects, self.output_topic, pub_qos)
        self.dbg = self.create_publisher(String, '/fj/filter_debug', 10)
        self.vpub = None
        if self.mode == 'graded':
            if VelocityLimit is None:
                self.get_logger().error('VelocityLimit message not found; graded mode will not limit speed')
            else:
                self.vpub = self.create_publisher(VelocityLimit, self.vlim_topic, 10)
        self.create_subscription(PredictedObjects, self.input_topic, self.cb, sub_qos)
        self.create_subscription(Odometry, '/localization/kinematic_state', self.cb_odom, 10)
        self.get_logger().info(f'existence_filter: mode={self.mode} {self.input_topic} -> {self.output_topic} '
                               f'cusum={self.cusum_kw} s_soft={self.s_soft} hold={self.hold_frames}')

    def cb_odom(self, msg):
        p = msg.pose.pose
        self.ego = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def new_id(self):
        self._n += 1
        return list(uuid.uuid5(uuid.NAMESPACE_OID, f'fj-{self._n}').bytes)

    # ------------------------------------------------------------------
    def associate(self, objs):
        dets = [(o, *obj_xy(o)) for o in objs]
        pairs = []
        for ti, t in enumerate(self.tracks):
            for di, (_, x, y) in enumerate(dets):
                d = math.hypot(x - t.x, y - t.y)
                if d <= self.gate:
                    pairs.append((d, ti, di))
        pairs.sort()
        used_t, used_d = set(), set()
        for t in self.tracks:
            t.matched = False
        for d, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            t = self.tracks[ti]
            t.obj, t.x, t.y = dets[di][0], dets[di][1], dets[di][2]
            t.matched = True
        for di, (o, x, y) in enumerate(dets):
            if di not in used_d:
                self.tracks.append(Track(o, self.cusum_kw, self.new_id()))

    def step_tracks(self):
        for t in self.tracks:
            if t.matched:
                t.miss = 0
                if not t.confirmed:
                    t.hits += 1
                    if t.hits >= self.confirm_hits:
                        t.confirmed = True
                        t.cusum.reset()
                else:
                    t.cusum.update(True)
            else:
                t.miss += 1
                if not t.confirmed:
                    if t.miss >= self.tentative_drop:
                        t.deleted = True
                else:
                    t.cusum.update(False)
                    if self.mode == 'khold' and t.miss > self.hold_frames:
                        t.deleted = True
                    elif self.mode in ('cusum', 'graded') and t.cusum.S >= t.cusum.h:
                        t.deleted = True
                    elif self.mode == 'passthrough':
                        t.deleted = True

    def status(self, t):
        if t.deleted:
            return 'del'
        if not t.confirmed:
            return 'tent'
        if self.mode == 'graded' and t.cusum.S >= self.s_soft:
            return 'soft'
        return 'hard'

    def published(self, t, st):
        if st == 'del':
            return False
        if st == 'soft':
            return self.soft_publish
        if st == 'tent':
            return t.matched
        if self.mode == 'passthrough':
            return t.matched
        return True

    # ------------------------------------------------------------------
    def cb(self, msg):
        self.associate(msg.objects)
        self.step_tracks()

        out = PredictedObjects()
        out.header = msg.header
        dbg_tracks, soft = [], []
        for t in self.tracks:
            st = self.status(t)
            pub = self.published(t, st)
            if self.mode != 'passthrough' and pub:
                o = copy.deepcopy(t.obj)
                o.object_id.uuid = t.id
                out.objects.append(o)
            if st == 'soft':
                soft.append(t)
            dbg_tracks.append(dict(id=bytes(t.id).hex()[:8], x=round(t.x, 2), y=round(t.y, 2), st=st,
                                   S=round(t.cusum.S, 3), miss=t.miss, pub=pub))
        if self.mode == 'passthrough':
            out = msg
        self.tracks = [t for t in self.tracks if not t.deleted]
        self.pub.publish(out)

        vlim = None
        if self.vpub is not None:
            vlim = self.v_release
            if self.ego is not None:
                ex, ey, eyaw = self.ego
                for t in soft:
                    ahead = (t.x - ex) * math.cos(eyaw) + (t.y - ey) * math.sin(eyaw)
                    if ahead <= 0:
                        continue
                    half_len = t.obj.shape.dimensions.x / 2
                    gap = ahead - self.ego_front - half_len - self.d_stop
                    v = math.sqrt(self.v_floor ** 2 + 2 * self.a_soft * max(0.0, gap))
                    vlim = min(vlim, v)
            m = VelocityLimit()
            m.stamp = self.get_clock().now().to_msg()
            m.max_velocity = float(vlim)
            m.use_constraints = False
            m.sender = 'fj_existence_filter'
            self.vpub.publish(m)

        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.dbg.publish(String(data=json.dumps(dict(t=t_s, mode=self.mode, n_in=len(msg.objects),
                                                     n_out=len(out.objects), vlim=vlim, tracks=dbg_tracks))))


def main():
    rclpy.init()
    node = ExistenceFilter()
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
