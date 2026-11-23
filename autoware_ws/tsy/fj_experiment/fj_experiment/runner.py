"""Scenario runner for Autoware planning_simulator.

Two sub-commands:

  teach  -- record a scenario once by clicking in RViz (2D Pose Estimate, 2D Goal Pose, one dummy car).
            ros2 run fj_experiment runner teach --name straight
            -> ~/fj_ws/scenarios/straight/{initialpose,goal,object}.bin + meta.json

  run    -- replay the scenario automatically for every (scenario, method, seed) combination,
            start the fault injector + existence filter with the right parameters, engage, log, reset.
            ros2 run fj_experiment runner run --name straight --scenarios S0 S1 --methods M0 M2 --seeds 0-9

Each run is written to ~/fj_ws/results/<name>/<S>_<M>_seed<k>.json and evaluated by `evaluate`.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import serialize_message, deserialize_message
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Trajectory
from tier4_simulation_msgs.msg import DummyObject
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, ClearRoute

from .common import yaw_from_quat

HOME = os.path.expanduser('~/fj_ws')


def load_config(path=None):
    if path is None:
        path = os.path.join(get_package_share_directory('fj_experiment'), 'config', 'experiments.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def parse_seeds(items):
    out = []
    for it in items:
        if '-' in it:
            a, b = it.split('-')
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(it))
    return out


def ros_args(params):
    args = []
    for k, v in params.items():
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        args += ['-p', f'{k}:={v}']
    return args


class Runner(Node):
    def __init__(self, cfg):
        super().__init__('fj_runner')
        self.cfg = cfg
        r = cfg['run']
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, r['initialpose_topic'], 10)
        self.pub_goal = self.create_publisher(PoseStamped, r['goal_topic'], 10)
        self.pub_obj = self.create_publisher(DummyObject, r['object_topic'], 10)
        self.cli_auto = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        self.cli_stop = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_stop')
        self.cli_clear = self.create_client(ClearRoute, '/api/routing/clear_route')
        self.create_subscription(Odometry, '/localization/kinematic_state', self.cb_odom, be)
        self.create_subscription(Trajectory, r['traj_topic'], self.cb_traj, be)
        self.create_subscription(PredictedObjects, r['final_topic'], self.cb_final, be)
        self.create_subscription(String, '/fj/filter_debug', lambda m: self.log_dbg('fdbg', m), 50)
        self.create_subscription(String, '/fj/injector_debug', lambda m: self.log_dbg('idbg', m), 50)
        # teach-mode subscriptions
        self.taught = {}
        self.create_subscription(PoseWithCovarianceStamped, r['initialpose_topic'],
                                 lambda m: self.teach_cb('initialpose', m), 10)
        self.create_subscription(PoseStamped, r['goal_topic'], lambda m: self.teach_cb('goal', m), 10)
        self.create_subscription(DummyObject, r['object_topic'], lambda m: self.teach_cb('object', m), 10)
        self.teaching = False
        self.ego = None
        self.reset_log()

    # ---------------- logging ----------------
    def reset_log(self):
        self.log = dict(odom=[], traj=[], objs=[], fdbg=[], idbg=[], events={})
        self.recording = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def cb_odom(self, m):
        p = m.pose.pose
        self.ego = (p.position.x, p.position.y, yaw_from_quat(p.orientation), m.twist.twist.linear.x)
        if self.recording:
            self.log['odom'].append([self.stamp(m.header), round(p.position.x, 3), round(p.position.y, 3),
                                     round(yaw_from_quat(p.orientation), 4), round(m.twist.twist.linear.x, 3),
                                     round(m.twist.twist.angular.z, 4)])

    def cb_traj(self, m):
        if not self.recording:
            return
        pts = [[round(q.pose.position.x, 3), round(q.pose.position.y, 3)] for q in m.points[::2][:150]]
        self.log['traj'].append([self.stamp(m.header), pts])

    def cb_final(self, m):
        if not self.recording:
            return
        xy = [[round(o.kinematics.initial_pose_with_covariance.pose.position.x, 2),
               round(o.kinematics.initial_pose_with_covariance.pose.position.y, 2)] for o in m.objects]
        self.log['objs'].append([self.stamp(m.header), xy])

    def log_dbg(self, key, m):
        if self.recording:
            self.log[key].append(m.data)

    # ---------------- teach ----------------
    def teach_cb(self, key, m):
        if not self.teaching:
            return
        self.taught[key] = m
        self.get_logger().info(f'[teach] got {key}')

    # ---------------- helpers ----------------
    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, cli, req, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        end = time.time() + timeout
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    def engage(self, tries=15):
        for _ in range(tries):
            res = self.call(self.cli_auto, ChangeOperationMode.Request())
            if res is not None and res.status.success:
                return True
            self.spin_for(1.0)
        return False


# ---------------------------------------------------------------------------
def cmd_teach(args, cfg):
    node = Runner(cfg)
    node.teaching = True
    d = os.path.join(HOME, 'scenarios', args.name)
    os.makedirs(d, exist_ok=True)
    print('\n在 RViz 中依次操作：\n  1) 2D Pose Estimate 设置起点\n  2) 2D Goal Pose 设置终点\n'
          '  3) 用 2D Dummy Car 在车道上放一辆车（障碍物）\n三项都收到后自动保存。Ctrl+C 退出。\n')
    try:
        while rclpy.ok() and not all(k in node.taught for k in ('initialpose', 'goal', 'object')):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        return
    for k in ('initialpose', 'goal', 'object'):
        with open(os.path.join(d, f'{k}.bin'), 'wb') as f:
            f.write(serialize_message(node.taught[k]))
    o = node.taught['object']
    p = o.initial_state.pose_covariance.pose
    meta = dict(obstacle=dict(x=p.position.x, y=p.position.y, yaw=yaw_from_quat(p.orientation),
                              length=o.shape.dimensions.x, width=o.shape.dimensions.y),
                goal=dict(x=node.taught['goal'].pose.position.x, y=node.taught['goal'].pose.position.y))
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print(f'已保存到 {d}\n{json.dumps(meta, indent=1)}')


def start_proc(exe, params, logf):
    cmd = ['ros2', 'run', 'fj_experiment', exe, '--ros-args'] + ros_args(params)
    return subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, preexec_fn=os.setsid)


def stop_proc(p):
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass


def cmd_run(args, cfg):
    node = Runner(cfg)
    rc = cfg['run']
    d = os.path.join(HOME, 'scenarios', args.name)
    msgs = {}
    for k, typ in (('initialpose', PoseWithCovarianceStamped), ('goal', PoseStamped), ('object', DummyObject)):
        with open(os.path.join(d, f'{k}.bin'), 'rb') as f:
            msgs[k] = deserialize_message(f.read(), typ)
    meta = json.load(open(os.path.join(d, 'meta.json')))
    out_dir = os.path.join(HOME, 'results', args.name)
    os.makedirs(out_dir, exist_ok=True)
    log_dir = os.path.join(out_dir, 'node_logs')
    os.makedirs(log_dir, exist_ok=True)

    scen = args.scenarios or list(cfg['scenarios'])
    meth = args.methods or list(cfg['methods'])
    seeds = parse_seeds(args.seeds) if args.seeds else cfg['seeds']
    jobs = [(s, m, k) for s in scen for m in meth for k in seeds]
    print(f'共 {len(jobs)} 次运行，预计 {len(jobs) * (rc["run_time"] + 15) / 60:.0f} 分钟')

    for i, (s, m, k) in enumerate(jobs):
        fname = os.path.join(out_dir, f'{s}_{m}_seed{k}.json')
        if os.path.exists(fname) and not args.overwrite:
            print(f'[{i + 1}/{len(jobs)}] {s} {m} seed{k} 已存在，跳过')
            continue
        print(f'[{i + 1}/{len(jobs)}] {s} {m} seed{k}')
        sc, me = cfg['scenarios'][s], cfg['methods'][m]

        # 1) reset: stop, remove objects, teleport to start, clear route, set goal
        node.call(node.cli_stop, ChangeOperationMode.Request())
        clr = DummyObject()
        clr.header.frame_id = 'map'
        clr.header.stamp = node.get_clock().now().to_msg()
        clr.action = DummyObject.DELETEALL
        node.pub_obj.publish(clr)
        node.spin_for(1.0)
        ip = msgs['initialpose']
        ip.header.stamp = node.get_clock().now().to_msg()
        node.pub_init.publish(ip)
        node.spin_for(rc['settle_time'])
        node.call(node.cli_clear, ClearRoute.Request())
        node.spin_for(1.0)
        g = msgs['goal']
        g.header.stamp = node.get_clock().now().to_msg()
        node.pub_goal.publish(g)
        node.spin_for(rc['settle_time'])

        # 2) start injector + filter for this combination
        inj_p = dict(cfg['injector_common'], **sc.get('injector', {}), seed=k)
        flt_p = dict(cfg['filter_common'], **me)
        lf = open(os.path.join(log_dir, f'{s}_{m}_seed{k}.log'), 'w')
        p_inj = start_proc('fault_injector', inj_p, lf)
        p_flt = start_proc('existence_filter', flt_p, lf)
        node.spin_for(3.0)

        # 3) obstacle
        obj = None
        if sc.get('obstacle', True):
            obj = msgs['object']
            obj.id.uuid = list(uuid.uuid4().bytes)
            obj.header.stamp = node.get_clock().now().to_msg()
            obj.action = DummyObject.ADD
            node.pub_obj.publish(obj)
            node.spin_for(2.0)

        # 4) engage and record
        node.reset_log()
        node.recording = True
        ok = node.engage()
        t0 = node.now()
        node.log['events'] = dict(engaged=ok, t_engage=t0, scenario=s, method=m, seed=k,
                                  obstacle=meta['obstacle'] if obj is not None else None,
                                  injector=inj_p, filter=flt_p)
        if not ok:
            print('  !! 无法切换到自动驾驶模式（检查路线是否设置成功），本次跳过')
        deleted = False
        while ok and node.now() - t0 < rc['run_time'] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if obj is not None and not deleted and sc.get('delete_ahead') and node.ego is not None:
                ex, ey, eyaw, _ = node.ego
                ob = meta['obstacle']
                ahead = (ob['x'] - ex) * math.cos(eyaw) + (ob['y'] - ey) * math.sin(eyaw)
                if ahead < sc['delete_ahead']:
                    dm = DummyObject()
                    dm.header.frame_id = 'map'
                    dm.header.stamp = node.get_clock().now().to_msg()
                    dm.id = obj.id
                    dm.action = DummyObject.DELETE
                    node.pub_obj.publish(dm)
                    node.log['events']['t_delete'] = node.now()
                    deleted = True
        node.recording = False

        # 5) tear down
        node.call(node.cli_stop, ChangeOperationMode.Request())
        stop_proc(p_inj)
        stop_proc(p_flt)
        lf.close()
        if ok:
            with open(fname, 'w') as f:
                json.dump(node.log, f)
            print(f'  已保存 {fname}（{len(node.log["traj"])} 帧轨迹）')
    print('全部完成')


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    t = sub.add_parser('teach')
    t.add_argument('--name', required=True)
    t.add_argument('--config')
    r = sub.add_parser('run')
    r.add_argument('--name', required=True)
    r.add_argument('--config')
    r.add_argument('--scenarios', nargs='*')
    r.add_argument('--methods', nargs='*')
    r.add_argument('--seeds', nargs='*', help='例如 0-9 或 0 1 2')
    r.add_argument('--overwrite', action='store_true')
    args, _ = ap.parse_known_args()
    cfg = load_config(args.config)
    rclpy.init(args=sys.argv)
    try:
        (cmd_teach if args.cmd == 'teach' else cmd_run)(args, cfg)
    except KeyboardInterrupt:
        print('中断')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
