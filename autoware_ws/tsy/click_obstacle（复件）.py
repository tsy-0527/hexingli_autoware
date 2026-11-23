"""
仿真障碍物管理节点（click_to_obstacle 增强版）

障碍物来源：
  1. RViz「2D Checkpoint Pose」：发布到 checkpoint_pose_topic（与原版行为相同，行人尺寸）
  2. 上位机网页：经 rosbridge 发布 std_msgs/String（JSON）到 /web/obstacle/cmd
       {"op": "add", "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0, "size": [0.5, 0.5, 1.5]}
       {"op": "remove", "id": 3}
       {"op": "clear"}
       {"op": "list"}          # 请求立即回传一次列表

障碍物列表回传（上位机据此显示，刷新网页后也能恢复）：
  /web/obstacle/list  std_msgs/String（JSON），变化时立即发布，另外每秒发布一次
       {"obstacles": [{"id": 1, "x": .., "y": .., "z": .., "yaw": .., "size": [dx, dy, dz], "label": "UNKNOWN", "source": "web"}]}

障碍物通过 /simulation/dummy_perception_publisher/object_info 送入 Autoware 仿真
（dummy_perception_publisher 会据此生成点云与感知结果，规划会绕行或停车）。

清空：ros2 service call /click_to_obstacle/clear std_srvs/srv/Trigger {}
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tier4_simulation_msgs.msg import DummyObject
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger

# RViz 点选时的默认尺寸：前后 0.5 m、左右 0.5 m、高 1.5 m（人体大小），与原版一致
# 类别一律为 UNKNOWN（label 只是列表里的显示文字，不影响发给 Autoware 的类别）
DEFAULT_SIZE = (0.5, 0.5, 1.5)
DEFAULT_LABEL = 'UNKNOWN'
# 网页传来的尺寸上下限（米），防止误操作放出巨大或不可见的障碍物
SIZE_MIN, SIZE_MAX = 0.1, 20.0
MAX_OBSTACLES = 200


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ClickToObstacle(Node):
    def __init__(self):
        super().__init__('click_to_obstacle')
        self.declare_parameter('checkpoint_pose_topic', '/planning/mission_planning/checkpoint')
        self.declare_parameter('web_cmd_topic', '/web/obstacle/cmd')
        self.declare_parameter('web_list_topic', '/web/obstacle/list')
        self.checkpoint_pose_topic = self.get_parameter('checkpoint_pose_topic').get_parameter_value().string_value
        web_cmd_topic = self.get_parameter('web_cmd_topic').get_parameter_value().string_value
        web_list_topic = self.get_parameter('web_list_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(DummyObject, '/simulation/dummy_perception_publisher/object_info', 10)

        # 列表用 transient_local：后连上的订阅者（如刚打开的网页）也能马上拿到最新一份
        list_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.list_pub = self.create_publisher(String, web_list_topic, list_qos)

        self.create_subscription(PoseStamped, self.checkpoint_pose_topic, self.checkpoint_pose_callback, 10)
        self.create_subscription(String, web_cmd_topic, self.web_cmd_callback, 10)

        # id -> 障碍物信息
        self.obstacles = {}
        self.uuid_counter = 1
        self.pending_clear = 0  # 还要补发几次 DELETEALL

        # 一个 10 Hz 定时器统一 MODIFY 全部障碍物（保持存在，不反复 ADD 以免闪烁）
        self.create_timer(0.1, self.keepalive)
        # 每秒回传一次列表，兼容不支持 transient_local 的 rosbridge 版本
        self.create_timer(1.0, self.publish_list)

        self.create_service(Trigger, '~/clear', self.clear_obstacles_callback)

        self.get_logger().info(f'Ready! RViz: 2D Checkpoint Pose on {self.checkpoint_pose_topic}')
        self.get_logger().info(f'Web: commands on {web_cmd_topic}, list on {web_list_topic}')
        self.get_logger().info('Clear all obstacles with: ros2 service call /click_to_obstacle/clear std_srvs/srv/Trigger {}')

    # ── 来源 1：RViz 2D Checkpoint Pose（行为与原版相同） ──
    def checkpoint_pose_callback(self, msg):
        p = msg.pose.position
        self.add_obstacle(p.x, p.y, p.z, quaternion_to_yaw(msg.pose.orientation), DEFAULT_SIZE, DEFAULT_LABEL, 'rviz')

    # ── 来源 2：上位机网页 ──
    def web_cmd_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn(f'Invalid web command (not JSON): {msg.data!r}')
            return
        op = cmd.get('op')
        try:
            if op == 'add':
                size = cmd.get('size') or DEFAULT_SIZE
                size = tuple(min(max(float(v), SIZE_MIN), SIZE_MAX) for v in size[:3])
                if len(size) != 3:
                    size = DEFAULT_SIZE
                self.add_obstacle(
                    float(cmd['x']), float(cmd['y']), float(cmd.get('z', 0.0)), float(cmd.get('yaw', 0.0)),
                    size, str(cmd.get('label', DEFAULT_LABEL))[:16], 'web',
                )
            elif op == 'remove':
                self.remove_obstacle(int(cmd['id']))
            elif op == 'clear':
                self.clear_obstacles()
            elif op == 'list':
                self.publish_list()
            else:
                self.get_logger().warn(f'Unknown web command: {cmd}')
        except (KeyError, ValueError, TypeError) as e:
            self.get_logger().warn(f'Bad web command {cmd}: {e}')

    # ── 障碍物管理 ──
    def make_uuid(self, n):
        # uuid 每个元素是 0~255，用两个字节编码编号，超过 255 个也不会出错
        return [n & 0xFF, (n >> 8) & 0xFF] + [0] * 14

    def add_obstacle(self, x, y, z, yaw, size, label, source):
        if len(self.obstacles) >= MAX_OBSTACLES:
            self.get_logger().warn(f'Too many obstacles (>{MAX_OBSTACLES}), ignored')
            return
        oid = self.uuid_counter
        self.uuid_counter += 1
        ob = {'id': oid, 'x': x, 'y': y, 'z': z, 'yaw': yaw, 'size': list(size), 'label': label, 'source': source,
              'uuid': self.make_uuid(oid)}
        self.obstacles[oid] = ob
        self.publish_obstacle(ob, DummyObject.ADD)  # 立即新增一次
        self.get_logger().info(f'Placed obstacle #{oid} ({label}, {source}) at ({x:.2f}, {y:.2f}) '
                               f'size {size[0]:.1f}x{size[1]:.1f}x{size[2]:.1f}')
        self.publish_list()

    def remove_obstacle(self, oid):
        ob = self.obstacles.pop(oid, None)
        if ob is None:
            self.get_logger().warn(f'Obstacle #{oid} not found')
            self.publish_list()
            return
        for _ in range(3):  # 多发几次，确保仿真器收到
            self.publish_obstacle(ob, DummyObject.DELETE)
        self.get_logger().info(f'Removed obstacle #{oid}')
        self.publish_list()

    def clear_obstacles(self):
        self.obstacles.clear()
        self.publish_delete_all()
        self.pending_clear = 4  # 接下来 4 个定时周期再补发，确保仿真器收到（不在回调里阻塞或嵌套 spin）
        self.get_logger().info('Cleared all obstacles.')
        self.publish_list()

    def publish_delete_all(self):
        msg = DummyObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id.uuid = [0] * 16
        msg.action = DummyObject.DELETEALL
        self.pub.publish(msg)

    def keepalive(self):
        if self.pending_clear > 0:
            self.pending_clear -= 1
            self.publish_delete_all()
        for ob in self.obstacles.values():
            self.publish_obstacle(ob, DummyObject.MODIFY)

    def publish_obstacle(self, ob, action):
        msg = DummyObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id.uuid = ob['uuid']

        pose = msg.initial_state.pose_covariance.pose
        pose.position.x = ob['x']
        pose.position.y = ob['y']
        pose.position.z = ob['z']
        pose.orientation = yaw_to_quaternion(ob['yaw'])
        msg.initial_state.twist_covariance.twist.linear.x = 0.0

        msg.classification.label = 0          # UNKNOWN
        msg.classification.probability = 1.0
        msg.shape.type = 0                    # BOUNDING_BOX
        msg.shape.dimensions.x = ob['size'][0]  # 前后方向
        msg.shape.dimensions.y = ob['size'][1]  # 左右方向（影响绕行 margin 计算）
        msg.shape.dimensions.z = ob['size'][2]  # 高度

        msg.max_velocity = 0.0
        msg.min_velocity = 0.0
        msg.action = action
        self.pub.publish(msg)

    def publish_list(self):
        data = {
            'obstacles': [
                {k: ob[k] for k in ('id', 'x', 'y', 'z', 'yaw', 'size', 'label', 'source')}
                for ob in sorted(self.obstacles.values(), key=lambda o: o['id'])
            ],
            'stamp': time.time(),
        }
        self.list_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def clear_obstacles_callback(self, request, response):
        del request
        self.clear_obstacles()
        response.success = True
        response.message = 'Cleared all dummy obstacles.'
        return response


def main():
    rclpy.init()
    node = ClickToObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Clearing all obstacles...')
        # spin 已结束，这里可以直接发并稍作等待
        node.obstacles.clear()
        for _ in range(5):
            node.publish_delete_all()
            time.sleep(0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
