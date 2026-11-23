import rclpy
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject
from geometry_msgs.msg import PoseStamped

class ClickToObstacle(Node):
    def __init__(self):
        super().__init__('click_to_obstacle')
        self.declare_parameter(
            'checkpoint_pose_topic',
            '/planning/mission_planning/checkpoint'
        )
        self.checkpoint_pose_topic = self.get_parameter(
            'checkpoint_pose_topic'
        ).get_parameter_value().string_value
        self.pub = self.create_publisher(
            DummyObject,
            '/simulation/dummy_perception_publisher/object_info',
            10
        )
        self.sub = self.create_subscription(
            PoseStamped,
            self.checkpoint_pose_topic,
            self.checkpoint_pose_callback,
            10
        )
        self.uuid_counter = 1
        self.get_logger().info(
            f'Ready! Use 2D Checkpoint Pose on {self.checkpoint_pose_topic} '
            'to place UNKNOWN obstacles.'
        )
        self.get_logger().info('Obstacle size: 0.4m x 0.5m x 1.5m (person-sized)')

    def checkpoint_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        orientation = msg.pose.orientation

        uuid = [self.uuid_counter] + [0] * 15
        self.uuid_counter += 1

        # 立即发布一次
        self.publish_obstacle(x, y, z, orientation, uuid, action=0)

        # 持续发布保持障碍物存在
        self.create_timer(
            0.5,
            lambda: self.publish_obstacle(x, y, z, orientation, uuid, action=0)
        )

        self.get_logger().info(
            f'Placed obstacle #{self.uuid_counter - 1} at ({x:.2f}, {y:.2f})'
        )

    def publish_obstacle(self, x, y, z, orientation, uuid, action):
        msg = DummyObject()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'map'
        msg.id.uuid = uuid

        msg.initial_state.pose_covariance.pose.position.x = x
        msg.initial_state.pose_covariance.pose.position.y = y
        msg.initial_state.pose_covariance.pose.position.z = z
        msg.initial_state.pose_covariance.pose.orientation = orientation

        msg.initial_state.twist_covariance.twist.linear.x = 0.0

        msg.classification.label = 0        # UNKNOWN
        msg.classification.probability = 1.0

        msg.shape.type = 0                  # BOUNDING_BOX
        # 人体大小：宽0.5m 深0.4m 高1.5m
        # 与实车录制时观察到的障碍物2尺寸接近
        msg.shape.dimensions.x = 0.4       # 前后方向
        msg.shape.dimensions.y = 0.5       # 左右方向（影响绕行margin计算）
        msg.shape.dimensions.z = 1.5       # 高度

        msg.max_velocity = 0.0
        msg.min_velocity = 0.0
        msg.action = action

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ClickToObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Clearing all obstacles...')
        msg = DummyObject()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id.uuid = [0] * 16
        msg.action = 3  # DELETEALL
        node.pub.publish(msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
