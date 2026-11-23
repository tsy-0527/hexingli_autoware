#!/usr/bin/env python3

import uuid as uuid_lib

import rclpy
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject
from geometry_msgs.msg import PoseStamped


class ClickToObstacle(Node):
    """
    Click a 2D Checkpoint Pose in RViz and add one static UNKNOWN obstacle.

    Key fix:
      - ADD is published only once per obstacle.
      - No per-obstacle timer is created.
      - Dummy Perception owns the object lifecycle after ADD.
    """

    ACTION_ADD = 0
    ACTION_DELETEALL = 3

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

        self.obstacle_count = 0

        self.get_logger().info(
            f'Ready! Use 2D Checkpoint Pose on {self.checkpoint_pose_topic} '
            'to place UNKNOWN obstacles.'
        )
        self.get_logger().info(
            'Obstacle size: 0.4m x 0.5m x 1.5m'
        )
        self.get_logger().info(
            'Fixed mode: each obstacle is ADDed once; no periodic refresh timer.'
        )

    @staticmethod
    def make_uuid():
        # DummyObject UUID is 16 uint8 values.
        return list(uuid_lib.uuid4().bytes)

    def checkpoint_pose_callback(self, msg):
        obstacle_uuid = self.make_uuid()

        self.publish_obstacle(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
            orientation=msg.pose.orientation,
            obstacle_uuid=obstacle_uuid,
            action=self.ACTION_ADD,
        )

        self.obstacle_count += 1

        self.get_logger().info(
            f'Placed obstacle #{self.obstacle_count} at '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}); '
            'published ADD once.'
        )

    def publish_obstacle(
        self,
        x,
        y,
        z,
        orientation,
        obstacle_uuid,
        action,
    ):
        msg = DummyObject()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id.uuid = obstacle_uuid

        msg.initial_state.pose_covariance.pose.position.x = x
        msg.initial_state.pose_covariance.pose.position.y = y
        msg.initial_state.pose_covariance.pose.position.z = z
        msg.initial_state.pose_covariance.pose.orientation = orientation

        # Static obstacle
        msg.initial_state.twist_covariance.twist.linear.x = 0.0

        # UNKNOWN
        msg.classification.label = 0
        msg.classification.probability = 1.0

        # BOUNDING_BOX
        msg.shape.type = 0
        msg.shape.dimensions.x = 0.4
        msg.shape.dimensions.y = 0.5
        msg.shape.dimensions.z = 1.5

        msg.max_velocity = 0.0
        msg.min_velocity = 0.0
        msg.action = action

        self.pub.publish(msg)

    def delete_all_obstacles(self):
        msg = DummyObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id.uuid = [0] * 16
        msg.action = self.ACTION_DELETEALL

        self.pub.publish(msg)
        self.get_logger().info('DELETEALL published.')


def main():
    rclpy.init()
    node = ClickToObstacle()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Clearing all obstacles...')
        node.delete_all_obstacles()

        # Give DDS a short opportunity to send the final DELETEALL.
        rclpy.spin_once(node, timeout_sec=0.2)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
