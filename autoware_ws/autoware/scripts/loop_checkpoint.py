#!/usr/bin/env python3
# multi_waypoint_route.py
#
# 使用方式（例如设置 3 个中途点）：
#   1. python3 loop_checkpoint.py --ros-args -p num_waypoints:=3
#   2. Rviz 点 2D Pose Estimate
#   3. 按行驶顺序点击 3 次 2D Checkpoint Pose（中途点）
#   4. 再点击 1 次 2D Checkpoint Pose（最终终点）
#   5. 路线设置成功后点 Autonomous
#
# 前 num_waypoints 个点通过 SetRoutePoints.waypoints 一次性发送，
# 最后一个点通过 SetRoutePoints.goal 发送。车辆按顺序经过所有
# 中途点且不会仅仅因为途经点而停车，最后在最终终点停车并结束。

import math

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


# autoware_adapi_v1_msgs/msg/RouteState:
# UNKNOWN=0, UNSET=1, SET=2, ARRIVED=3, CHANGING=4
ROUTE_ARRIVED = 3

# autoware_adapi_v1_msgs/msg/OperationModeState
MODE_AUTONOMOUS = 2


class MultiWaypointRoute(Node):
    def __init__(self):
        # 保留旧节点名，避免影响现有启动或监控脚本。
        super().__init__('checkpoint_loop')
        cb = ReentrantCallbackGroup()

        self.declare_parameter('num_waypoints', 2)
        self.declare_parameter('arrive_threshold', 8.0)

        self.num_waypoints = int(self.get_parameter('num_waypoints').value)
        self.arrive_threshold = float(
            self.get_parameter('arrive_threshold').value
        )

        if self.num_waypoints < 0:
            self.get_logger().warn(
                'num_waypoints 不能小于 0，已自动改为 0'
            )
            self.num_waypoints = 0

        self.total_points = self.num_waypoints + 1

        # 前 num_waypoints 个是中途点，最后一个是唯一最终终点。
        self.route_points = []
        self.sending = False
        self.started = False
        self.route_set = False
        self.arrival_logged = False
        self.auto_engaged = False
        self.route_state_val = -1
        self.current_x = None
        self.current_y = None

        self.create_subscription(
            PoseStamped,
            '/planning/mission_planning/publishpoint',
            self.on_checkpoint,
            10,
            callback_group=cb,
        )
        self.create_subscription(
            RouteState,
            '/api/routing/state',
            self.on_state,
            10,
            callback_group=cb,
        )
        self.create_subscription(
            OperationModeState,
            '/api/operation_mode/state',
            self.on_operation_mode,
            10,
            callback_group=cb,
        )
        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.on_odom,
            10,
            callback_group=cb,
        )

        self.route_cli = self.create_client(
            SetRoutePoints,
            '/api/routing/set_route_points',
            callback_group=cb,
        )

        self.create_timer(10.0, self.print_status)

        self.get_logger().info('=' * 58)
        self.get_logger().info('🚗 单次多途经点路线')
        self.get_logger().info(
            f'  中途点数量: {self.num_waypoints}，最终终点数量: 1'
        )
        self.get_logger().info(
            f'  请按行驶顺序点击 {self.total_points} 个 Checkpoint Pose'
        )
        self.get_logger().info('  经过所有中途点后，在最后一个点停车并结束')
        self.get_logger().info('=' * 58)

    def print_status(self):
        if not self.started:
            self.get_logger().info(
                f'[等待点位] 已点击 {len(self.route_points)}/'
                f'{self.total_points}'
            )
            return

        dist = self.dist_to_goal()
        dist_text = f'{dist:.1f}m' if dist is not None else '未知'
        phase = '已完成' if self.arrival_logged else '行驶中'
        self.get_logger().info(
            f'[状态] {phase} route={self.route_state_val} '
            f'auto={self.auto_engaged} '
            f'dist_to_goal={dist_text} '
            f'waypoints={self.num_waypoints}'
        )

    def on_checkpoint(self, msg: PoseStamped):
        if self.started or len(self.route_points) >= self.total_points:
            self.get_logger().warn('⚠️ 路线点已经收集完成，忽略新的点')
            return

        self.route_points.append(msg.pose)
        idx = len(self.route_points)

        if idx <= self.num_waypoints:
            self.get_logger().info(
                f'📍 中途点 {idx}/{self.num_waypoints}: '
                f'x={msg.pose.position.x:.2f}, '
                f'y={msg.pose.position.y:.2f}'
            )
        else:
            self.get_logger().info(
                f'🏁 最终终点: x={msg.pose.position.x:.2f}, '
                f'y={msg.pose.position.y:.2f}'
            )

        remaining = self.total_points - idx
        if remaining > 0:
            next_type = '中途点' if idx < self.num_waypoints else '最终终点'
            self.get_logger().info(
                f'👉 请继续点击 {next_type}，还需 {remaining} 个点'
            )
            return

        self.started = True
        self.get_logger().info(
            f'✅ 已收集 {self.num_waypoints} 个中途点和 1 个最终终点'
        )
        self.send_route()

    def on_operation_mode(self, msg: OperationModeState):
        was_auto = self.auto_engaged
        self.auto_engaged = msg.mode == MODE_AUTONOMOUS

        if not was_auto and self.auto_engaged:
            self.get_logger().info('🟢 Autonomous 已激活')

    def on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def dist_to_goal(self):
        if (
            not self.route_points
            or self.current_x is None
            or self.current_y is None
        ):
            return None

        goal = self.route_points[-1].position
        return math.hypot(self.current_x - goal.x, self.current_y - goal.y)

    def send_route(self):
        if self.sending or not self.started:
            return

        self.sending = True
        if not self.route_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ 路线服务不可用')
            self.sending = False
            return

        req = SetRoutePoints.Request()
        req.header.frame_id = 'map'
        req.waypoints = self.route_points[:-1]
        req.goal = self.route_points[-1]

        try:
            future = self.route_cli.call_async(req)
            future.add_done_callback(self.on_sent)
            self.get_logger().info(
                f'📤 正在发送完整路线：{len(req.waypoints)} 个中途点 '
                '+ 1 个最终终点'
            )
        except Exception as exc:
            self.get_logger().error(f'❌ 发送异常: {exc}')
            self.sending = False

    def on_sent(self, future):
        self.sending = False
        try:
            response = future.result()
            if not response.status.success:
                error_names = {
                    SetRoutePoints.Response.ERROR_INVALID_STATE:
                        'INVALID_STATE',
                    SetRoutePoints.Response.ERROR_PLANNER_UNREADY:
                        'PLANNER_UNREADY',
                    SetRoutePoints.Response.ERROR_PLANNER_FAILED:
                        'PLANNER_FAILED',
                    SetRoutePoints.Response.ERROR_REROUTE_FAILED:
                        'REROUTE_FAILED',
                }
                error_name = error_names.get(
                    response.status.code,
                    f'CODE_{response.status.code}',
                )
                self.route_set = False
                self.get_logger().error(
                    f'❌ 路线设置失败 [{error_name}]: '
                    f'{response.status.message}'
                )
                return

            self.route_set = True
            self.get_logger().info('✅ 完整路线设置成功')
            self.get_logger().info(
                '⚡ 请点 Autonomous；车辆将依次经过全部中途点'
            )
        except Exception as exc:
            self.route_set = False
            self.get_logger().error(f'❌ 路线设置失败: {exc}')

    def on_state(self, msg: RouteState):
        self.route_state_val = msg.state

        if (
            msg.state != ROUTE_ARRIVED
            or not self.started
            or not self.route_set
            or self.arrival_logged
        ):
            return

        dist = self.dist_to_goal()
        if dist is not None and dist > self.arrive_threshold:
            return

        self.arrival_logged = True
        self.route_set = False
        self.get_logger().info('🏁 已经过全部中途点并到达最终终点，路线结束')


def main():
    rclpy.init()
    node = MultiWaypointRoute()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('\n🛑 用户停止脚本')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
