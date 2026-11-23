#!/usr/bin/env python3
"""Replay a recorded route by calling Autoware's SetLaneletRoute service.

This takes the "route from segments" branch inside mission_planner
(create_route with segments), so no Dijkstra search is performed and
start_pose defaults to the vehicle's current position.
"""
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# NOTE: 消息类型在不同 Autoware 版本略有差异，若报错就把下面两行换掉。
# 常见组合：
#   autoware_adapi_v1_msgs/srv/SetRoute            + msg RouteSegment/RoutePrimitive
#   autoware_planning_msgs/srv/SetLaneletRoute     + msg LaneletSegment/LaneletPrimitive
from autoware_planning_msgs.srv import SetLaneletRoute
from autoware_planning_msgs.msg import LaneletSegment, LaneletPrimitive


class RoutePlayer(Node):
    def __init__(self):
        super().__init__('route_player')

        self.declare_parameter(
            'route_path',
            '/home/tang/tsy/route_repeat_ws/recorded_route.yaml')
        self.declare_parameter(
            'set_route_service',
            '/planning/mission_planning/mission_planner/srv/set_lanelet_route')
        self.declare_parameter('auto_send', True)
        self.declare_parameter('primitive_type', 'lane')

        route_path = self.get_parameter('route_path').value
        srv_name = self.get_parameter('set_route_service').value
        self.prim_type = self.get_parameter('primitive_type').value

        with open(route_path) as f:
            self.data = yaml.safe_load(f)
        self.get_logger().info(
            f'Loaded {len(self.data["segments"])} segments from {route_path}')

        # 可视化
        self.marker_pub = self.create_publisher(
            MarkerArray, '/route_player/markers', 10)
        self.viz_timer = self.create_timer(0.5, self.publish_viz)

        # service
        self.cli = self.create_client(SetLaneletRoute, srv_name)
        if self.get_parameter('auto_send').value:
            self._send_when_ready()

    def _send_when_ready(self):
        self.get_logger().info('Waiting for service...')
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'set_lanelet_route service not available. '
                'Is Autoware planning_simulation running?')
            return
        self.send_route()

    def send_route(self):
        req = SetLaneletRoute.Request()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = 'map'

        g = self.data['goal_pose']
        goal = Pose()
        goal.position.x = g['x']
        goal.position.y = g['y']
        goal.position.z = g['z']
        goal.orientation.x = g['qx']
        goal.orientation.y = g['qy']
        goal.orientation.z = g['qz']
        goal.orientation.w = g['qw']
        req.goal_pose = goal

        for seg in self.data['segments']:
            ls = LaneletSegment()
            for pid in seg['primitives']:
                p = LaneletPrimitive()
                p.id = int(pid)
                p.primitive_type = self.prim_type
                ls.primitives.append(p)
            pref = LaneletPrimitive()
            pref.id = int(seg['preferred'])
            pref.primitive_type = self.prim_type
            ls.preferred_primitive = pref
            req.segments.append(ls)

        # allow_modification 之类的字段视版本而定，一般默认 False 即可
        # req.allow_modification = False

        self.get_logger().info(f'Sending {len(req.segments)} segments...')
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'set_lanelet_route response: {res}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def publish_viz(self):
        """把加载的 route 画出来，方便确认加载的是不是想要的那条。"""
        if 'segments' not in self.data:
            return
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, seg in enumerate(self.data['segments']):
            cl = seg.get('centerline')
            if not cl:
                continue
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = now
            line.ns = 'planned_route'
            line.id = i
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.4
            line.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
            for x, y, z in cl:
                p = Point()
                p.x, p.y, p.z = float(x), float(y), float(z)
                line.points.append(p)
            markers.markers.append(line)

        # goal marker
        g = self.data.get('goal_pose')
        if g:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'goal'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = g['x']
            m.pose.position.y = g['y']
            m.pose.position.z = g['z']
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.5
            m.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0)
            markers.markers.append(m)

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = RoutePlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
