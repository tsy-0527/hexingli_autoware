#!/usr/bin/env python3
"""Publish Lanelet2 parking_lot areas as colored RViz MarkerArray messages."""

import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


PALETTE = (
    (0.10, 0.85, 0.25),
    (1.00, 0.55, 0.05),
    (0.10, 0.55, 1.00),
    (0.85, 0.20, 0.90),
)


def point_message(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point


def load_parking_lots(osm_path: Path):
    root = ET.parse(osm_path).getroot()
    nodes = {}
    for node in root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        if "local_x" not in tags or "local_y" not in tags:
            continue
        nodes[node.get("id")] = (
            float(tags["local_x"]),
            float(tags["local_y"]),
            float(tags.get("ele", 0.0)),
        )

    ways = {}
    way_tags = {}
    for way in root.findall("way"):
        way_id = way.get("id")
        ways[way_id] = [nd.get("ref") for nd in way.findall("nd")]
        way_tags[way_id] = {
            tag.get("k"): tag.get("v") for tag in way.findall("tag")
        }

    parking_lots = []

    # Official sample-map-planning style:
    # <way><tag k="area" v="yes"/><tag k="type" v="parking_lot"/></way>
    for way_id, refs in ways.items():
        tags = way_tags[way_id]
        if tags.get("type") != "parking_lot" and tags.get("subtype") != "parking_lot":
            continue
        polygon = [nodes[ref] for ref in refs if ref in nodes]
        if len(polygon) > 1 and polygon[0] == polygon[-1]:
            polygon.pop()
        if len(polygon) >= 3:
            parking_lots.append((f"way:{way_id}", polygon))

    # Also support maps that encode the area as a multipolygon relation.
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        if tags.get("subtype") != "parking_lot" and tags.get("type") != "parking_lot":
            continue
        for member in relation.findall("member"):
            if member.get("type") != "way" or member.get("role") != "outer":
                continue
            refs = ways.get(member.get("ref"), [])
            polygon = [nodes[ref] for ref in refs if ref in nodes]
            if len(polygon) > 1 and polygon[0] == polygon[-1]:
                polygon.pop()
            if len(polygon) >= 3:
                parking_lots.append((f"relation:{relation.get('id')}", polygon))
    return parking_lots


class ParkingLotMarkerPublisher(Node):
    def __init__(self):
        super().__init__("parking_lot_marker_publisher")
        self.declare_parameter(
            "osm_path", "/home/tang/autoware_map/map_task4/lanelet2_map.osm"
        )
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/debug/parking_lots")
        self.declare_parameter("fill_alpha", 0.35)
        self.declare_parameter("z_offset", 0.15)

        osm_path = Path(self.get_parameter("osm_path").value).expanduser()
        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        self.fill_alpha = float(self.get_parameter("fill_alpha").value)
        self.z_offset = float(self.get_parameter("z_offset").value)

        if not osm_path.is_file():
            raise FileNotFoundError(f"Lanelet2 map not found: {osm_path}")
        self.parking_lots = load_parking_lots(osm_path)
        if not self.parking_lots:
            raise RuntimeError(f"No parking_lot way or Area relation found in {osm_path}")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, topic, qos)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            f"Loaded {len(self.parking_lots)} parking lot(s) from {osm_path}; publishing {topic}"
        )

    def base_marker(self, marker_id: int, marker_type: int, namespace: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_markers(self):
        output = MarkerArray()
        marker_id = 0
        for index, (object_id, polygon) in enumerate(self.parking_lots):
            red, green, blue = PALETTE[index % len(PALETTE)]
            z = max(point[2] for point in polygon) + self.z_offset

            fill = self.base_marker(marker_id, Marker.TRIANGLE_LIST, "parking_lot_fill")
            marker_id += 1
            fill.scale.x = fill.scale.y = fill.scale.z = 1.0
            fill.color.r, fill.color.g, fill.color.b = red, green, blue
            fill.color.a = self.fill_alpha
            for triangle_index in range(1, len(polygon) - 1):
                for vertex in (polygon[0], polygon[triangle_index], polygon[triangle_index + 1]):
                    fill.points.append(point_message(vertex[0], vertex[1], z))
            output.markers.append(fill)

            border = self.base_marker(marker_id, Marker.LINE_STRIP, "parking_lot_border")
            marker_id += 1
            border.scale.x = 0.20
            border.color.r, border.color.g, border.color.b = red, green, blue
            border.color.a = 1.0
            for vertex in polygon + [polygon[0]]:
                border.points.append(point_message(vertex[0], vertex[1], z + 0.03))
            output.markers.append(border)

            center_x = sum(point[0] for point in polygon) / len(polygon)
            center_y = sum(point[1] for point in polygon) / len(polygon)
            label = self.base_marker(marker_id, Marker.TEXT_VIEW_FACING, "parking_lot_label")
            marker_id += 1
            label.pose.position = point_message(center_x, center_y, z + 0.5)
            label.scale.z = 0.8
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"parking_lot {object_id}"
            output.markers.append(label)

        self.publisher.publish(output)


def main():
    rclpy.init()
    node = ParkingLotMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
