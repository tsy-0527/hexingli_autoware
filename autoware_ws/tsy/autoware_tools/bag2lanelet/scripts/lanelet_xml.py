import math
from xml.dom.minidom import parseString
import xml.etree.ElementTree as ET

from mgrspy import mgrs
from pyproj import Transformer


class LaneletMap:
    """Generate a lanelet2 map from local (map frame) coordinates.

    The lanelet2 OSM format requires every node to have WGS84 ``lat``/``lon``
    attributes, so purely local coordinates cannot be written there directly.
    Instead, the given MGRS code is used as the geographic anchor of the local
    map: the original local coordinates are kept unchanged in the
    ``local_x`` / ``local_y`` tags and ``lat``/``lon`` are derived from them
    with an accurate UTM projection (negative and fractional coordinates are
    supported).

    When Autoware loads the map with ``projector_type: Local`` it ignores
    ``lat``/``lon``, overwrites x/y with ``local_x`` / ``local_y`` and thus
    reproduces the local geometry exactly.  With a UTM/MGRS projector the
    relative geometry is still correct, only the absolute position depends on
    the chosen MGRS anchor.
    """

    def __init__(self, mgrs="53SPU"):
        self.mgrs = mgrs.strip().upper()
        self.element_num = 0
        self.node_list = []
        self.way_list = []
        self.relation_list = []

        self.root = ET.Element("osm", {"generator": "bag2lanelet"})
        self.meta = ET.SubElement(
            self.root, "MetaInfo", {"format_version": "1", "map_version": "2"}
        )
        self._init_projection()

    def _init_projection(self):
        code = self.mgrs
        if len(code) not in (5, 15):
            raise ValueError(
                "MGRS code must be a 100km grid code such as '54SUE' or a full "
                "MGRS code such as '54SUE1234567890', got '{}'".format(code)
            )

        self.grid_code = code[:5]
        zone = int(self.grid_code[:2])
        lat_band = self.grid_code[2]
        epsg = (32600 if lat_band >= "N" else 32700) + zone
        self._to_wgs = Transformer.from_crs(
            "EPSG:{}".format(epsg), "EPSG:4326", always_xy=True
        )
        self._from_wgs = Transformer.from_crs(
            "EPSG:4326", "EPSG:{}".format(epsg), always_xy=True
        )

        # Anchor of the local map in WGS84 and UTM.
        anchor_lat, anchor_lon = mgrs.toWgs(code)
        self._origin_e, self._origin_n = self._from_wgs.transform(anchor_lon, anchor_lat)

        # Lower left corner of the MGRS 100km square, used for the mgrs_code tag.
        # The 0.5m offset makes the derivation robust against projection noise.
        self._base_e = math.floor((self._origin_e + 0.5) / 100000.0) * 100000.0
        self._base_n = math.floor((self._origin_n + 0.5) / 100000.0) * 100000.0

    def get_latlon(self, x, y, z=0.0):
        easting = self._origin_e + float(x)
        northing = self._origin_n + float(y)
        lon, lat = self._to_wgs.transform(easting, northing)
        return (lat, lon)

    def _get_mgrs_code(self, x, y):
        easting = int(round(self._origin_e + float(x) - self._base_e))
        northing = int(round(self._origin_n + float(y) - self._base_n))
        if 0 <= easting <= 99999 and 0 <= northing <= 99999:
            # 3 digits each = 100m precision, same convention as the
            # Autoware lanelet2 maps.
            return self.grid_code + "%03d%03d" % (easting // 100, northing // 100)
        # Coordinates outside the 100km square (e.g. a map frame origin south
        # or west of the chosen anchor): keep the square code only.  The exact
        # position is preserved in local_x/local_y and lat/lon.
        return self.grid_code

    def add_node(self, x, y, z):
        self.element_num += 1
        lat, lon = self.get_latlon(x, y, z)
        node = ET.SubElement(
            self.root,
            "node",
            {"id": str(self.element_num), "lat": str(lat), "lon": str(lon)},
        )
        tag = [
            {"k": "type", "v": ""},
            {"k": "subtype", "v": ""},
            {"k": "mgrs_code", "v": self._get_mgrs_code(x, y)},
            {"k": "local_x", "v": str(x)},
            {"k": "local_y", "v": str(y)},
            {"k": "ele", "v": str(z)},
        ]
        for t in tag:
            ET.SubElement(node, "tag", t)

        return self.element_num

    def add_way(self, node_list):
        self.element_num += 1

        way = ET.SubElement(self.root, "way", {"id": str(self.element_num)})
        tag = [
            {"k": "type", "v": "line_thin"},
            {"k": "subtype", "v": "solid"},
        ]

        for nd in node_list:
            ET.SubElement(way, "nd", {"ref": str(nd)})

        for t in tag:
            ET.SubElement(way, "tag", t)

        return self.element_num

    def add_relation(self, left_id, right_id, center_id=None):
        self.element_num += 1
        relation = ET.SubElement(self.root, "relation", {"id": str(self.element_num)})

        ET.SubElement(relation, "member", {"type": "way", "ref": str(left_id), "role": "left"})
        ET.SubElement(relation, "member", {"type": "way", "ref": str(right_id), "role": "right"})

        if center_id:
            ET.SubElement(
                relation, "member", {"type": "way", "ref": str(center_id), "role": "centerline"}
            )

        tag = [
            {"k": "type", "v": "lanelet"},
            {"k": "subtype", "v": "road"},
            {"k": "location", "v": "urban"},
            {"k": "participant:vehicle", "v": "yes"},
            {"k": "one_way", "v": "yes"},
            {"k": "speed_limit", "v": str(20)},
        ]
        for t in tag:
            ET.SubElement(relation, "tag", t)
        return self.element_num

    def save(self, filename):
        # tree = ET.ElementTree(self.root)
        parsed = parseString(ET.tostring(self.root, "utf-8")).toprettyxml(indent="  ")
        with open(filename, "w") as f:
            f.write(parsed)
