"""Station route graph: nodes (x, y[, axis_deg]), undirected edges, stations (node + final yaw).

route(start_xy, station) -> list of (x, y, yaw) waypoints for NavigateThroughPoses:
  - starts from the graph node nearest to the robot (skipped if the robot is already past it)
  - an intermediate node with an axis (e.g. a corridor centre line) is passed along that axis,
    in whichever of the two directions matches the travel direction (previous -> next point);
    a node without an axis is passed facing the next node
  - the last waypoint has the station's yaw (Hybrid-A* plans any turn-around needed)
"""
import heapq
import math


class RouteGraph:
    def __init__(self, nodes, edges, stations):
        self.nodes = {k: (float(v[0]), float(v[1])) for k, v in nodes.items()}
        self.axis = {k: math.radians(float(v[2])) for k, v in nodes.items() if len(v) > 2}
        self.adj = {k: [] for k in self.nodes}
        for a, b in edges:
            if a not in self.nodes or b not in self.nodes:
                raise ValueError(f'edge {a}-{b}: unknown node')
            d = math.dist(self.nodes[a], self.nodes[b])
            self.adj[a].append((b, d))
            self.adj[b].append((a, d))
        self.stations = {}
        for k, v in stations.items():
            if v['node'] not in self.nodes:
                raise ValueError(f'station {k}: unknown node {v["node"]}')
            self.stations[k] = (v['node'], math.radians(float(v['yaw_deg'])))

    @staticmethod
    def from_dict(d):
        return RouteGraph(d['nodes'], d['edges'], d['stations'])

    def nearest(self, x, y):
        return min(self.nodes, key=lambda k: math.dist(self.nodes[k], (x, y)))

    def shortest(self, a, b):
        dist, prev = {a: 0.0}, {}
        pq = [(0.0, a)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == b:
                break
            if d > dist.get(u, math.inf):
                continue
            for v, w in self.adj[u]:
                nd = d + w
                if nd < dist.get(v, math.inf):
                    dist[v], prev[v] = nd, u
                    heapq.heappush(pq, (nd, v))
        if b not in dist:
            return None
        path = [b]
        while path[-1] != a:
            path.append(prev[path[-1]])
        return path[::-1]

    def route(self, x, y, station, skip_radius=1.0):
        if station not in self.stations:
            raise KeyError(f'unknown station {station}')
        goal_node, goal_yaw = self.stations[station]
        start = self.nearest(x, y)
        nodes = self.shortest(start, goal_node)
        if nodes is None:
            return None, []
        # drop the first node if the robot is already at it, or already between it and the next one
        if len(nodes) > 1:
            n0, n1 = self.nodes[nodes[0]], self.nodes[nodes[1]]
            at_first = math.dist(n0, (x, y)) < skip_radius
            past_first = math.dist((x, y), n1) < math.dist(n0, n1)
            if at_first or past_first:
                nodes = nodes[1:]
        pts = [self.nodes[n] for n in nodes]
        wps = []
        for i, (px, py) in enumerate(pts):
            if i + 1 == len(pts):
                yaw = goal_yaw
            else:
                nx, ny = pts[i + 1]
                if nodes[i] in self.axis:
                    bx, by = pts[i - 1] if i > 0 else (x, y)
                    travel = math.atan2(ny - by, nx - bx)
                    a = self.axis[nodes[i]]
                    yaw = a if abs(_wrap(travel - a)) <= math.pi / 2 else _wrap(a + math.pi)
                else:
                    yaw = math.atan2(ny - py, nx - px)
            wps.append((px, py, yaw))
        return nodes, wps


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
