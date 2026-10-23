import math
import heapq
from typing import Dict, List, Tuple, Optional

class RoadNetwork:
    def __init__(self):
        self.waypoints: Dict[int, dict] = {}
        self.edges: Dict[int, List[Tuple[int, float]]] = {}
        self.next_id = 1

    def add_waypoint(self, position: Tuple[float, float, float], name: str = "", description: str = "") -> int:
        waypoint_id = self.next_id
        self.next_id += 1
        self.waypoints[waypoint_id] = {
            'id': waypoint_id,
            'position': position,
            'name': name,
            'description': description
        }
        self.edges[waypoint_id] = []
        return waypoint_id

    def add_edge(self, from_id: int, to_id: int, weight: Optional[float] = None) -> bool:
        if from_id not in self.waypoints or to_id not in self.waypoints:
            return False
        if weight is None:
            pos1 = self.waypoints[from_id]['position']
            pos2 = self.waypoints[to_id]['position']
            weight = math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2)
        self.edges[from_id].append((to_id, weight))
        self.edges[to_id].append((from_id, weight))
        return True

    def get_waypoint(self, waypoint_id: int) -> Optional[dict]:
        return self.waypoints.get(waypoint_id)

    def get_neighbors(self, waypoint_id: int) -> List[Tuple[int, float]]:
        return self.edges.get(waypoint_id, [])

    def dijkstra(self, start_id: int, goal_id: int) -> Tuple[List[int], float]:
        if start_id not in self.waypoints or goal_id not in self.waypoints:
            return [], 0.0
        distances = {node: float('inf') for node in self.waypoints}
        distances[start_id] = 0.0
        previous = {node: None for node in self.waypoints}
        priority_queue = [(0.0, start_id)]
        while priority_queue:
            current_dist, current_node = heapq.heappop(priority_queue)
            if current_node == goal_id:
                break
            if current_dist > distances[current_node]:
                continue
            for neighbor, weight in self.edges[current_node]:
                distance = current_dist + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))
        if previous[goal_id] is None and start_id != goal_id:
            return [], 0.0
        path = []
        current = goal_id
        while current is not None:
            path.append(current)
            current = previous[current]
        path.reverse()
        return path, distances[goal_id]

    def clear(self):
        self.waypoints = {}
        self.edges = {}
        self.next_id = 1

    def get_all_waypoints(self) -> List[dict]:
        return list(self.waypoints.values())