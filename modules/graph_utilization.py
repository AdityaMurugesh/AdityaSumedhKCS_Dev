from collections import deque
from typing import Dict, List, Optional, Tuple


def build_neighbors(adj: List[List[int]]) -> List[List[Tuple[int, int]]]:
    n = len(adj)
    neighbors: List[List[Tuple[int, int]]] = [[] for _ in range(n + 1)]

    for i in range(n):
        for j in range(n):
            et = adj[i][j]
            if isinstance(et, int) and et != -1:
                neighbors[i + 1].append((j + 1, et))

    return neighbors


def can_use_edge(vehicle_kind: str, edge_type: int) -> bool:
    if vehicle_kind == "truck":
        return 1 <= edge_type <= 5
    if vehicle_kind == "drone":
        return 0 <= edge_type <= 5
    return False


def bfs_path(
    neighbors: List[List[Tuple[int, int]]],
    start: int,
    goal: int,
    vehicle_kind: str
) -> Optional[List[int]]:
    if start == goal:
        return [start]

    q = deque([start])
    parent: Dict[int, int] = {start: -1}

    while q:
        u = q.popleft()
        for v, et in neighbors[u]:
            if not can_use_edge(vehicle_kind, et):
                continue
            if v in parent:
                continue

            parent[v] = u
            if v == goal:
                path = [v]
                cur = v
                while parent[cur] != -1:
                    cur = parent[cur]
                    path.append(cur)
                path.reverse()
                return path

            q.append(v)

    return None
