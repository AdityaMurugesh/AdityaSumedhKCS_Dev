from heapq import heappop, heappush
from typing import Callable, Dict, List, Optional, Tuple


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


def dijkstra_path(
    neighbors: List[List[Tuple[int, int]]],
    start: int,
    goal: int,
    vehicle_kind: str,
    cost_fn: Callable[[int, int, int], float],
) -> Optional[List[int]]:
    """
    cost_fn(u, v, edge_type) -> non-negative cost for taking edge u->v
    Returns node path [start..goal] with minimum total cost.
    """
    if start == goal:
        return [start]

    dist: Dict[int, float] = {start: 0.0}
    parent: Dict[int, int] = {start: -1}
    pq: List[Tuple[float, int]] = [(0.0, start)]

    while pq:
        d, u = heappop(pq)
        if d != dist.get(u, float("inf")):
            continue
        if u == goal:
            break

        for v, et in neighbors[u]:
            if not can_use_edge(vehicle_kind, et):
                continue
            w = float(cost_fn(u, v, et))
            if w < 0:
                continue

            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                parent[v] = u
                heappush(pq, (nd, v))

    if goal not in parent:
        return None

    path = [goal]
    cur = goal
    while parent[cur] != -1:
        cur = parent[cur]
        path.append(cur)
    path.reverse()
    return path
