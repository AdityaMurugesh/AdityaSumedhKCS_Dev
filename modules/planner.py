from collections import deque
from typing import Dict, List, Tuple, Optional, Set, Any


# ----------------------------
# Helpers: infer basic settings
# ----------------------------

def infer_T(public_map: dict, sensor_data: dict) -> int:
    """Infer number of time steps T."""
    for key in ["T", "T_max", "t_max", "time_steps", "num_time_steps"]:
        if key in public_map and isinstance(public_map[key], int):
            return public_map[key]

    # fallback: length of any sensor list
    for v in sensor_data.values():
        if isinstance(v, list) and len(v) > 0:
            return len(v)

    raise ValueError("Could not infer T from public_map.json or sensor_data.json")


def infer_start_node(objectives: dict) -> int:
    """Infer start node; default is 1."""
    for key in ["start_node", "start", "source", "startNode"]:
        if key in objectives and isinstance(objectives[key], int):
            return objectives[key]
    return 1


def infer_vehicle_counts(public_map: dict, objectives: dict) -> Tuple[int, int]:
    """Return (num_trucks, num_drones). Defaults to (1,1) if missing."""
    pairs = [
        ("num_trucks", "num_drones"),
        ("truck_count", "drone_count"),
        ("n_trucks", "n_drones"),
        ("trucks", "drones"),
    ]
    for a, b in pairs:
        if a in public_map and b in public_map and isinstance(public_map[a], int) and isinstance(public_map[b], int):
            return public_map[a], public_map[b]
        if a in objectives and b in objectives and isinstance(objectives[a], int) and isinstance(objectives[b], int):
            return objectives[a], objectives[b]

    if "vehicles" in public_map and isinstance(public_map["vehicles"], dict):
        v = public_map["vehicles"]
        t = int(v.get("num_trucks", v.get("trucks", 1)) or 1)
        d = int(v.get("num_drones", v.get("drones", 1)) or 1)
        return t, d

    return 1, 1


# -----------------------------------
# Graph: adjacency matrix -> neighbors
# -----------------------------------

def build_neighbors(adj: List[List[int]]) -> List[List[Tuple[int, int]]]:
    """
    adj is NxN matrix with:
      -1 = no edge
       0 = air edge
       1..5 = road type

    Returns neighbors[u] = list of (v, edge_type) using 1-based node ids for outside.
    Internally we will keep nodes as 1..N (not 0-indexed) for simplicity.
    """
    n = len(adj)
    neighbors: List[List[Tuple[int, int]]] = [[] for _ in range(n + 1)]  # index 1..n

    for i in range(n):
        for j in range(n):
            et = adj[i][j]
            if isinstance(et, int) and et != -1:
                u = i + 1
                v = j + 1
                neighbors[u].append((v, et))
    return neighbors


def can_use_edge(vehicle_kind: str, edge_type: int) -> bool:
    """
    vehicle_kind: 'truck' or 'drone'
    Trucks: edge_type 1..5
    Drones: edge_type 0..5
    """
    if vehicle_kind == "truck":
        return 1 <= edge_type <= 5
    if vehicle_kind == "drone":
        return 0 <= edge_type <= 5
    return False


# ----------------------------
# BFS shortest path (by hops)
# ----------------------------

def bfs_path(neighbors: List[List[Tuple[int, int]]], start: int, goal: int, vehicle_kind: str) -> Optional[List[int]]:
    """
    Returns a path [start, ..., goal] with fewest edges, respecting vehicle edge rules.
    If unreachable, returns None.
    """
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
                # reconstruct
                path = [v]
                cur = v
                while parent[cur] != -1:
                    cur = parent[cur]
                    path.append(cur)
                path.reverse()
                return path
            q.append(v)

    return None


# -----------------------------------------
# Objectives parsing (robust to key variants)
# -----------------------------------------

def parse_objectives(objectives_json: dict) -> List[dict]:
    """
    Try to find list of objectives in common key names.
    Each objective dict should end up with:
      id, node, release, deadline, points
    """
    obj_list = None
    for key in ["objectives", "tasks", "missions", "targets"]:
        if key in objectives_json and isinstance(objectives_json[key], list):
            obj_list = objectives_json[key]
            break
    if obj_list is None:
        return []

    def get_int(d: dict, keys: List[str], default: int) -> int:
        for k in keys:
            if k in d and isinstance(d[k], int):
                return d[k]
        return default

    parsed = []
    for idx, o in enumerate(obj_list):
        if not isinstance(o, dict):
            continue
        parsed.append({
            "id": o.get("id", idx),
            "node": get_int(o, ["node", "target", "target_node", "location"], 1),
            "release": get_int(o, ["T_start", "release", "start", "start_time"], 0),
            "deadline": get_int(o, ["T_end", "deadline", "end", "end_time"], 0),
            "points": get_int(o, ["P_max", "points", "reward", "max_points"], 0),
        })
    return parsed


# ----------------------------
# Planning core
# ----------------------------

def extend_timeline(tl: List[int], node: int, steps: int) -> None:
    """Append `node` for `steps` time steps (waiting)."""
    if steps <= 0:
        return
    tl.extend([node] * steps)


def append_move_path(tl: List[int], path_nodes: List[int]) -> None:
    """
    Convert path nodes into per-tick positions appended to timeline.
    If path is [a,b,c], that means:
      currently at a (already last in tl),
      move to b (append b),
      move to c (append c)
    So we append path[1:], not path[0].
    """
    if len(path_nodes) <= 1:
        return
    tl.extend(path_nodes[1:])


def plan_vehicle_to_objective(
    neighbors: List[List[Tuple[int, int]]],
    vehicle_kind: str,
    timeline: List[int],
    current_time: int,
    target_node: int,
    release: int,
    deadline: int,
    T: int
) -> Optional[Tuple[List[int], int, int]]:
    """
    Try to plan this vehicle from its current node/time to reach target within [release, deadline].
    Returns (new_timeline, new_time, new_node) if feasible, else None.
    """
    if current_time >= T:
        return None

    cur_node = timeline[-1]

    path = bfs_path(neighbors, cur_node, target_node, vehicle_kind)
    if path is None:
        return None

    travel_steps = len(path) - 1
    arrival_time = current_time + travel_steps

    # If arrive after deadline -> not feasible
    if arrival_time > deadline:
        return None

    # Create a copy timeline to simulate
    new_tl = list(timeline)

    # Move along path
    append_move_path(new_tl, path)
    new_time = current_time + travel_steps
    new_node = target_node

    # If arrive before release, wait until release
    if new_time < release:
        wait_steps = release - new_time
        extend_timeline(new_tl, new_node, wait_steps)
        new_time += wait_steps

    # Now we are at target at time >= release and <= deadline
    if not (release <= new_time <= deadline):
        return None

    # Clip if over T (we will clip/fill later anyway)
    if len(new_tl) > T:
        new_tl = new_tl[:T]
        new_time = T
        new_node = new_tl[-1]

    return new_tl, new_time, new_node


def finalize_to_T(timeline: List[int], T: int) -> List[int]:
    """Ensure timeline length exactly T by waiting at last node or clipping."""
    if len(timeline) >= T:
        return timeline[:T]
    last = timeline[-1]
    return timeline + [last] * (T - len(timeline))


def plan(public_map: dict, sensor_data: dict, objectives_json: dict) -> Dict[str, List[int]]:
    """
    Main planner called by main.py.
    Greedy approach:
      - Sort objectives by earliest deadline
      - Try assign each objective to best (earliest finish) vehicle:
          drones first, then trucks
    """
    T = infer_T(public_map, sensor_data)
    start_node = infer_start_node(objectives_json)
    num_trucks, num_drones = infer_vehicle_counts(public_map, objectives_json)

    adj = public_map.get("map")
    if not isinstance(adj, list):
        # fallback: no map, just wait
        sol: Dict[str, List[int]] = {}
        for i in range(1, num_trucks + 1):
            sol[f"truck{i}"] = [start_node] * T
        for i in range(1, num_drones + 1):
            sol[f"drone{i}"] = [start_node] * T
        return sol

    neighbors = build_neighbors(adj)
    objectives = parse_objectives(objectives_json)
    # Sort by deadline then release (simple)
    objectives.sort(key=lambda o: (o["deadline"], o["release"]))

    # Vehicle timelines start at time 0 at start_node.
    drone_tls: List[List[int]] = [[start_node] for _ in range(num_drones)]
    truck_tls: List[List[int]] = [[start_node] for _ in range(num_trucks)]
    drone_time: List[int] = [0] * num_drones
    truck_time: List[int] = [0] * num_trucks

    done: Set[Any] = set()

    for obj in objectives:
        oid = obj["id"]
        if oid in done:
            continue
        target = obj["node"]
        release = obj["release"]
        deadline = obj["deadline"]

        best_choice = None
        # Try drones first
        for i in range(num_drones):
            attempt = plan_vehicle_to_objective(
                neighbors, "drone", drone_tls[i], drone_time[i],
                target, release, deadline, T
            )
            if attempt is None:
                continue
            new_tl, new_t, _ = attempt
            finish_time = new_t
            if best_choice is None or finish_time < best_choice[0]:
                best_choice = (finish_time, "drone", i, new_tl, new_t)

        # If no drone can do it, try trucks
        if best_choice is None:
            for i in range(num_trucks):
                attempt = plan_vehicle_to_objective(
                    neighbors, "truck", truck_tls[i], truck_time[i],
                    target, release, deadline, T
                )
                if attempt is None:
                    continue
                new_tl, new_t, _ = attempt
                finish_time = new_t
                if best_choice is None or finish_time < best_choice[0]:
                    best_choice = (finish_time, "truck", i, new_tl, new_t)

        # Commit assignment if found
        if best_choice is not None:
            _, kind, idx, new_tl, new_t = best_choice
            if kind == "drone":
                drone_tls[idx] = new_tl
                drone_time[idx] = new_t
            else:
                truck_tls[idx] = new_tl
                truck_time[idx] = new_t
            done.add(oid)

    # Finalize all timelines to length T
    sol: Dict[str, List[int]] = {}
    for i in range(num_trucks):
        sol[f"truck{i+1}"] = finalize_to_T(truck_tls[i], T)
    for i in range(num_drones):
        sol[f"drone{i+1}"] = finalize_to_T(drone_tls[i], T)

    return sol
