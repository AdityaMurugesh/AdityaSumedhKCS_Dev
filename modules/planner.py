from typing import Any, Dict, List, Optional, Set, Tuple, Any
from webbrowser import get

from modules.config import infer_T, infer_start_node, infer_vehicle_counts
from modules.graph_utilization import dijkstra_path, build_neighbors
from modules.objective_utilization import parse_objectives, sort_objectives
from modules.timeline_utilization import append_move_path, extend_wait, finalize_to_T


def try_do_objective(
    neighbors: List[List[Tuple[int, int]]],
    vehicle_kind: str,
    timeline: List[int],
    current_time: int,
    target_node: int,
    release: int,
    deadline: int,
    T: int,
    sensor_data: dict = None
) -> Optional[Tuple[List[int], int]]:
    if current_time >= T:
        return None

    cur_node = timeline[-1]

    def cost_fn(u: int, v: int, edge_type: int) -> float:
        # Hard feasibility constraints + uniform base edge cost.
        W_base = 1.0
        t = current_time

        # If no sensor data, just use base cost
        if not sensor_data or t >= T:
            return W_base

        if vehicle_kind == "truck":
            if (
                "earth_shock" in sensor_data and isinstance(sensor_data["earth_shock"], list)
                and "rainfall" in sensor_data and isinstance(sensor_data["rainfall"], list)
                and t < len(sensor_data["earth_shock"])
                and t < len(sensor_data["rainfall"])
            ):
                S_earth = sensor_data["earth_shock"][t]
                S_rain = sensor_data["rainfall"][t]
                if (W_base * S_earth > 10) and (W_base * S_rain > 30):
                    return float("inf")  # HARD BLOCK


        if vehicle_kind == "drone":
            if (
                "wind" in sensor_data and isinstance(sensor_data["wind"], list)
                and t < len(sensor_data["wind"])
            ):
                S_wind = sensor_data["wind"][t]
                if (W_base * S_wind > 60):
                    return float("inf")  # HARD BLOCK

        return W_base



    path = dijkstra_path(neighbors, cur_node, target_node, vehicle_kind, cost_fn)

    if path is None:
        return None

    travel_steps = len(path) - 1
    arrival_time = current_time + travel_steps
    if arrival_time > deadline:
        return None

    new_tl = list(timeline)
    append_move_path(new_tl, path)
    new_time = current_time + travel_steps

    if new_time < release:
        extend_wait(new_tl, target_node, release - new_time)
        new_time = release

    if not (release <= new_time <= deadline):
        return None

    if len(new_tl) > T:
        new_tl = new_tl[:T]
        new_time = T

    return new_tl, new_time


def plan(public_map: dict, sensor_data: dict, objectives_json: dict) -> Dict[str, List[int]]:
    T = infer_T(public_map, sensor_data)
    start_node = infer_start_node(objectives_json)
    num_trucks, num_drones = infer_vehicle_counts(public_map, objectives_json)

    adj = public_map.get("map")
    if not isinstance(adj, list):
        sol: Dict[str, List[int]] = {}
        for i in range(1, num_trucks + 1):
            sol[f"truck{i}"] = [start_node] * T
        for i in range(1, num_drones + 1):
            sol[f"drone{i}"] = [start_node] * T
        return sol

    neighbors = build_neighbors(adj)
    objectives = sort_objectives(parse_objectives(objectives_json))

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

        best = None  # (finish_time, kind, idx, new_tl, new_time)

        for i in range(num_drones):
            attempt = try_do_objective(neighbors, "drone", drone_tls[i], drone_time[i], target, release, deadline, T, sensor_data)
            if attempt is None:
                continue
            new_tl, new_t = attempt
            if best is None or new_t < best[0]:
                best = (new_t, "drone", i, new_tl, new_t)

        if best is None:
            for i in range(num_trucks):
                attempt = try_do_objective(neighbors, "truck", truck_tls[i], truck_time[i], target, release, deadline, T, sensor_data)
                if attempt is None:
                    continue
                new_tl, new_t = attempt
                if best is None or new_t < best[0]:
                    best = (new_t, "truck", i, new_tl, new_t)

        if best is not None:
            _, kind, idx, new_tl, new_t = best
            if kind == "drone":
                drone_tls[idx] = new_tl
                drone_time[idx] = new_t
            else:
                truck_tls[idx] = new_tl
                truck_time[idx] = new_t
            done.add(oid)

    sol: Dict[str, List[int]] = {}
    for i in range(num_trucks):
        sol[f"truck{i+1}"] = finalize_to_T(truck_tls[i], T)
    for i in range(num_drones):
        sol[f"drone{i+1}"] = finalize_to_T(drone_tls[i], T)

    return sol

def _travel_cost_from_timeline(tl: List[int]) -> int:
    # 1 cost per MOVE (node changes), 0 cost for WAIT (same node)
    cost = 0
    for i in range(1, len(tl)):
        if tl[i] != tl[i - 1]:
            cost += 1
    return cost

def total_travel_cost(solution: Dict[str, List[int]]) -> int:
    return sum(_travel_cost_from_timeline(tl) for tl in solution.values())

from typing import Dict, List, Tuple, Any

def _travel_cost_from_timeline(tl: List[int]) -> int:
    cost = 0
    for i in range(1, len(tl)):
        if tl[i] != tl[i - 1]:
            cost += 1
    return cost

def total_travel_cost(solution: Dict[str, List[int]]) -> int:
    return sum(_travel_cost_from_timeline(tl) for tl in solution.values())

def score_solution(
    objectives_json: dict,
    solution: Dict[str, List[int]],
) -> Tuple[float, Dict[Any, float], int]:
    # --- get objectives the SAME way planner does ---
    try:
        from modules.objective_utilization import parse_objectives
        objs = parse_objectives(objectives_json)
    except Exception:
        objs = objectives_json.get("objectives", objectives_json)

    if not isinstance(objs, list):
        raise ValueError("objectives_json must be a list or have key 'objectives' as a list")

    def get(o: dict, *keys, default=None):
        for k in keys:
            if k in o:
                return o[k]
        return default

    vehicle_names = list(solution.keys())
    if not vehicle_names:
        return 0.0, {}, 0

    T = len(next(iter(solution.values())))

    # Collect visited nodes per time for fast membership checks
    # visited_at_t[t] = set(nodes occupied by any vehicle at time t)
    visited_at_t: List[set] = [set() for _ in range(T)]
    for name in vehicle_names:
        tl = solution[name]
        for t in range(min(T, len(tl))):
            visited_at_t[t].add(tl[t])

    per_obj: Dict[Any, float] = {}
    total_obj_score = 0.0

    # Small debug counters (optional)
    completed_count = 0

    for o in objs:
        if not isinstance(o, dict):
            continue

        oid = get(o, "id", "objective_id", "name")

        # These are the fields your planner ends up using after parse_objectives
        target = get(o, "node", "target", "target_node")
        tstart = get(o, "release", "tstart", "start", "start_time", "Tstart")
        tend = get(o, "deadline", "tend", "end", "end_time", "Tend")

        pmax = float(get(o, "points", "P_max", "reward", "pmax", "Pmax", default=0))
        late_pen = float(get(o, "late_penalty_per_step", "late_penalty", "penalty", default=0))

        if target is None or tstart is None or tend is None:
            per_obj[oid] = 0.0
            continue

        target = int(target)
        tstart = int(tstart)
        tend = int(tend)

        lo = max(0, tstart)
        hi = min(T - 1, tend)

        # Try common node-index variants: target, target+1, target-1
        candidates = (target, target + 1, target - 1)

        best_score = 0.0
        best_found = False

        for cand in candidates:
            if lo > hi:
                continue

            t_arrival = None
            for t in range(lo, hi + 1):
                if cand in visited_at_t[t]:
                    t_arrival = t
                    break

            if t_arrival is None:
                continue

            lateness = t_arrival - tstart
            s = pmax - late_pen * lateness
            if s < 0:
                s = 0.0

            if s > best_score:
                best_score = s
                best_found = True

        per_obj[oid] = best_score
        total_obj_score += best_score
        if best_found and best_score > 0:
            completed_count += 1

    travel_cost = total_travel_cost(solution)
    total_score = total_obj_score - travel_cost

    return total_score, per_obj, travel_cost

   
