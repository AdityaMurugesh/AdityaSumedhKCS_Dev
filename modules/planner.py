from typing import Any, Dict, List, Optional, Set, Tuple

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
        W_base = 1.0  # base weight per edge
        t = current_time

        # ================= TRUCK CONSTRAINT =================
        if vehicle_kind == "truck":
            if (
                "earth_shock" in sensor_data and
                "rainfall" in sensor_data and
                t < len(sensor_data["earth_shock"]) and
                t < len(sensor_data["rainfall"])
            ):
                S_earth = sensor_data["earth_shock"][t]
                S_rain  = sensor_data["rainfall"][t]

                if (W_base * S_earth > 10) and (W_base * S_rain > 30):
                    return float("inf")  # HARD BLOCK

        # ================= DRONE CONSTRAINT =================
        if vehicle_kind == "drone":
            if (
                "wind" in sensor_data and
                "visibility" in sensor_data and
                t < len(sensor_data["wind"]) and
                t < len(sensor_data["visibility"])
            ):
                S_wind = sensor_data["wind"][t]
                S_vis  = sensor_data["visibility"][t]

                if (W_base * S_wind > 60):
                    return float("inf")  # HARD BLOCK

        # ================= DEFAULT COST =================
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
