from typing import Tuple


def infer_T(public_map: dict, sensor_data: dict) -> int:
    for key in ["T", "T_max", "t_max", "time_steps", "num_time_steps"]:
        if key in public_map and isinstance(public_map[key], int):
            return public_map[key]

    for v in sensor_data.values():
        if isinstance(v, list) and len(v) > 0:
            return len(v)

    raise ValueError("Could not infer T from inputs.")


def infer_start_node(objectives: dict) -> int:
    for key in ["start_node", "start", "source", "startNode"]:
        if key in objectives and isinstance(objectives[key], int):
            return objectives[key]
    return 1


def infer_vehicle_counts(public_map: dict, objectives: dict) -> Tuple[int, int]:
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
