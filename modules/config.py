from typing import Tuple, Iterable, Optional, Any


def _first_int(d: dict, keys: Iterable[str]) -> Optional[int]:
    """Return the first value in d[key] that exists and is an int, else None."""
    for k in keys:
        v = d.get(k, None)
        if isinstance(v, int):
            return v
    return None


def _first_nonempty_list_len(d: dict) -> Optional[int]:
    """Return length of the first non-empty list found in dict values, else None."""
    for v in d.values():
        if isinstance(v, list) and v:
            return len(v)
    return None


def infer_T(public_map: dict, sensor_data: dict) -> int:
    # 1) Prefer explicit T-like fields in public_map
    T = _first_int(public_map, ["T", "T_max", "t_max", "time_steps", "num_time_steps"])
    if T is not None:
        return T

    # 2) Otherwise infer from first non-empty sensor list length
    T = _first_nonempty_list_len(sensor_data)
    if T is not None:
        return T

    raise ValueError("Could not infer T from inputs.")


def infer_start_node(objectives: dict) -> int:
    # Prefer explicit start fields, else default to 1
    start = _first_int(objectives, ["start_node", "start", "source", "startNode"])
    return start if start is not None else 1


def infer_vehicle_counts(public_map: dict, objectives: dict) -> Tuple[int, int]:
    # 1) Try common key pairs in public_map, then objectives
    key_pairs = [
        ("num_trucks", "num_drones"),
        ("truck_count", "drone_count"),
        ("n_trucks", "n_drones"),
        ("trucks", "drones"),
    ]

    for a, b in key_pairs:
        t = public_map.get(a)
        d = public_map.get(b)
        if isinstance(t, int) and isinstance(d, int):
            return t, d

        t = objectives.get(a)
        d = objectives.get(b)
        if isinstance(t, int) and isinstance(d, int):
            return t, d

    # 2) Try nested public_map["vehicles"] dict
    vehicles = public_map.get("vehicles")
    if isinstance(vehicles, dict):
        t = vehicles.get("num_trucks", vehicles.get("trucks", 1))
        d = vehicles.get("num_drones", vehicles.get("drones", 1))
        # Coerce to int safely + default to 1 if None/0/False
        return int(t or 1), int(d or 1)

    # 3) Final fallback
    return 1, 1
