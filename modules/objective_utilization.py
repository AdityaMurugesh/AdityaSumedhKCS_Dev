from typing import Any, Dict, List


def parse_objectives(objectives_json: dict) -> List[Dict[str, Any]]:
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

    out: List[Dict[str, Any]] = []
    for idx, o in enumerate(obj_list):
        if not isinstance(o, dict):
            continue
        out.append({
            "id": o.get("id", idx),
            "node": get_int(o, ["node", "target", "target_node", "location"], 1),
            "release": get_int(o, ["T_start", "release", "start", "start_time"], 0),
            "deadline": get_int(o, ["T_end", "deadline", "end", "end_time"], 0),
            "points": get_int(o, ["P_max", "points", "reward", "max_points"], 0),
        })

    return out


def sort_objectives(objs: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return sorted(objs, key=lambda o: (o["deadline"], o["release"]))
