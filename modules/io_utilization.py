import json
from typing import Any, Dict, List, Tuple

def load_json(path: str) -> Any:

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_inputs() -> Tuple[dict, dict, dict]:

    public_map = load_json("public_map.json")
    sensor_data = load_json("sensor_data.json")
    objectives = load_json("objectives.json")
    return public_map, sensor_data, objectives


def write_solution(solution: Dict[str, List[int]]) -> None:

    with open("solution.json", "w", encoding="utf-8") as f:
        json.dump(solution, f, indent=2)
