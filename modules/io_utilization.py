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



def write_solution(
    solution: Dict[str, List[int]],
    total_score: float,
    travel_cost: int,
    objective_scores: Dict[Any, float],
) -> None:
    out = {
        "score": total_score,
        "travel_cost": travel_cost,
        "objective_scores": objective_scores,
        "routes": solution,
    }

    with open("solution.json", "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)

