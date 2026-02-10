from modules.io_utilization import load_inputs, write_solution
from modules.planner import plan, score_solution

def main():
    public_map, sensor_data, objectives = load_inputs()

    solution = plan(public_map, sensor_data, objectives)

    total_score, per_obj, travel_cost = score_solution(
        objectives,
        solution
    )

    write_solution(
        solution,
        total_score,
        travel_cost,
        per_obj
    )

if __name__ == "__main__":
    main()
