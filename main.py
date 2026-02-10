from modules.io_utilization import load_inputs, write_solution
from modules.planner import plan


def main() -> None:
    public_map, sensor_data, objectives = load_inputs()
    solution = plan(public_map, sensor_data, objectives)
    write_solution(solution)
    print("solution.json generated successfully.")


if __name__ == "__main__":
    main()
