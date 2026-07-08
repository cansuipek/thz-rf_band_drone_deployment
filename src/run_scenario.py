"""
Command-line runner for one configured UAV RF/THz deployment scenario.
python src/run_scenario.py --config configs/dense_urban_50users_200_300m_rmin5.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path

from optimization_model import solve_drone_deployment_gurobi
from scenario_builder import build_scenario
from utils import ensure_dir, load_config, parse_time_limit, save_json
from visualization import analyze_and_visualize_solution


def print_solution_summary(sol: dict) -> None:
    print("\n SOLUTION SUMMARY")
    print("Status:", sol.get("status_str"), "Obj:", sol.get("obj"))
    print("t_air (0 RF, 1 THz):", sol.get("t_air"))
    print("t_space (0 RF, 1 THz):", sol.get("t_space"))
    print("Deployed:", sol.get("deployed"))
    print("Masters :", sol.get("masters"))
    print("Slaves  :", sol.get("slaves"))

    print("\nA2A LINKS ")
    print("RF links :", sol.get("a2a_rf_links"))
    print("THz links:", sol.get("a2a_thz_links"))

    print("\nUSER ASSIGNMENTS (first 20)")
    assign = sol.get("assignment", {})
    for i, (u, d) in enumerate(assign.items()):
        if i >= 20:
            break
        print(f"User {u} -> Drone {d}")

    if "status_str" in sol:
        print(sol["status_str"], "obj=", sol.get("obj"))
    print("t_air=", sol.get("t_air"), "masters=", sol.get("masters"), "slaves=", sol.get("slaves"))

    print("\nA2A FORWARDED FLOWS")
    for link, flow in sol.get("a2a_flow", {}).items():
        print(f"{link}: {flow:.3f} Mbps")

    print("\nA2A RF SHARES")
    for link, share in sol.get("a2a_phi_rf", {}).items():
        print(f"{link}: {share:.3f}")

    print("\nA2A THz SHARES")
    for link, share in sol.get("a2a_phi_thz", {}).items():
        print(f"{link}: {share:.3f}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="configs/dense_urban_50users_200_300m_rmin5.yaml",
        help="Path to a YAML scenario config file.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Solve only; skip the PDF visualization.",
    )
    args = parser.parse_args()

    config = load_config(args.config)

    scenario_cfg = config.get("scenario", {})
    solver_cfg = config.get("solver", {})
    outputs_cfg = config.get("outputs", {})

    scenario_name = scenario_cfg.get("name", "scenario")
    output_dir = Path(scenario_cfg.get("output_dir", f"results/{scenario_name}"))
    ensure_dir(output_dir)

    data, user_xy, cand = build_scenario(config)

    sol = solve_drone_deployment_gurobi(
        data,
        time_limit_s=parse_time_limit(solver_cfg.get("time_limit_s", "inf")),
        mip_gap=float(solver_cfg.get("mip_gap", 0.1)),
        verbose=True,
    )

    print_solution_summary(sol)

    # Save a JSON-friendly solution copy. Tuple keys are converted to strings.
    save_json(sol, output_dir / "solution_summary.json")

    if not args.no_plot:
        plot_filename = outputs_cfg.get("plot_filename", "drone_deployment.pdf")
        analyze_and_visualize_solution(
            sol,
            data,
            user_xy,
            cand,
            filename=str(output_dir / plot_filename),
        )


if __name__ == "__main__":
    main()
