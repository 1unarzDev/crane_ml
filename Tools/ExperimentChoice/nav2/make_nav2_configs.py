import copy, sys, yaml
from pathlib import Path
root = Path(sys.argv[1])
base = yaml.safe_load((root / "src/modular_robot_bringup/config/nav2.yaml").read_text())
out_dir = root / "src/modular_robot_bringup/config/expchoice"
out_dir.mkdir(exist_ok=True)

SMAC = {
    "plugin": "nav2_smac_planner::SmacPlannerHybrid",
    "downsample_costmap": True, "downsampling_factor": 5,
    "tolerance": 0.25, "allow_unknown": True,
    "max_iterations": 1000000, "max_on_approach_iterations": 1000,
    "max_planning_time": 5.0, "terminal_checking_interval": 5000,
    "motion_model_for_search": "REEDS_SHEPP", "angle_quantization_bins": 72,
    "analytic_expansion_ratio": 3.5, "analytic_expansion_max_length": 3.0,
    "analytic_expansion_max_cost": 200.0, "analytic_expansion_max_cost_override": False,
    "minimum_turning_radius": 0.4, "reverse_penalty": 2.0, "change_penalty": 0.0,
    "non_straight_penalty": 1.2, "cost_penalty": 2.0, "retrospective_penalty": 0.015,
    "lookup_table_size": 20.0, "cache_obstacle_heuristic": False,
    "debug_visualizations": False, "use_quadratic_cost_penalty": False,
    "downsample_obstacle_heuristic": True, "allow_primitive_interpolation": False,
    "smooth_path": True,
    "smoother": {"max_iterations": 1000, "w_smooth": 0.3, "w_data": 0.2,
                 "tolerance": 1.0e-10, "do_refinement": True, "refinement_num": 2},
}
MPPI_LIMITS = {  # (vx, wz) = the RPP arm's desired_linear_vel and rotate_to_heading_angular_vel
    "compact_diff": (0.35, 0.6), "narrow_tandem": (0.18, 0.4)}
def mppi(morph, transform_tolerance):
    vx, wz = MPPI_LIMITS[morph]
    return {
        "plugin": "nav2_mppi_controller::MPPIController",
        "time_steps": 56, "model_dt": 0.05, "batch_size": 2000,
        # vy/ay are unused by the DiffDrive model and left at Nav2's stock values.
        "ax_max": 1.2, "ax_min": -1.2, "ay_max": 3.0, "ay_min": -3.0, "az_max": 2.0,
        "vx_std": 0.2, "vy_std": 0.2, "wz_std": 0.4,
        "vx_max": vx, "vx_min": -vx, "vy_max": 0.5, "wz_max": wz,
        "iteration_count": 1, "prune_distance": 1.7,
        "transform_tolerance": transform_tolerance,
        "temperature": 0.3, "gamma": 0.015, "motion_model": "DiffDrive",
        "visualize": False, "regenerate_noises": True,
        "critics": ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
                    "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
                    "PreferForwardCritic"],
        "ConstraintCritic": {"enabled": True, "cost_power": 1, "cost_weight": 4.0},
        "GoalCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0,
                       "threshold_to_consider": 1.4},
        "GoalAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.0,
                            "threshold_to_consider": 0.5},
        "PreferForwardCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0,
                                "threshold_to_consider": 0.5},
        "CostCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.81,
                       "near_collision_cost": 253, "critical_cost": 300.0,
                       "consider_footprint": True, "collision_cost": 1000000.0,
                       "near_goal_distance": 1.0, "trajectory_point_step": 2},
        "PathAlignCritic": {"enabled": True, "cost_power": 1, "cost_weight": 14.0,
                            "max_path_occupancy_ratio": 0.05, "trajectory_point_step": 4,
                            "threshold_to_consider": 0.5, "offset_from_furthest": 20,
                            "use_path_orientations": False},
        "PathFollowCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0,
                             "offset_from_furthest": 5, "threshold_to_consider": 1.4},
        "PathAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 2.0,
                            "offset_from_furthest": 4, "threshold_to_consider": 0.5,
                            "max_angle_to_furthest": 1.0, "mode": 0},
    }

HEADER = """# FROZEN - ExperimentChoice offset-doorway study (EXPLORATORY). Do not edit:
# the file's sha256 is bound in modular_robot_benchmarks/doorway_design.py and
# in every design that names it. To change anything, add a new file.
#
# Morphology: {morph}.  Arm {arm}: {what}
# Derived from config/nav2.yaml at 00ac590: controller block {block} renamed
# FollowPath (the id stock Nav2's default behaviour tree asks for); the other
# controllers and the keepout costmap filters removed (no mask is served for
# this family); everything else copied. Differences between the arms are
# listed in studies/expchoice_doorway_v0/NAV2_CONFIGS.md.
"""
WHAT = {
    "A": ("A_rpp_rotate_navfn",
          "Regulated Pure Pursuit, rotate to heading, no reversing; NavFn A*."),
    "B": ("B_mppi_reverse_navfn",
          "MPPI DiffDrive with reversing allowed (Nav2 Jazzy's stock critics);\n# NavFn A*."),
    "C": ("C_rpp_reverse_smac_rs",
          "Regulated Pure Pursuit, reversing allowed (no rotate to heading);\n"
          "# Smac Hybrid-A* with Reeds-Shepp motions."),
}
def build(morph, block, arm):
    cfg = copy.deepcopy(base)
    ctl = cfg["controller_server"]["ros__parameters"]
    follow = copy.deepcopy(ctl[block])
    for name in list(ctl["controller_plugins"]):
        del ctl[name]
    ctl["controller_plugins"] = ["FollowPath"]
    if arm == "B":
        follow = mppi(morph, follow["transform_tolerance"])
    else:
        # Explicit in both RPP arms, at Nav2's defaults for A.
        follow["use_rotate_to_heading"] = arm == "A"
        follow["allow_reversing"] = arm == "C"
    ctl["FollowPath"] = follow
    for name in ("local_costmap", "global_costmap"):
        params = cfg[name][name]["ros__parameters"]
        del params["filters"], params["keepout_filter"]
    if arm == "C":
        cfg["planner_server"]["ros__parameters"]["GridBased"] = copy.deepcopy(SMAC)
    stem, what = WHAT[arm]
    text = HEADER.format(morph=morph, arm=arm, what=what, block=block) + yaml.safe_dump(
        cfg, sort_keys=False, default_flow_style=False, width=100)
    (out_dir / f"{morph}_{stem}.yaml").write_text(text)
    assert yaml.safe_load(text) == cfg
for old in out_dir.glob("*.yaml"):
    old.unlink()
for morph, block in (("compact_diff", "FollowPathDiff"), ("narrow_tandem", "FollowPathNarrow")):
    for arm in "ABC":
        build(morph, block, arm)
