#pragma once

#include "icp_2d_planning/types.hpp"

namespace icp_planning {

// All runtime-tunable planner parameters in one place.
// Defaults mirror the compile-time constants in types.hpp.
// The ROS node populates this from YAML and passes it to each component.
struct PlannerConfig
{
    // ── Cost weights ──────────────────────────────────────────────────────────
    double reverse_penalty          = REVERSE_PENALTY;
    double direction_change_penalty = DIRECTION_CHANGE_PENALTY;
    double steer_change_penalty     = STEER_CHANGE_PENALTY;
    double heuristic_weight         = HEURISTIC_WEIGHT;

    // ── Search limits ─────────────────────────────────────────────────────────
    int    max_iterations           = MAX_ITERATIONS;
    double planning_timeout         = PLANNING_TIMEOUT;

    // ── Analytic expansion ────────────────────────────────────────────────────
    int    analytic_expand_n        = ANALYTIC_EXPAND_N;
    double analytic_max_len         = ANALYTIC_MAX_LEN;

    // ── Path smoother ─────────────────────────────────────────────────────────
    double shortcut_max_dist        = 1.5;        // m  max skip per shortcut step
    int    smoother_max_iter        = SMOOTHER_MAX_ITER;
    double smoother_lr              = 0.05;        // gradient step size
    double smoother_w_smooth        = SMOOTHER_W_SMOOTH;
    double smoother_w_data          = SMOOTHER_W_DATA;
    double smoother_w_obs           = SMOOTHER_W_OBS;
    double smoother_min_clear       = 0.60;   // matches planner.yaml; types.hpp has a different value

    // ── Goal tolerance ────────────────────────────────────────────────────────
    double goal_xy_tol              = 0.15;        // m
    double goal_theta_tol           = 0.20;        // rad

    // ── Robot footprint (rarely changed) ─────────────────────────────────────
    double robot_length             = ROBOT_LENGTH;
    double robot_width              = ROBOT_WIDTH;
    double bounding_radius          = BOUNDING_RADIUS;
    // 0.2m = v_min/omega_max = 0.3/1.5 — matches tightest primitive, keeps h2 admissible
    double min_turn_radius          = 0.20;
};

}  // namespace icp_planning
