#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace icp_planning {

// ── Robot physical constants (Husky with safety margin) ─────────────────────
static constexpr double ROBOT_LENGTH       = 1.05;   // m
static constexpr double ROBOT_WIDTH        = 0.75;   // m
static constexpr double BOUNDING_RADIUS    = 0.64;   // m  (half-diagonal + margin)
static constexpr double MIN_TURN_RADIUS    = 0.50;   // m  used by Reeds-Shepp

// ── Kinematic primitives ─────────────────────────────────────────────────────
static constexpr double SIM_DT             = 0.10;   // s  per integration step
static constexpr int    SIM_STEPS          = 5;      // steps per primitive arc
static constexpr double ARC_STEP           = 0.05;   // m  collision check spacing along arc

// ── State-space discretisation ───────────────────────────────────────────────
static constexpr double XY_RESOLUTION      = 0.05;   // m  (matches map_002)
static constexpr int    HEADING_BINS       = 72;     // 5° per bin
static constexpr double HEADING_RES        = (2.0 * M_PI) / HEADING_BINS;

// ── Cost weights ─────────────────────────────────────────────────────────────
static constexpr double REVERSE_PENALTY           = 3.0;
static constexpr double DIRECTION_CHANGE_PENALTY  = 10.0;
static constexpr double STEER_CHANGE_PENALTY      = 0.5;
static constexpr double HEURISTIC_WEIGHT          = 1.05;

// ── Search limits ────────────────────────────────────────────────────────────
static constexpr int    MAX_ITERATIONS     = 1'000'000;
static constexpr double PLANNING_TIMEOUT   = 3.0;    // s

// ── Analytic expansion ───────────────────────────────────────────────────────
static constexpr int    ANALYTIC_EXPAND_N  = 5;      // try every N pops
static constexpr double ANALYTIC_MAX_LEN   = 4.0;    // m  skip if farther

// ── Path smoother ────────────────────────────────────────────────────────────
static constexpr int    SMOOTHER_MAX_ITER  = 150;
static constexpr double SMOOTHER_W_SMOOTH  = 0.4;
static constexpr double SMOOTHER_W_DATA    = 0.2;
static constexpr double SMOOTHER_W_OBS     = 0.4;
static constexpr double SMOOTHER_MIN_CLEAR = 0.80;   // m  obstacle clearance target

// ── Occupancy threshold ──────────────────────────────────────────────────────
static constexpr int8_t OCC_THRESHOLD      = 50;     // >= this is obstacle

// ── Direction enum ───────────────────────────────────────────────────────────
enum class Direction : int8_t { FORWARD = 1, REVERSE = -1 };

// ── Utility ──────────────────────────────────────────────────────────────────
inline double normalise_angle(double a)
{
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    return (a < 0.0) ? a + M_PI : a - M_PI;
}

static constexpr double INF = std::numeric_limits<double>::infinity();

}  // namespace icp_planning
