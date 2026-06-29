#pragma once

#include "icp_2d_planning/types.hpp"
#include "icp_2d_planning/node3d.hpp"

#include <array>
#include <cmath>

namespace icp_planning {

// ── (v, ω) pair describing one kinematic primitive ───────────────────────────
struct Primitive
{
    double v;      // linear velocity (m/s) — negative = reverse
    double omega;  // angular velocity (rad/s)
    Direction dir;
};

// ── 14 primitives: 7 forward + 7 reverse ────────────────────────────────────
static constexpr std::array<Primitive, 14> PRIMITIVES = {{
    // forward
    { 1.0,  0.000, Direction::FORWARD },
    { 0.8,  0.400, Direction::FORWARD },
    { 0.8, -0.400, Direction::FORWARD },
    { 0.6,  0.900, Direction::FORWARD },
    { 0.6, -0.900, Direction::FORWARD },
    { 0.3,  1.500, Direction::FORWARD },
    { 0.3, -1.500, Direction::FORWARD },
    // reverse
    {-1.0,  0.000, Direction::REVERSE },
    {-0.8,  0.400, Direction::REVERSE },
    {-0.8, -0.400, Direction::REVERSE },
    {-0.6,  0.900, Direction::REVERSE },
    {-0.6, -0.900, Direction::REVERSE },
    {-0.3,  1.500, Direction::REVERSE },
    {-0.3, -1.500, Direction::REVERSE },
}};

// ── Forward-integrate one primitive from a parent node ───────────────────────
inline Node3D apply_primitive(const Node3D & parent, const Primitive & prim)
{
    Node3D child;
    child.x     = parent.x;
    child.y     = parent.y;
    child.theta = parent.theta;
    child.direction = prim.dir;
    child.omega     = prim.omega;
    child.hw        = parent.hw;   // propagate heuristic weight

    for (int s = 0; s < SIM_STEPS; ++s) {
        child.x     += prim.v * std::cos(child.theta) * SIM_DT;
        child.y     += prim.v * std::sin(child.theta) * SIM_DT;
        child.theta += prim.omega * SIM_DT;
    }
    child.theta = normalise_angle(child.theta);
    return child;
}

// ── Arc length of one primitive (always positive) ────────────────────────────
inline double primitive_length(const Primitive & prim)
{
    return std::fabs(prim.v) * SIM_DT * SIM_STEPS;
}

}  // namespace icp_planning
