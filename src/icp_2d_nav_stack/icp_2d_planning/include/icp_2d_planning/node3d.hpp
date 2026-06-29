#pragma once

#include "icp_2d_planning/types.hpp"
#include "icp_2d_planning/config.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace icp_planning {

struct Node3D
{
    double x      {0.0};
    double y      {0.0};
    double theta  {0.0};

    double g      {INF};
    double h      {0.0};
    // heuristic weight is stored per-node so f() is self-contained.
    // Must be set from PlannerConfig before the node is pushed to the open list.
    double hw     {1.0};
    double f() const { return g + hw * h; }

    int parent_idx {-1};
    int self_idx   {-1};

    Direction direction {Direction::FORWARD};
    double    omega     {0.0};

    // ── 3-D discretised cell key ─────────────────────────────────────────────
    // planner_w / planner_h are the grid dimensions in XY_RESOLUTION-sized cells,
    // computed by the caller as:
    //   planner_w = ceil(map_width  * map_res / XY_RESOLUTION) + 1
    //   planner_h = ceil(map_height * map_res / XY_RESOLUTION) + 1
    // Using these (not map_width/height in map-resolution cells) prevents
    // key collisions when map_res != XY_RESOLUTION.
    int64_t cell_key(int planner_w, int planner_h) const
    {
        int ix = static_cast<int>(std::floor(x / XY_RESOLUTION));
        int iy = static_cast<int>(std::floor(y / XY_RESOLUTION));
        int it = heading_bin();

        ix = std::max(0, std::min(ix, planner_w - 1));
        iy = std::max(0, std::min(iy, planner_h - 1));

        return static_cast<int64_t>(ix)
             + static_cast<int64_t>(iy) * planner_w
             + static_cast<int64_t>(it) * planner_w * planner_h;
    }

    int heading_bin() const
    {
        double n = theta;
        while (n <  0.0)        n += 2.0 * M_PI;
        while (n >= 2.0 * M_PI) n -= 2.0 * M_PI;
        int bin = static_cast<int>(n / HEADING_RES);
        if (bin < 0)             bin = 0;
        if (bin >= HEADING_BINS) bin = HEADING_BINS - 1;
        return bin;
    }

    bool is_close_to(const Node3D & goal, double xy_tol, double theta_tol) const
    {
        double dx = x - goal.x;
        double dy = y - goal.y;
        double da = std::fabs(normalise_angle(theta - goal.theta));
        return (dx*dx + dy*dy) < xy_tol*xy_tol && da < theta_tol;
    }
};

}  // namespace icp_planning
