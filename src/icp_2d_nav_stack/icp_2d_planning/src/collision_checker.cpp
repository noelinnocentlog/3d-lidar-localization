#include "icp_2d_planning/collision_checker.hpp"

#include <cmath>
#include <algorithm>

namespace icp_planning {

void CollisionChecker::set_map(const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
{
    width_  = static_cast<int>(map->info.width);
    height_ = static_cast<int>(map->info.height);
    res_    = map->info.resolution;
    ox_     = map->info.origin.position.x;
    oy_     = map->info.origin.position.y;
    data_   = map->data;
}

bool CollisionChecker::world_to_cell(double x, double y, int & ix, int & iy) const
{
    ix = static_cast<int>((x - ox_) / res_);
    iy = static_cast<int>((y - oy_) / res_);
    return ix >= 0 && ix < width_ && iy >= 0 && iy < height_;
}

bool CollisionChecker::cell_occupied(int ix, int iy) const
{
    if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) return true;
    int8_t val = data_[static_cast<size_t>(iy * width_ + ix)];
    return val < 0 || val >= OCC_THRESHOLD;
}

// ── Pose collision check: fast circle pre-filter then full footprint ──────────
bool CollisionChecker::is_pose_free(double x, double y, double theta) const
{
    if (data_.empty()) return false;

    int cells = static_cast<int>(std::ceil(BOUNDING_RADIUS / res_));
    int cx, cy;
    if (!world_to_cell(x, y, cx, cy)) return false;

    for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
            double wx = dx * res_;
            double wy = dy * res_;
            if (wx*wx + wy*wy <= BOUNDING_RADIUS * BOUNDING_RADIUS) {
                if (cell_occupied(cx + dx, cy + dy))
                    return footprint_free(x, y, theta);
            }
        }
    }
    return true;
}

// ── Full oriented-rectangle footprint check ───────────────────────────────────
bool CollisionChecker::footprint_free(double x, double y, double theta) const
{
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto & c : CORNERS) {
        double wx = x + cos_t * c[0] - sin_t * c[1];
        double wy = y + sin_t * c[0] + cos_t * c[1];
        min_x = std::min(min_x, wx); max_x = std::max(max_x, wx);
        min_y = std::min(min_y, wy); max_y = std::max(max_y, wy);
    }

    int ix0, iy0, ix1, iy1;
    world_to_cell(min_x, min_y, ix0, iy0);
    world_to_cell(max_x, max_y, ix1, iy1);

    // Clamp to map bounds so out-of-boundary AABB corners don't skip real cells
    ix0 = std::max(0, ix0);  iy0 = std::max(0, iy0);
    ix1 = std::min(width_  - 1, ix1);
    iy1 = std::min(height_ - 1, iy1);

    double ax0 = cos_t, ay0 = sin_t;
    double ax1 = -sin_t, ay1 = cos_t;

    for (int iy = iy0; iy <= iy1; ++iy) {
        for (int ix = ix0; ix <= ix1; ++ix) {
            if (!cell_occupied(ix, iy)) continue;
            double dx = ox_ + (ix + 0.5) * res_ - x;
            double dy = oy_ + (iy + 0.5) * res_ - y;
            double proj0 = dx * ax0 + dy * ay0;
            double proj1 = dx * ax1 + dy * ay1;
            if (std::fabs(proj0) <= HL + res_ * 0.5 &&
                std::fabs(proj1) <= HW + res_ * 0.5)
                return false;
        }
    }
    return true;
}

// ── Arc collision: sample poses along the arc ─────────────────────────────────
bool CollisionChecker::is_arc_free(const Node3D & from, const Primitive & prim) const
{
    double x     = from.x;
    double y     = from.y;
    double theta = from.theta;

    double dist_per_step = std::fabs(prim.v) * SIM_DT;
    int    check_steps   = std::max(1, static_cast<int>(dist_per_step / ARC_STEP));
    double sub_dt        = SIM_DT / check_steps;

    for (int s = 0; s < SIM_STEPS; ++s) {
        for (int c = 0; c < check_steps; ++c) {
            x     += prim.v     * std::cos(theta) * sub_dt;
            y     += prim.v     * std::sin(theta) * sub_dt;
            theta += prim.omega * sub_dt;
            if (!is_pose_free(x, y, theta)) return false;
        }
    }
    return true;
}

// ── Straight segment check (smoother shortcutting) ────────────────────────────
bool CollisionChecker::is_segment_free(double x0, double y0,
                                        double x1, double y1) const
{
    double dx  = x1 - x0;
    double dy  = y1 - y0;
    double len = std::hypot(dx, dy);
    if (len < 1e-6) return true;

    int    steps   = std::max(1, static_cast<int>(len / ARC_STEP));
    double heading = std::atan2(dy, dx);

    for (int i = 0; i <= steps; ++i) {
        double t  = static_cast<double>(i) / steps;
        if (!is_pose_free(x0 + t * dx, y0 + t * dy, heading)) return false;
    }
    return true;
}

// ── Euclidean clearance at a world point ──────────────────────────────────────
// Returns true Euclidean distance to nearest obstacle cell centre.
double CollisionChecker::clearance(double x, double y) const
{
    if (data_.empty()) return 0.0;

    int max_cells = static_cast<int>(std::ceil(2.0 / res_));
    int cx, cy;
    if (!world_to_cell(x, y, cx, cy)) return 0.0;

    for (int r = 0; r <= max_cells; ++r) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (std::abs(dx) != r && std::abs(dy) != r) continue;
                if (cell_occupied(cx + dx, cy + dy))
                    return std::hypot(dx, dy) * res_;   // true Euclidean distance
            }
        }
    }
    return max_cells * res_;
}

}  // namespace icp_planning
