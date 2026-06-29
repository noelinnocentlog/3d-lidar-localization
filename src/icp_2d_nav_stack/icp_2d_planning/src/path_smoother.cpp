#include "icp_2d_planning/path_smoother.hpp"

#include <cmath>
#include <algorithm>

namespace icp_planning {

// ── Phase 1: shortcut pruner ─────────────────────────────────────────────────
void PathSmoother::shortcut(std::vector<Node3D> & path,
                             const CollisionChecker & cc) const
{
    if (path.size() < 3) return;

    // Save goal heading — must survive all smoothing phases
    const double goal_theta = path.back().theta;

    std::vector<Node3D> pruned;
    pruned.push_back(path.front());

    size_t i = 0;
    while (i < path.size() - 1) {
        size_t best_j = i + 1;
        for (size_t j = path.size() - 1; j > i + 1; --j) {
            double dx = path[j].x - path[i].x;
            double dy = path[j].y - path[i].y;
            if (std::hypot(dx, dy) > cfg_.shortcut_max_dist) continue;
            if (cc.is_segment_free(path[i].x, path[i].y,
                                    path[j].x, path[j].y)) {
                best_j = j;
                break;
            }
        }
        pruned.push_back(path[best_j]);
        i = best_j;
    }

    // Recompute intermediate headings from position deltas
    for (size_t k = 0; k + 1 < pruned.size(); ++k) {
        double dx = pruned[k+1].x - pruned[k].x;
        double dy = pruned[k+1].y - pruned[k].y;
        if (std::hypot(dx, dy) > 1e-6)
            pruned[k].theta = std::atan2(dy, dx);
    }
    // Restore the user-requested goal heading (do not overwrite with approach vector)
    pruned.back().theta = goal_theta;

    path = std::move(pruned);
}

// ── Phase 2: gradient-descent smoother ───────────────────────────────────────
void PathSmoother::gradient_descent(std::vector<Node3D> & path,
                                     const CollisionChecker & cc) const
{
    if (path.size() < 3) return;

    const size_t N = path.size();
    const double goal_theta = path.back().theta;   // preserve before smoothing

    std::vector<double> ox(N), oy(N);
    for (size_t i = 0; i < N; ++i) { ox[i] = path[i].x; oy[i] = path[i].y; }

    const double step = cc.resolution();   // finite-difference step = 1 map cell

    for (int iter = 0; iter < cfg_.smoother_max_iter; ++iter) {
        double total_change = 0.0;

        for (size_t i = 1; i + 1 < N; ++i) {
            double gx = 0.0, gy = 0.0;

            // Smoothness: penalise second derivative of position
            double smx = path[i].x - 0.5 * (path[i-1].x + path[i+1].x);
            double smy = path[i].y - 0.5 * (path[i-1].y + path[i+1].y);
            gx += cfg_.smoother_w_smooth * 2.0 * smx;
            gy += cfg_.smoother_w_smooth * 2.0 * smy;

            // Data fidelity: stay close to original path
            gx += cfg_.smoother_w_data * (path[i].x - ox[i]);
            gy += cfg_.smoother_w_data * (path[i].y - oy[i]);

            // Obstacle repulsion using a one-cell finite difference
            double clr = cc.clearance(path[i].x, path[i].y);
            if (clr < cfg_.smoother_min_clear && clr > 1e-3) {
                double deficit  = cfg_.smoother_min_clear - clr;
                double dx_clr   = cc.clearance(path[i].x + step, path[i].y) - clr;
                double dy_clr   = cc.clearance(path[i].x, path[i].y + step) - clr;
                gx -= cfg_.smoother_w_obs * (dx_clr / step) * deficit;
                gy -= cfg_.smoother_w_obs * (dy_clr / step) * deficit;
            }

            double nx = path[i].x - cfg_.smoother_lr * gx;
            double ny = path[i].y - cfg_.smoother_lr * gy;

            if (cc.is_pose_free(nx, ny, path[i].theta)) {
                total_change += std::hypot(nx - path[i].x, ny - path[i].y);
                path[i].x = nx;
                path[i].y = ny;
            }
        }

        if (total_change < 1e-4) break;
    }

    // Recompute intermediate headings from updated positions
    for (size_t i = 0; i + 1 < N; ++i) {
        double dx = path[i+1].x - path[i].x;
        double dy = path[i+1].y - path[i].y;
        if (std::hypot(dx, dy) > 1e-6)
            path[i].theta = std::atan2(dy, dx);
    }
    // Restore user-requested goal heading
    path.back().theta = goal_theta;
}

void PathSmoother::smooth(std::vector<Node3D> & path,
                           const CollisionChecker & cc) const
{
    if (path.size() < 2) return;
    shortcut(path, cc);
    gradient_descent(path, cc);
}

}  // namespace icp_planning
