#include "icp_2d_planning/heuristic.hpp"
#include "icp_2d_planning/reeds_shepp.hpp"

#include <queue>
#include <limits>
#include <cmath>

namespace icp_planning {

bool Heuristic::world_to_cell(double x, double y, int & ix, int & iy) const
{
    ix = static_cast<int>((x - ox_) / res_);
    iy = static_cast<int>((y - oy_) / res_);
    return ix >= 0 && ix < width_ && iy >= 0 && iy < height_;
}

// ── 2D Dijkstra from goal, ignoring orientation ───────────────────────────────
// Treats unknown/obstacle cells as impassable; propagates cost outward.
// Using 8-connected grid with diagonal cost = √2 * res.
void Heuristic::precompute(const Node3D & goal, const CollisionChecker & cc)
{
    width_  = cc.width();
    height_ = cc.height();
    res_    = cc.resolution();
    ox_     = cc.origin_x();
    oy_     = cc.origin_y();

    const size_t N = static_cast<size_t>(width_ * height_);
    dijkstra_.assign(N, std::numeric_limits<float>::infinity());

    int gx, gy;
    if (!world_to_cell(goal.x, goal.y, gx, gy)) return;

    // min-heap: (cost, flat_index)
    using Entry = std::pair<float, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    size_t goal_idx = static_cast<size_t>(gy * width_ + gx);
    dijkstra_[goal_idx] = 0.0f;
    pq.push({0.0f, static_cast<int>(goal_idx)});

    const int dx8[8] = {1,-1,0, 0, 1, 1,-1,-1};
    const int dy8[8] = {0, 0,1,-1, 1,-1, 1,-1};
    const float costs8[8] = {
        static_cast<float>(res_),
        static_cast<float>(res_),
        static_cast<float>(res_),
        static_cast<float>(res_),
        static_cast<float>(res_ * M_SQRT2),
        static_cast<float>(res_ * M_SQRT2),
        static_cast<float>(res_ * M_SQRT2),
        static_cast<float>(res_ * M_SQRT2),
    };

    while (!pq.empty()) {
        auto [cost, idx] = pq.top(); pq.pop();

        if (cost > dijkstra_[static_cast<size_t>(idx)]) continue;   // stale entry

        int cx = idx % width_;
        int cy = idx / width_;

        for (int d = 0; d < 8; ++d) {
            int nx = cx + dx8[d];
            int ny = cy + dy8[d];
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
            if (cc.cell_occupied(nx, ny)) continue;

            size_t nidx    = static_cast<size_t>(ny * width_ + nx);
            float  new_cost = dijkstra_[static_cast<size_t>(idx)] + costs8[d];

            if (new_cost < dijkstra_[nidx]) {
                dijkstra_[nidx] = new_cost;
                pq.push({new_cost, static_cast<int>(nidx)});
            }
        }
    }
}

double Heuristic::h1(double x, double y) const
{
    if (dijkstra_.empty()) return 0.0;
    int ix, iy;
    if (!world_to_cell(x, y, ix, iy)) return INF;
    float v = dijkstra_[static_cast<size_t>(iy * width_ + ix)];
    return static_cast<double>(v);
}

double Heuristic::h2(const Node3D & node, const Node3D & goal) const
{
    // Reeds-Shepp path length as non-holonomic lower bound
    return reeds_shepp::path_length(
        node.x, node.y, node.theta,
        goal.x, goal.y, goal.theta,
        MIN_TURN_RADIUS);
}

double Heuristic::compute(const Node3D & node, const Node3D & goal) const
{
    double val_h1 = h1(node.x, node.y);
    double val_h2 = h2(node, goal);
    return std::max(val_h1, val_h2);
}

}  // namespace icp_planning
