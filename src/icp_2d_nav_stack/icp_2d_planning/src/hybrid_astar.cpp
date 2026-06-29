#include "icp_2d_planning/hybrid_astar.hpp"
#include "icp_2d_planning/motion_primitives.hpp"
#include "icp_2d_planning/reeds_shepp.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>

namespace icp_planning {

void HybridAstar::set_map(const nav_msgs::msg::OccupancyGrid::SharedPtr & map)
{
    cc_.set_map(map);
}

double HybridAstar::transition_cost(const Node3D & parent, const Node3D & child,
                                     const Primitive & prim) const
{
    double cost = primitive_length(prim);
    if (child.direction == Direction::REVERSE)
        cost += cfg_.reverse_penalty * primitive_length(prim);
    if (child.direction != parent.direction)
        cost += cfg_.direction_change_penalty;
    cost += cfg_.steer_change_penalty * std::fabs(prim.omega - parent.omega);
    return cost;
}

// ── Analytic expansion — node taken by VALUE to avoid dangling ref on push_back
bool HybridAstar::try_analytic_expansion(Node3D node, const Node3D & goal,
                                          int node_idx,
                                          std::vector<Node3D> & pool,
                                          PlanResult & result)
{
    double dist = std::hypot(node.x - goal.x, node.y - goal.y);
    if (dist > cfg_.analytic_max_len) return false;

    reeds_shepp::RSPath rs = reeds_shepp::shortest_path(
        node.x, node.y, node.theta,
        goal.x, goal.y, goal.theta,
        cfg_.min_turn_radius);

    if (rs.total_length > cfg_.analytic_max_len) return false;

    // Sample the RS path — each SampledPose carries its own direction
    auto poses = reeds_shepp::sample_path(
        rs, node.x, node.y, node.theta, cfg_.min_turn_radius, ARC_STEP);

    // Collision check every sampled pose
    for (const auto & sp : poses) {
        if (!cc_.is_pose_free(sp.x, sp.y, sp.theta)) return false;
    }

    // Build path nodes, preserving per-pose direction from the RS segments
    double running_g = node.g;
    int    parent    = node_idx;

    for (size_t i = 1; i < poses.size(); ++i) {
        Node3D n;
        n.x          = poses[i].x;
        n.y          = poses[i].y;
        n.theta      = poses[i].theta;
        n.hw         = cfg_.heuristic_weight;
        n.direction  = (poses[i].dir == reeds_shepp::SegDir::FORWARD)
                       ? Direction::FORWARD : Direction::REVERSE;
        running_g   += std::hypot(n.x - poses[i-1].x, n.y - poses[i-1].y);
        n.g          = running_g;
        n.h          = 0.0;
        n.parent_idx = parent;
        n.self_idx   = static_cast<int>(pool.size());
        pool.push_back(n);
        parent = n.self_idx;
    }

    Node3D gnode  = goal;
    gnode.g          = running_g;
    gnode.h          = 0.0;
    gnode.hw         = cfg_.heuristic_weight;
    gnode.parent_idx = parent;
    gnode.self_idx   = static_cast<int>(pool.size());
    pool.push_back(gnode);

    result.success = true;
    result.cost    = running_g;
    result.path    = reconstruct(gnode.self_idx, pool);
    return true;
}

// ── Main search ───────────────────────────────────────────────────────────────
PlanResult HybridAstar::plan(const Node3D & start, const Node3D & goal)
{
    PlanResult result;
    if (!cc_.has_map()) return result;

    // Early-exit: start at goal
    if (start.is_close_to(goal, cfg_.goal_xy_tol, cfg_.goal_theta_tol)) {
        result.success = true;
        result.cost    = 0.0;
        result.path    = {start};
        return result;
    }

    if (!cc_.is_pose_free(start.x, start.y, start.theta) ||
        !cc_.is_pose_free(goal.x,  goal.y,  goal.theta)) {
        return result;
    }

    heuristic_.precompute(goal, cc_);

    // Planner-resolution grid dimensions (may differ from map resolution)
    int planner_w = static_cast<int>(
        std::ceil(cc_.width()  * cc_.resolution() / XY_RESOLUTION)) + 1;
    int planner_h = static_cast<int>(
        std::ceil(cc_.height() * cc_.resolution() / XY_RESOLUTION)) + 1;

    std::vector<Node3D> pool;
    pool.reserve(50'000);

    std::unordered_map<int64_t, double> visited;
    visited.reserve(50'000);

    auto cmp = [&pool](int a, int b) { return pool[a].f() > pool[b].f(); };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

    Node3D s     = start;
    s.g          = 0.0;
    s.hw         = cfg_.heuristic_weight;
    s.h          = heuristic_.compute(s, goal);
    s.parent_idx = -1;
    s.self_idx   = 0;
    pool.push_back(s);
    open.push(0);

    int  iter          = 0;
    int  analytic_tick = 0;
    auto t0            = std::chrono::steady_clock::now();

    while (!open.empty() && iter < cfg_.max_iterations) {
        ++iter;

        if (iter % 1000 == 0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            if (elapsed > cfg_.planning_timeout) break;
        }

        int cur_idx = open.top(); open.pop();

        // Stale-entry check (lazy deletion)
        int64_t key = pool[cur_idx].cell_key(planner_w, planner_h);
        auto it = visited.find(key);
        if (it != visited.end() && it->second <= pool[cur_idx].g) continue;
        visited[key] = pool[cur_idx].g;

        if (pool[cur_idx].is_close_to(goal, cfg_.goal_xy_tol, cfg_.goal_theta_tol)) {
            result.success = true;
            result.cost    = pool[cur_idx].g;
            result.path    = reconstruct(cur_idx, pool);
            return result;
        }

        ++analytic_tick;
        if (analytic_tick >= cfg_.analytic_expand_n) {
            analytic_tick = 0;
            // Pass node by value (M-4 fix: pool.push_back inside would invalidate ref)
            if (try_analytic_expansion(pool[cur_idx], goal, cur_idx, pool, result))
                return result;
        }

        // Cache parent values before any push_back that could reallocate pool
        const double par_g     = pool[cur_idx].g;
        const double par_x     = pool[cur_idx].x;
        const double par_y     = pool[cur_idx].y;
        const double par_theta = pool[cur_idx].theta;
        const double par_omega = pool[cur_idx].omega;
        const Direction par_dir = pool[cur_idx].direction;

        for (const auto & prim : PRIMITIVES) {
            // Build a local parent node from cached values for arc check + cost
            Node3D par_node;
            par_node.x         = par_x;
            par_node.y         = par_y;
            par_node.theta     = par_theta;
            par_node.omega     = par_omega;
            par_node.direction = par_dir;

            // Arc collision check along the entire primitive
            if (!cc_.is_arc_free(par_node, prim)) continue;

            Node3D child = apply_primitive(par_node, prim);
            child.parent_idx = cur_idx;
            child.hw         = cfg_.heuristic_weight;

            // Endpoint pose check is already covered by is_arc_free above
            double edge_cost = transition_cost(par_node, child, prim);
            child.g = par_g + edge_cost;

            child.self_idx = static_cast<int>(pool.size());
            int64_t ckey   = child.cell_key(planner_w, planner_h);
            auto cit       = visited.find(ckey);
            if (cit != visited.end() && cit->second <= child.g) continue;

            child.h = heuristic_.compute(child, goal);
            pool.push_back(child);
            open.push(child.self_idx);
        }
    }

    return result;
}

std::vector<Node3D> HybridAstar::reconstruct(int goal_idx,
                                               const std::vector<Node3D> & pool) const
{
    std::vector<Node3D> path;
    int idx = goal_idx;
    while (idx >= 0) {
        path.push_back(pool[idx]);
        idx = pool[idx].parent_idx;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace icp_planning
