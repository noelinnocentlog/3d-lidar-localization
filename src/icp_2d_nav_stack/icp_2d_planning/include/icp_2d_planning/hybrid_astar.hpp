#pragma once

#include "icp_2d_planning/types.hpp"
#include "icp_2d_planning/config.hpp"
#include "icp_2d_planning/node3d.hpp"
#include "icp_2d_planning/collision_checker.hpp"
#include "icp_2d_planning/heuristic.hpp"

#include <vector>
#include <unordered_map>
#include <queue>

namespace icp_planning {

struct PlanResult
{
    bool success {false};
    std::vector<Node3D> path;
    double cost {INF};
};

class HybridAstar
{
public:
    HybridAstar() = default;

    void configure(const PlannerConfig & cfg) { cfg_ = cfg; }

    void set_map(const nav_msgs::msg::OccupancyGrid::SharedPtr & map);

    PlanResult plan(const Node3D & start, const Node3D & goal);

    bool has_map() const { return cc_.has_map(); }

    const Heuristic        & heuristic()        const { return heuristic_; }
    const CollisionChecker & collision_checker() const { return cc_; }

private:
    double transition_cost(const Node3D & parent, const Node3D & child,
                            const Primitive & prim) const;

    // node taken by value: pool.push_back inside would invalidate a reference
    bool try_analytic_expansion(Node3D node, const Node3D & goal,
                                 int node_idx,
                                 std::vector<Node3D> & pool,
                                 PlanResult & result);

    std::vector<Node3D> reconstruct(int goal_idx,
                                     const std::vector<Node3D> & pool) const;

    CollisionChecker cc_;
    Heuristic        heuristic_;
    PlannerConfig    cfg_;
};

}  // namespace icp_planning
