#pragma once

#include "icp_2d_planning/config.hpp"
#include "icp_2d_planning/node3d.hpp"
#include "icp_2d_planning/collision_checker.hpp"

#include <vector>

namespace icp_planning {

class PathSmoother
{
public:
    PathSmoother() = default;

    void configure(const PlannerConfig & cfg) { cfg_ = cfg; }

    void smooth(std::vector<Node3D> & path, const CollisionChecker & cc) const;

private:
    void shortcut(std::vector<Node3D> & path, const CollisionChecker & cc) const;
    void gradient_descent(std::vector<Node3D> & path, const CollisionChecker & cc) const;

    PlannerConfig cfg_;
};

}  // namespace icp_planning
