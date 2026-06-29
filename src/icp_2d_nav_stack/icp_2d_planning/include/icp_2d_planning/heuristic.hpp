#pragma once

#include "icp_2d_planning/types.hpp"
#include "icp_2d_planning/node3d.hpp"
#include "icp_2d_planning/collision_checker.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

namespace icp_planning {

class Heuristic
{
public:
    Heuristic() = default;

    // Precompute 2D Dijkstra map backward from goal cell.
    // Must be called every time the goal changes.
    void precompute(const Node3D & goal, const CollisionChecker & cc);

    // H1: obstacle-aware 2D cost (from Dijkstra map)
    double h1(double x, double y) const;

    // H2: non-holonomic-without-obstacles (Reeds-Shepp lower bound)
    double h2(const Node3D & node, const Node3D & goal) const;

    // Combined heuristic: max(H1, H2)
    double compute(const Node3D & node, const Node3D & goal) const;

    bool is_ready() const { return !dijkstra_.empty(); }

    // Expose for debug visualisation
    const std::vector<float> & dijkstra_map() const { return dijkstra_; }
    int   map_width()  const { return width_; }
    int   map_height() const { return height_; }
    double map_res()    const { return res_; }
    double map_ox()     const { return ox_; }
    double map_oy()     const { return oy_; }

private:
    std::vector<float> dijkstra_;
    int    width_  {0};
    int    height_ {0};
    double res_    {0.05};
    double ox_     {0.0};
    double oy_     {0.0};

    bool world_to_cell(double x, double y, int & ix, int & iy) const;
};

}  // namespace icp_planning
