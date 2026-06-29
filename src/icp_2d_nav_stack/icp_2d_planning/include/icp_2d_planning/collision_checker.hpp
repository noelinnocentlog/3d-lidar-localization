#pragma once

#include "icp_2d_planning/types.hpp"
#include "icp_2d_planning/node3d.hpp"
#include "icp_2d_planning/motion_primitives.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <array>

namespace icp_planning {

class CollisionChecker
{
public:
    CollisionChecker() = default;

    // Load a new map (call whenever /map is received)
    void set_map(const nav_msgs::msg::OccupancyGrid::SharedPtr & map);

    bool has_map() const { return !data_.empty(); }

    // True if the pose is collision-free (fast circle check first, then full footprint)
    bool is_pose_free(double x, double y, double theta) const;

    // True if every intermediate pose along the primitive arc is collision-free
    bool is_arc_free(const Node3D & from, const Primitive & prim) const;

    // True if a straight line between two (x,y) world points is obstacle-free
    // (used by path smoother)
    bool is_segment_free(double x0, double y0, double x1, double y1) const;

    // Clearance from obstacles at (x,y) in metres — returns 0.0 if inside obstacle
    double clearance(double x, double y) const;

    int   width()      const { return width_; }
    int   height()     const { return height_; }
    double resolution() const { return res_; }
    double origin_x()   const { return ox_; }
    double origin_y()   const { return oy_; }

    // Raw occupied check for a map cell
    bool cell_occupied(int ix, int iy) const;

private:
    // World → map-cell
    bool world_to_cell(double x, double y, int & ix, int & iy) const;

    // Full oriented-rectangle footprint check
    bool footprint_free(double x, double y, double theta) const;

    std::vector<int8_t> data_;
    int    width_  {0};
    int    height_ {0};
    double res_    {0.05};
    double ox_     {0.0};
    double oy_     {0.0};

    // half-dimensions for footprint corners
    static constexpr double HL = ROBOT_LENGTH / 2.0;
    static constexpr double HW = ROBOT_WIDTH  / 2.0;

    // 4 corners in robot-local frame (L×W rectangle)
    static constexpr std::array<std::array<double,2>, 4> CORNERS = {{
        {{ HL,  HW}},
        {{ HL, -HW}},
        {{-HL, -HW}},
        {{-HL,  HW}},
    }};
};

}  // namespace icp_planning
