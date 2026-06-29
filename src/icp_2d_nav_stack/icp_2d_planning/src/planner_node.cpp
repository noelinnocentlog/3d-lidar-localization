#include "icp_2d_planning/hybrid_astar.hpp"
#include "icp_2d_planning/path_smoother.hpp"
#include "icp_2d_planning/config.hpp"
#include "icp_2d_planning/types.hpp"
#include "icp_2d_planning/node3d.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <memory>
#include <string>

namespace icp_planning {

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode()
    : Node("hybrid_astar_planner"),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_)
    {
        declare_parameters();
        configure_components();

        // Re-configure whenever any parameter is updated at runtime
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &) {
                configure_components();
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                return r;
            });

        const std::string map_topic  = get_parameter("map_topic").as_string();
        const std::string goal_topic = get_parameter("goal_topic").as_string();
        const std::string plan_topic = get_parameter("plan_topic").as_string();
        const std::string dir_topic  = get_parameter("plan_directions_topic").as_string();
        const std::string dij_topic  = get_parameter("dijkstra_map_topic").as_string();

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, rclcpp::QoS(1).transient_local(),
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                planner_.set_map(msg);
                RCLCPP_INFO(get_logger(), "Map received: %d x %d cells @ %.3fm",
                    msg->info.width, msg->info.height, msg->info.resolution);
            });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic, 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                on_goal(msg);
            });

        plan_pub_     = create_publisher<nav_msgs::msg::Path>(plan_topic,
                            rclcpp::QoS(1).transient_local());
        dir_pub_      = create_publisher<std_msgs::msg::Int8MultiArray>(dir_topic,
                            rclcpp::QoS(1).transient_local());
        dijkstra_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(dij_topic,
                            rclcpp::QoS(1).transient_local());

        RCLCPP_INFO(get_logger(), "Hybrid A* planner ready. "
            "Waiting for map on '%s' and goals on '%s'.",
            map_topic.c_str(), goal_topic.c_str());
    }

private:
    // ── Declare all ROS parameters ────────────────────────────────────────────
    void declare_parameters()
    {
        // Topic / frame names
        declare_parameter("map_topic",             std::string("/P1_robot0/map"));
        declare_parameter("goal_topic",            std::string("/P1_robot0/goal_pose"));
        declare_parameter("plan_topic",            std::string("/P1_robot0/hybrid_astar/plan"));
        declare_parameter("plan_directions_topic", std::string("/P1_robot0/hybrid_astar/plan_directions"));
        declare_parameter("dijkstra_map_topic",    std::string("/P1_robot0/hybrid_astar/dijkstra_map"));
        declare_parameter("base_frame",            std::string("P1_robot0/base_link"));
        declare_parameter("map_frame",             std::string("map"));

        // Cost weights
        declare_parameter("reverse_penalty",          REVERSE_PENALTY);
        declare_parameter("direction_change_penalty", DIRECTION_CHANGE_PENALTY);
        declare_parameter("steer_change_penalty",     STEER_CHANGE_PENALTY);
        declare_parameter("heuristic_weight",         HEURISTIC_WEIGHT);

        // Search limits
        declare_parameter("max_iterations",   MAX_ITERATIONS);
        declare_parameter("planning_timeout", PLANNING_TIMEOUT);

        // Analytic expansion
        declare_parameter("analytic_expand_n",  ANALYTIC_EXPAND_N);
        declare_parameter("analytic_max_len",   ANALYTIC_MAX_LEN);

        // Smoother
        declare_parameter("shortcut_max_dist",  1.5);
        declare_parameter("smoother_max_iter",  SMOOTHER_MAX_ITER);
        declare_parameter("smoother_lr",        0.05);
        declare_parameter("smoother_w_smooth",  SMOOTHER_W_SMOOTH);
        declare_parameter("smoother_w_data",    SMOOTHER_W_DATA);
        declare_parameter("smoother_w_obs",     SMOOTHER_W_OBS);
        declare_parameter("smoother_min_clear", 0.60);

        // Goal tolerance
        declare_parameter("goal_xy_tol",    0.15);
        declare_parameter("goal_theta_tol", 0.20);

        // Robot footprint
        declare_parameter("robot_length",    ROBOT_LENGTH);
        declare_parameter("robot_width",     ROBOT_WIDTH);
        declare_parameter("bounding_radius", BOUNDING_RADIUS);
        declare_parameter("min_turn_radius", 0.20);   // v_min/ω_max = 0.3/1.5
    }

    // ── Build PlannerConfig from live params and push to components ───────────
    void configure_components()
    {
        PlannerConfig cfg;
        cfg.reverse_penalty          = get_parameter("reverse_penalty").as_double();
        cfg.direction_change_penalty = get_parameter("direction_change_penalty").as_double();
        cfg.steer_change_penalty     = get_parameter("steer_change_penalty").as_double();
        cfg.heuristic_weight         = get_parameter("heuristic_weight").as_double();
        cfg.max_iterations           = get_parameter("max_iterations").as_int();
        cfg.planning_timeout         = get_parameter("planning_timeout").as_double();
        cfg.analytic_expand_n        = get_parameter("analytic_expand_n").as_int();
        cfg.analytic_max_len         = get_parameter("analytic_max_len").as_double();
        cfg.shortcut_max_dist        = get_parameter("shortcut_max_dist").as_double();
        cfg.smoother_max_iter        = get_parameter("smoother_max_iter").as_int();
        cfg.smoother_lr              = get_parameter("smoother_lr").as_double();
        cfg.smoother_w_smooth        = get_parameter("smoother_w_smooth").as_double();
        cfg.smoother_w_data          = get_parameter("smoother_w_data").as_double();
        cfg.smoother_w_obs           = get_parameter("smoother_w_obs").as_double();
        cfg.smoother_min_clear       = get_parameter("smoother_min_clear").as_double();
        cfg.goal_xy_tol              = get_parameter("goal_xy_tol").as_double();
        cfg.goal_theta_tol           = get_parameter("goal_theta_tol").as_double();
        cfg.robot_length             = get_parameter("robot_length").as_double();
        cfg.robot_width              = get_parameter("robot_width").as_double();
        cfg.bounding_radius          = get_parameter("bounding_radius").as_double();
        cfg.min_turn_radius          = get_parameter("min_turn_radius").as_double();

        planner_.configure(cfg);
        smoother_.configure(cfg);
    }

    // ── TF-based robot pose lookup ────────────────────────────────────────────
    bool get_robot_pose(Node3D & out) const
    {
        const std::string base  = get_parameter("base_frame").as_string();
        const std::string mframe = get_parameter("map_frame").as_string();
        try {
            auto tf = tf_buffer_.lookupTransform(
                mframe, base, tf2::TimePointZero, tf2::durationFromSec(0.1));
            out.x = tf.transform.translation.x;
            out.y = tf.transform.translation.y;
            const auto & q = tf.transform.rotation;
            out.theta = normalise_angle(std::atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
            return true;
        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN(get_logger(),
                "Cannot get robot pose (%s→%s): %s\n"
                "  → Click '2D Pose Estimate' in RViz to initialise AMCL.",
                mframe.c_str(), base.c_str(), e.what());
            return false;
        }
    }

    void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr & goal_msg)
    {
        if (!planner_.has_map()) {
            RCLCPP_WARN(get_logger(), "No map yet — ignoring goal.");
            return;
        }

        Node3D start;
        if (!get_robot_pose(start)) return;

        Node3D goal = pose_to_node(goal_msg->pose);

        RCLCPP_INFO(get_logger(),
            "Planning: start (%.2f, %.2f, %.1f°) → goal (%.2f, %.2f, %.1f°)",
            start.x, start.y, start.theta * 180.0 / M_PI,
            goal.x,  goal.y,  goal.theta  * 180.0 / M_PI);

        auto   plan_start = now();
        auto   result     = planner_.plan(start, goal);
        double elapsed    = (now() - plan_start).seconds();

        if (!result.success) {
            RCLCPP_WARN(get_logger(),
                "Planning failed after %.2fs. "
                "Try raising heuristic_weight or planning_timeout.", elapsed);
            return;
        }

        RCLCPP_INFO(get_logger(),
            "Plan found: %zu waypoints, cost=%.2f, time=%.3fs",
            result.path.size(), result.cost, elapsed);

        smoother_.smooth(result.path, planner_.collision_checker());
        RCLCPP_INFO(get_logger(), "Smoothed to %zu waypoints.", result.path.size());

        // Publish with the time the plan was generated, not the goal arrival time
        publish_plan(result.path, now());
        publish_dijkstra_map();
    }

    // ── Publish plan + per-waypoint direction array ───────────────────────────
    void publish_plan(const std::vector<Node3D> & path,
                      const rclcpp::Time & stamp)
    {
        const std::string mframe = get_parameter("map_frame").as_string();

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp    = stamp;
        path_msg.header.frame_id = mframe;

        std_msgs::msg::Int8MultiArray dir_msg;

        for (const auto & node : path) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = node.x;
            ps.pose.position.y = node.y;
            ps.pose.position.z = 0.0;

            // Convention: forward → yaw = heading; reverse → yaw flipped 180°
            double yaw = node.theta;
            if (node.direction == Direction::REVERSE)
                yaw = normalise_angle(yaw + M_PI);

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            ps.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(ps);

            dir_msg.data.push_back(
                static_cast<int8_t>(node.direction == Direction::FORWARD ? 1 : -1));
        }

        plan_pub_->publish(path_msg);
        dir_pub_->publish(dir_msg);
    }

    // ── Publish Dijkstra heuristic map for RViz ───────────────────────────────
    void publish_dijkstra_map()
    {
        const Heuristic & h = planner_.heuristic();
        if (!h.is_ready()) return;

        const std::string mframe = get_parameter("map_frame").as_string();
        const auto & dmap = h.dijkstra_map();

        float max_cost = 0.0f;
        for (float v : dmap)
            if (std::isfinite(v)) max_cost = std::max(max_cost, v);

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp    = now();
        grid.header.frame_id = mframe;
        grid.info.width      = static_cast<uint32_t>(h.map_width());
        grid.info.height     = static_cast<uint32_t>(h.map_height());
        grid.info.resolution = static_cast<float>(h.map_res());
        grid.info.origin.position.x = h.map_ox();
        grid.info.origin.position.y = h.map_oy();

        grid.data.resize(dmap.size());
        for (size_t i = 0; i < dmap.size(); ++i) {
            grid.data[i] = !std::isfinite(dmap[i]) ? int8_t(100) :
                static_cast<int8_t>(
                    std::clamp(static_cast<int>(dmap[i] / max_cost * 99.0f), 0, 99));
        }
        dijkstra_pub_->publish(grid);
    }

    static Node3D pose_to_node(const geometry_msgs::msg::Pose & pose)
    {
        Node3D node;
        node.x = pose.position.x;
        node.y = pose.position.y;
        const auto & q = pose.orientation;
        node.theta = normalise_angle(std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        return node;
    }

    HybridAstar  planner_;
    PathSmoother smoother_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr   map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr            plan_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr  dir_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr   dijkstra_pub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace icp_planning

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<icp_planning::PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
