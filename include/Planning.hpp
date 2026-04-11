#ifndef PLANNING_HPP
#define PLANNING_HPP

#include <vector>
#include <algorithm>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include <nav_msgs/msg/odometry.hpp>

// A-star cell structure
struct Cell {
    int x, y;
    float f, g, h;
    std::shared_ptr<Cell> parent;

    Cell(int c, int r);
};

class PlanningNode : public rclcpp::Node {
public:
    PlanningNode();

private:
    // Parameters
    double current_robot_x_ = 0.0;
    double current_robot_y_ = 0.0;
    bool has_robot_pose_ = false; // Pojistka, abychom neplánovali dřív, než známe polohu

    // Callbacks
    void mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future);
    void planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);

    // Clients
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;

    // Services
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_service_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Methods
    void dilateMap();
    void aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal);
    void smoothPath();

    // Data
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Path path_;
};

#endif // PLANNING_HPP
