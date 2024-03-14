#pragma once

#include "SearchBased.hpp"
#include "rclcpp/rclcpp.hpp"

//#include "uvinterfaces/srv/uv_pathplan.hpp"
//#include "uvinterfaces/msg/uv_map.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace Eigen;
class UvPlanner :   public rclcpp::Node
{
public:
    UvPlanner();
    //void subMapCallback(const uvinterfaces::msg::UvMap::SharedPtr msg);
    /*void srvSolveCallback(const uvinterfaces::srv::UvPathplan::Request::SharedPtr req,
                          const uvinterfaces::srv::UvPathplan::Response::SharedPtr res);*/
    void subGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void subMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
public:
    UV::AStar solver;
    Vector3d lastGoal;
    ///TODO:
    ///sub for map
    //rclcpp::Subscription<uvinterfaces::msg::UvMap>::SharedPtr subMap;
    ///srv for plan
    //rclcpp::Service<uvinterfaces::srv::UvPathplan>::SharedPtr srvPath;
    ///
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subnMap;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    bool readyPlan = false;
    bool inProgress = false;
};
