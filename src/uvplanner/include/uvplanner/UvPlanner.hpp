#pragma once

#include "SearchBased.hpp"
#include "rclcpp/rclcpp.hpp"

#include "uvinterfaces/srv/uv_pathplan.hpp"
#include "uvinterfaces/msg/uv_map.hpp"

class UvPlanner :   public rclcpp::Node
{
public:
    UvPlanner();
    void subMapCallback(const uvinterfaces::msg::UvMap::SharedPtr msg);
    void srvSolveCallback(const uvinterfaces::srv::UvPathplan::Request::SharedPtr req,
                          const uvinterfaces::srv::UvPathplan::Response::SharedPtr res);
public:
    UV::AStar solver;
    ///TODO:
    ///sub for map
    rclcpp::Subscription<uvinterfaces::msg::UvMap>::SharedPtr subMap;
    ///srv for plan
    rclcpp::Service<uvinterfaces::srv::UvPathplan>::SharedPtr srvPath;
    bool readyPlan = false;
    bool inProgress = false;
};
