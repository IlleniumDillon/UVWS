#include "UvPlanner.hpp"
using namespace Eigen;
UvPlanner::UvPlanner():Node("uvplanner_node")
{
    /*uint8_t map[] = {
        0,0,0,0,0,0,
        0,1,1,0,0,0,
        0,1,0,0,0,0,
        0,0,0,0,0,0,
    };
    solver.setMap(map,4,6);
    solver.setScale(1,1);
    solver.reset();
    Vector2d start(0,0);
    Vector2d goal(3,5);
    solver.solve(start,goal);
    for(int i = solver.path.size()-1; i >=0; i--)
    {
        RCLCPP_INFO(this->get_logger(),"[%f,%f]",solver.path.at(i).x(),solver.path.at(i).y());
    }*/
    subMap = this->create_subscription<uvinterfaces::msg::UvMap>("map",1,
        std::bind(subMapCallback,std::placeholders::_1));
    srvPath = this->create_service<uvinterfaces::srv::UvPathplan>("path",
        std::bind(srvSolveCallback,std::placeholders::_1,std::placeholders::_2));
}

void UvPlanner::subMapCallback(const uvinterfaces::msg::UvMap::SharedPtr msg)
{
    if(inProgress)
    {
        RCLCPP_ERROR(this->get_logger(),"cannot update map.");
        return;
    }
    solver.setMap(msg->mapdata.data(),msg->maplength,msg->mapwidth,msg->mapheight);
    solver.setScale(msg->scalelength,msg->scalewidth,msg->scaleheight);
    readyPlan = true;
}

void UvPlanner::srvSolveCallback(const uvinterfaces::srv::UvPathplan::Request::SharedPtr req, const uvinterfaces::srv::UvPathplan::Response::SharedPtr res)
{
    if(!readyPlan)
    {
        RCLCPP_ERROR(this->get_logger(),"cannot plan without map.");
        return;
    }
    inProgress = true;
    solver.reset();
    Vector3d start(req->start.x,req->start.y,req->start.z);
    Vector3d goal(req->target.x,req->target.y,req->target.z);
    if(solver.solve(start,goal))
    {
        res->path.clear();
        for(int i = solver.path.size()-1; i >=0; i--)
        {
            geometry_msgs::msg::Point temp;
            temp.x = solver.path.at(i).x();
            temp.y = solver.path.at(i).y();
            temp.z = solver.path.at(i).z();
            res->path.push_back(temp);
        }
    }
    else
    {
        res->path.clear();
        RCLCPP_ERROR(this->get_logger(),"no valid plan.");
    }
    inProgress = false;
}
