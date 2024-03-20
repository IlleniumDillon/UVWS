#include "SamplePlanner.hpp"
using namespace Eigen;
UvPlanner::UvPlanner():Node("uvplanner_node")
{

    lastGoal = Vector3d(0,0,0);
    subnMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",1,
        std::bind(&UvPlanner::subMapCallback,this,std::placeholders::_1));
    /*srvPath = this->create_service<uvinterfaces::srv::UvPathplan>("path",
        std::bind(&UvPlanner::srvSolveCallback,this,std::placeholders::_1,std::placeholders::_2));*/
    subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose",1,
        std::bind(&UvPlanner::subGoalCallback,this,std::placeholders::_1));
    pubPath = this->create_publisher<nav_msgs::msg::Path>("/path",1);
    readyPlan = true;
}



void UvPlanner::subMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if(inProgress)
    {
        RCLCPP_ERROR(this->get_logger(),"cannot update map.");
        return;
    }
    
    uint8_t* dmap = new uint8_t[msg->info.height*msg->info.width];
    RCLCPP_INFO(this->get_logger(),"%d,%d",msg->info.height,msg->info.width);
    for(int i = 0; i < msg->info.height; i++)
    {
        for(int j = 0; j < msg->info.width; j++)
        {
            //RCLCPP_INFO(this->get_logger(),"%d",msg->data.at(i*msg->info.width+j));
            if(msg->data.at(i*msg->info.width+j) == -1||
                msg->data.at(i*msg->info.width+j) == 100)
            {
                dmap[j*msg->info.height+i]=1;
            }
            else
            {
                dmap[j*msg->info.height+i]=0;
            }
            
        }
    }
    solver.setMap(dmap,msg->info.width,msg->info.height);
    solver.setScale(msg->info.resolution,msg->info.resolution);
    solver.setOri(msg->info.origin.position.x,
        msg->info.origin.position.y,
        msg->info.origin.position.z);
    readyPlan = true;
}
void UvPlanner::subGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!readyPlan)
    {
        RCLCPP_ERROR(this->get_logger(),"cannot plan without map.");
        return;
    }
    inProgress = true;
    solver.reset();
    nav_msgs::msg::Path req;
    req.header.frame_id="/map";
    req.header.stamp = this->now();
    Vector3d goal(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(),"[%f,%f,%f]->[%f,%f,%f]",lastGoal.x(),lastGoal.y(),lastGoal.z(),goal.x(),goal.y(),goal.z());
    if(solver.solve(lastGoal,goal))
    {
        req.poses.clear();
        for(int i = solver.path.size()-1; i >=0; i--)
        {
            geometry_msgs::msg::PoseStamped temp;
            temp.pose.position.x = solver.path.at(i).x();
            temp.pose.position.y = solver.path.at(i).y();
            temp.pose.position.z = solver.path.at(i).z();
            req.poses.push_back(temp);
        }
    }
    else
    {
        req.poses.clear();
        RCLCPP_ERROR(this->get_logger(),"no valid plan.");
    }
    lastGoal = goal;
    pubPath->publish(req);
    inProgress = false;
}

