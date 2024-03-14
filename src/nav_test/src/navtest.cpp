#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class test : public rclcpp::Node
{
public:
    test():Node("test")
    {
        sub = create_subscription<nav_msgs::msg::Path>("wdnmd",1,
        std::bind(&test::callback,this,std::placeholders::_1));
    };
    void callback(const nav_msgs::msg::Path::SharedPtr msg){};
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<test>());
    //auto p = std::make_shared<UvPlanner>();
    rclcpp::shutdown();
    return 0;
}