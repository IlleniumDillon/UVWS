#include "rclcpp/rclcpp.hpp"
#include "uvinterfaces/msg/uv_map.hpp"
#include "uvinterfaces/srv/uv_pathplan.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    
    rclcpp::shutdown();
    return 0;
}