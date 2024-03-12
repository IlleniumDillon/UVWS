#include "rclcpp/rclcpp.hpp"
#include "uvinterfaces/msg/uv_map.hpp"
#include "uvinterfaces/srv/uv_pathplan.hpp"

class test   :   public rclcpp::Node
{
public:
    test();
    rclcpp::Subscription<uvinterfaces::msg::UvMap>::SharedPtr subMap;
    rclcpp::Service<uvinterfaces::srv::UvPathplan>::SharedPtr srvPath;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<test>());
    rclcpp::shutdown();
    return 0;
}

test::test() : Node("test_node")
{
}
