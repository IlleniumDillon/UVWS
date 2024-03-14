#include "rclcpp/rclcpp.hpp"
#include "uvinterfaces/msg/uv_map.hpp"
#include "uvinterfaces/srv/uv_pathplan.hpp"
using namespace std::chrono_literals;
class test   :   public rclcpp::Node
{
public:
    test();
    void timer_callback();
    bool ok = false;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<uvinterfaces::msg::UvMap>::SharedPtr pubMap;
    rclcpp::Client<uvinterfaces::srv::UvPathplan>::SharedPtr cliPath;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto nh = std::make_shared<test>();
    uvinterfaces::msg::UvMap map;
    map.maplength=4;
    map.mapwidth=6;
    map.mapheight=1;
    map.scaleheight=1;
    map.scalelength=1;
    map.scalewidth=1;
    uint8_t dmap[] = {
        0,0,0,0,0,0,
        0,1,1,0,0,0,
        0,1,0,0,0,0,
        0,0,0,0,0,0,
    };
    for(int i = 0; i < 24; i++)
    {
        map.mapdata.push_back(dmap[i]);
    }

    nh->pubMap->publish(map);
    nh->ok = true;

    auto req = std::make_shared<uvinterfaces::srv::UvPathplan::Request>();
    req->start.x = 0;
    req->start.y = 0;
    req->start.z = 0;
    req->target.x = 3;
    req->target.y = 5;
    req->target.z = 0;
    auto result = nh->cliPath->async_send_request(req);

    if (rclcpp::spin_until_future_complete(nh, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        for(int i = 0; i < result.get()->path.size(); i++)
        {
            RCLCPP_INFO(nh->get_logger(),"[%f,%f]",result.get()->path.at(i).x,result.get()->path.at(i).y);
        }
    }
    // while(rclcpp::ok())
    // {
    //     rclcpp::spin_some(nh);
    //     //nh->cliPath->create_request_header
    // }
    rclcpp::shutdown();
    return 0;
}

test::test() : Node("test_node")
{
    pubMap = this->create_publisher<uvinterfaces::msg::UvMap>("map",1);
    cliPath = this->create_client<uvinterfaces::srv::UvPathplan>("path");
    //timer = this->create_wall_timer(2s, std::bind(&test::timer_callback, this));
}

void test::timer_callback()
{
    if(ok)
    {
        auto req = std::make_shared<uvinterfaces::srv::UvPathplan::Request>();
        req->start.x = 0;
        req->start.y = 0;
        req->start.z = 0;
        req->target.x = 3;
        req->target.y = 5;
        req->target.z = 0;
        cliPath->async_send_request(req);
    }
}