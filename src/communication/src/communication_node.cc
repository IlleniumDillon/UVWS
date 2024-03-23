
#include "rclcpp/rclcpp.hpp"
#include "uvinterfaces/msg/uv_status.hpp"
#include "uvinterfaces/msg/uv_command.hpp"

#include <serial/serial.h> 
#include <thread>

#include "Communicate.h"
#include "Communication.hpp"

using uvinterfaces::msg::UvStatus;
using uvinterfaces::msg::UvCommand;

using namespace std::chrono_literals;

class CommunicationNode: public rclcpp::Node
{
public:
    CommunicationNode();
    void setAllCallback(const UvCommand::SharedPtr msg);
    void timeCallBack();
    void runSerialRead();
    std::thread *readThreadPtr;

private:
    rclcpp::Subscription<UvCommand>::SharedPtr Subscription_;
    rclcpp::Publisher<UvStatus>::SharedPtr pub_status;
    rclcpp::TimerBase::SharedPtr Timer_;
    Communication communication;
    std::string name;
};

void CommunicationNode::setAllCallback(const UvCommand::SharedPtr msg)
{
    communication.getData(msg);
}
void CommunicationNode::timeCallBack()
{
    communication.serialSetData();
    pub_status->publish(communication.status);
    // COM_INFO("yaw:%f",communication.status.yaw);

}
void CommunicationNode::runSerialRead()
{
    while(1)
    {
        if(!communication.serialRead()) continue;
        communication.recvFlag = 1;
        // pub_status->publish(msg);
        // COM_INFO("power:%f",msg.power); 
    }
    
}
CommunicationNode::CommunicationNode():Node("communication_node")
{
    this->declare_parameter("serial_port", "world");
    this->declare_parameter("name", "uv");
    communication.serial_port = this->get_parameter("serial_port").as_string();
    name = this->get_parameter("name").as_string();
    
    Subscription_ = this->create_subscription<UvCommand>(name + "/hostcommand",1,std::bind(&CommunicationNode::setAllCallback,this,std::placeholders::_1));
    Timer_ = this->create_wall_timer(100ms,std::bind(&CommunicationNode::timeCallBack,this));
    pub_status = this->create_publisher<UvStatus>(name + "/status",10);

    communication.serialInit();
    readThreadPtr = new std::thread(std::bind(&CommunicationNode::runSerialRead,this));

    rclcpp::WallRate loop_rate(1.0);
    while(communication.recvFlag!=1)
    {
        RCLCPP_INFO(rclcpp::get_logger("serial:"),"connecting slave device...");
        communication.ping();
        loop_rate.sleep();
        //timerCallback(ros::TimerEvent());
    }
    communication.recvFlag = 0;
    RCLCPP_INFO(rclcpp::get_logger("serial:"),"slave device up");
}
//
int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CommunicationNode>());
    rclcpp::shutdown();
    return 0;
}
