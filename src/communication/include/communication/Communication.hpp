#pragma once

#include "rclcpp/rclcpp.hpp"

#include <serial/serial.h> 
#include "Communicate.h"
#include "uvinterfaces/msg/uv_command.hpp"
#include "uvinterfaces/msg/uv_status.hpp"

#define COM_INFO(expr,...) RCLCPP_INFO(rclcpp::get_logger("serial:"),expr,__VA_ARGS__)

using namespace std;
using uvinterfaces::msg::UvStatus;
using uvinterfaces::msg::UvCommand;


class Communication
{
public:
    UvCommand command;
    UvStatus status;
    string serial_port;
    int baudrate;
    uint8_t recvFlag = 0;

    serial::Serial ser;  
    unsigned char send_data[TXBUFFERSIZE];
    unsigned char recv_data[RXBUFFERSIZE];

    SlaveMessage* slaveMessage = (SlaveMessage*)recv_data;
    MasterMessage* masterMessage = (MasterMessage*)send_data;
    void ping()
    {
        // auto timeStamp = rclcpp::Clock().now();
        uint8_t cmd = MASTER_CMD_PING;
        recvFlag = 0;
        ser.write(send_data,masterCmdAttribute[cmd][0] + 5);
        //while(recvFlag!=1);
    }
    Communication();
    int serialInit();
    void getData(const UvCommand::SharedPtr msg);
    void serialSetData();
    int serialRead();
};
