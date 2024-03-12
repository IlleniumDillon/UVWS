#include "Communication.hpp"

Communication::Communication()
{
    command.degree = 0;
    command.vc = 0;
    command.wc = 0;
    command.arm = 3500;
    command.base = 500;
    command.hand = 2048;
    command.time = 0;
    serial_port = "";
}
int Communication::serialInit()
{
    RCLCPP_INFO(rclcpp::get_logger("serial:"),"connect slave");
    try 
    { 
        ser.setPort(serial_port); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        RCLCPP_INFO(rclcpp::get_logger("serial:"),"Unable to open port "); 
        return -1; 
    } 
    if(ser.isOpen()) 
    { 
        RCLCPP_INFO(rclcpp::get_logger("serial:"),"Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
    return 0;
}
void Communication::getData(const UvCommand::SharedPtr msg)
{
    status.tripod = msg->degree;

    command.degree = msg->degree;
    command.vc = msg->vc;
    command.wc = msg->wc;
    command.arm = msg->arm;
    command.base = msg->base;
    command.hand = msg->hand;
}
int Communication::serialRead()
{
    if(!ser.available()) return 0;
    //COM_INFO("%d,%d",sizeof(SlaveMessage),ser.available());
    ser.read(recv_data,ser.available()); 
    status.voltage = cvtUFix8Float(slaveMessage->power);
    status.wheelspeed_left = cvtFix16Float(slaveMessage->speed_l);
    status.wheelspeed_right = cvtFix16Float(slaveMessage->speed_r);
    status.acc_x = slaveMessage->icm_acc_x;
    status.acc_y = slaveMessage->icm_acc_y;
    status.acc_z = slaveMessage->icm_acc_z;
    status.roll = slaveMessage->roll;
    status.pitch = slaveMessage->pitch;
    status.yaw = slaveMessage->yaw;
    return 1;       
 
}
void Communication::serialSetData()
{

    uint8_t cmd = MASTER_CMD_SETALL;
    SetAllCmd* pdata = (SetAllCmd*)masterMessage->dataPtr;

    masterMessage->timeStamp = 0;
    masterMessage->cmd = cmd;
    pdata->state = 0;
    pdata->base.vc = command.vc;
    pdata->base.wc = command.wc;
    pdata->arm.degrees.arm = command.arm;
    pdata->arm.degrees.base = command.base;
    pdata->arm.degrees.hand = command.hand;
    pdata->arm.time_ms = command.time;
    pdata->tripod = command.degree;

    recvFlag = 0;
    ser.write(send_data,masterCmdAttribute[cmd][0] + 5);
}