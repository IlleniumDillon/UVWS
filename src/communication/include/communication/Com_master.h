#ifndef __COM_MASTER_H__
#define __COM_MASTER_H__

#include <stdint.h>

///命令码

#define MASTER_CMD_PING 0
#define MASTER_CMD_STATE    1
#define MASTER_CMD_BASE 2
#define MASTER_CMD_ARM  3
#define MASTER_CMD_TRIPOD  4
#define MASTER_CMD_DUMMY    5
#define MASTER_CMD_SETALL    6


///帧格式定义
#pragma pack(push)
#pragma pack(1)
typedef struct 
{
    float timeStamp;
    uint8_t cmd;
    uint8_t dataPtr[0];
}MasterMessage;
#pragma pack(pop)

///底盘运动指令
#pragma pack(push)
#pragma pack(1)
typedef struct 
{
    float vc;
    float wc;
}BaseMotionCmd;
#pragma pack(pop)

///机械臂运动指令
#pragma pack(push)
#pragma pack(1)
typedef struct 
{
    union
    {
        uint16_t degreeArr[3];
        struct 
        {
            uint16_t base;
            uint16_t arm;
            uint16_t hand;
        }degrees;
    };
    uint16_t time_ms;
}ArmMotionCmd;
#pragma pack(pop)

///全部设置指令
#pragma pack(push)
#pragma pack(1)
typedef struct
{
	uint8_t state;
	BaseMotionCmd base;
	ArmMotionCmd arm;
	float tripod;
}SetAllCmd;
#pragma pack(pop)

///命令属性表
static const uint8_t masterCmdAttribute[][2] = 
{
//      命令码                 长度   响应
    [MASTER_CMD_PING]   =   {   0  ,   1    },
    [MASTER_CMD_STATE]  =   {   1  ,   1    },
    [MASTER_CMD_BASE]   =   {   8  ,   1    },
    [MASTER_CMD_ARM]    =   {   8  ,   1    },
	[MASTER_CMD_TRIPOD] =   {   4  ,   1    },
    [MASTER_CMD_DUMMY]  =   {   0  ,   0    },
    [MASTER_CMD_SETALL]  =  {   21 ,   1    },
};

#endif
