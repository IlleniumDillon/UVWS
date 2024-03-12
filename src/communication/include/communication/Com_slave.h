#ifndef __COM_SLAVE_H__
#define __COM_SLAVE_H__

#include <stdint.h>
#include "Com_type.h"

typedef enum : uint8_t
{
    ERR_OK = (uint8_t)0,
    ERR_BUSY,
    ERR_FAILURE,
}SlaveErrCode;

///֡��ʽ����
#pragma pack(push)
#pragma pack(1)
typedef struct 
{
    uint8_t state;
    uint8_t lastCmd;
    SlaveErrCode err;
    ufix8_t power;
    fix16_t speed_l;
    fix16_t speed_r;
    float icm_gyro_x,icm_gyro_y,icm_gyro_z;
    float icm_acc_x,icm_acc_y,icm_acc_z;
    float roll, pitch, yaw;
}SlaveMessage;
#pragma pack(pop)

#endif
