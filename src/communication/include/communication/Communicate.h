#ifndef __COM_COMMUNICATE_H__
#define __COM_COMMUNICATE_H__

#include "Com_master.h"
#include "Com_slave.h"

#ifdef __cplusplus 
extern "C" {
#endif 

#define ROLE_MASTER 
//#define ROLE_SLAVE 

#if defined ROLE_MASTER
#define TXBUFFERSIZE	(256)
#define RXBUFFERSIZE	(256)

// extern SlaveMessage* slaveMessage;
// extern MasterMessage* masterMessage;
#else
#define TXBUFFERSIZE	(16)
#define RXBUFFERSIZE	(256)

extern SlaveMessage* slaveMessage;
extern const MasterMessage* masterMessage;

typedef void (*Com_cmdAction)(void);

#endif

#if defined ROLE_SLAVE
void Com_recv();
void Com_send();
#endif

#ifdef __cplusplus 
}
#endif 

#if defined ROLE_MASTER

#endif

#endif