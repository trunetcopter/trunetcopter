#ifndef _comm_h
#define _comm_h

#include "ch.h"
#include "hal.h"

#include "config.h"
#include "mavlink/mavlink_types.h"

#define MAVLINK_STACK_SIZE		512
#define MAVLINK_PRIORITY		NORMALPRIO+5

#define MAVLINK_SERIAL_DEVICE		SD2
#define MAVLINK_SERIAL_BAUD		115200
#define MAVLINK_NOTICE_DEPTH		20
#define MAVLINK_NOTICE_LEN		40

#define MAVLINK_HEARTBEAT_INTERVAL	1000 // 1Hz

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

extern msg_t noticeBuf[MAVLINK_NOTICE_LEN];

typedef struct {
	SerialDriver *serialPort;

	Mailbox noticeQueue[MAVLINK_NOTICE_DEPTH];

	uint8_t mode;
	uint8_t nav_mode;
	uint8_t status;

	uint16_t packetDrops;
	uint16_t idlePercent;
	unsigned long lastCounter;

	unsigned long nextHeartbeat;
	unsigned long nextParam;
	unsigned int currentParam;
	unsigned long streamInterval[13];
	unsigned long streamNext[13];
} mavlinkStruct_t;

extern mavlinkStruct_t mavlinkData;
extern mavlink_system_t mavlink_system;

extern void mavlinkInit(void);
extern void mavlinkNotice(const char *s);
extern void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

#endif
