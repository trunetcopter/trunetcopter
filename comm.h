/*
This file is part of Trunetcopter.

Trunetcopter is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Trunetcopter is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Trunetcopter.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _comm_h
#define _comm_h

#include "ch.h"
#include "hal.h"

#include "config.h"

#include "mavlink_types.h"

/*
extern Mailbox mbNotice;
extern Mailbox mbImu;
extern Mailbox mbMagn;
extern Mailbox mbGps;

extern mavlink_statustext_t mavlink_statustext_struct;
*/

/**
 * Structure for data exchange with confimation capability.
 */
typedef struct Mail Mail;
struct Mail{
  /**
   * @brief   pointer to external buffer.
   * @details When receiver got data it must be set this pointer to NULL
   *          as a ready flag.
   */
  void *payload;
  /**
   * Content is on program responsibility. Can be contain anything.
   */
  msg_t invoice;
  /**
   * Protection semaphore.
   * Set to NULL if unused.
   */
  BinarySemaphore *semp;
};

void ReleaseMail(Mail* mailp);
void MsgInit(void);

#define MAVLINK_STACK_SIZE		512
#define MAVLINK_PRIORITY		NORMALPRIO

#define MAVLINK_SERIAL_DEVICE		SD2
#define MAVLINK_SERIAL_BAUD		115200

#define MAVLINK_HEARTBEAT_INTERVAL	1000 // 1Hz

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

typedef struct {
	SerialDriver *serialPort;

	uint8_t mode;
	uint8_t nav_mode;
	uint8_t status;

	uint16_t packetDrops;
	uint16_t idlePercent;

	unsigned long nextHeartbeat;
	unsigned long nextParam;
	unsigned int currentParam;
	unsigned long streamInterval[13];
	unsigned long streamNext[13];
} mavlinkStruct_t;

extern mavlinkStruct_t mavlinkData;
extern mavlink_system_t mavlink_system;

extern void mavlinkInit(void);
extern msg_t mavlinkNotice(uint8_t severity, const char *text);
extern void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

#endif
