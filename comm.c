#include "comm.h"

#include "mavlink.h"

#include "util.h"
#include "radio.h"

msg_t noticeBuf[MAVLINK_NOTICE_LEN];

mavlinkStruct_t mavlinkData;
mavlink_system_t mavlink_system;

void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
	if (chan == MAVLINK_COMM_0) {
		sdWrite(mavlinkData.serialPort, &ch, 1);
	}
}

void mavlinkNotice(const char *s) {
	// queue message, notify and leave
	chMBPost(mavlinkData.noticeQueue, (msg_t)&s, TIME_IMMEDIATE);
}

static WORKING_AREA(waThreadMavlink, MAVLINK_STACK_SIZE);
static msg_t ThreadMavlink(void *arg) {
        (void)arg;
        chRegSetThreadName("mavlink");

	static unsigned long mavCounter;
	static unsigned long lastMillis = 0;
	unsigned long millis;
	mavlinkNotice("MavLink Initialized!");

        while (TRUE) {
		millis = chTimeNow();

		// handle rollover
		if (millis < lastMillis) {
			mavlinkData.nextHeartbeat = 0;
			mavlinkData.nextParam = 0;
			memset(mavlinkData.streamNext, 0, sizeof(mavlinkData.streamInterval));
		}

		// heartbeat
		if (mavlinkData.nextHeartbeat < millis) {
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, mavlink_system.type, MAV_AUTOPILOT_GENERIC_MISSION_FULL, mavlinkData.mode, mavlinkData.nav_mode, mavlinkData.status);
			// calculate idle time
			//mavCounter = counter;
			//mavlinkData.idlePercent = (mavCounter - mavlinkData.lastCounter) * minCycles * 1000.0f / (MAVLINK_HEARTBEAT_INTERVAL * rccClocks.SYSCLK_Frequency / 1e6f);
			mavCounter = 0;
			mavlinkData.idlePercent = 0;
			mavlinkData.lastCounter = mavCounter;

			mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 1000-mavlinkData.idlePercent, -1, -1, -1, 0, mavlinkData.packetDrops, 0, 0, 0, 0);

			mavlinkData.nextHeartbeat = millis + MAVLINK_HEARTBEAT_INTERVAL;
		} else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS]) && mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS] < millis) {
			mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, millis, 0, RADIO_THROT+1024, RADIO_ROLL+1024, RADIO_PITCH+1024, RADIO_RUDD+1024, RADIO_GEAR+1024, RADIO_FLAPS+1024, RADIO_AUX2+1024, RADIO_AUX3+1024, RADIO_QUALITY);
			mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0, millis, 0, (RADIO_THROT-750)*13, RADIO_ROLL*13, RADIO_PITCH*13, RADIO_RUDD*13, RADIO_GEAR*13, RADIO_FLAPS*13, RADIO_AUX2*13, RADIO_AUX3*13, RADIO_QUALITY);
			mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS] = millis + mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS];
		}

		if (chMBFetch(mavlinkData.noticeQueue, noticeBuf, TIME_IMMEDIATE) == RDY_OK) {
			mavlink_msg_statustext_send(MAVLINK_COMM_0, 0, (const char *)noticeBuf);
		}

		chThdSleepMilliseconds(10);
	}

	return 0;
}

void mavlinkInit(void) {
	unsigned long millis;
	int i;

	memset((void *)&mavlinkData, 0, sizeof(mavlinkData));

	const SerialConfig mavlinkPortConfig = {
		MAVLINK_SERIAL_BAUD,
		0,
		USART_CR2_STOP1_BITS | USART_CR2_LINEN,
		0
	};
	mavlinkData.serialPort = &MAVLINK_SERIAL_DEVICE;
	sdStart(mavlinkData.serialPort, &mavlinkPortConfig);
	palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)); //TX
	palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7)); //RX

	random_int(); // discard first random
	mavlink_system.sysid = (random_int() & 0xff); //sysid is random 0-255
	mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER;
	mavlink_system.type = MAV_TYPE_HEXAROTOR;

	mavlinkData.mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	mavlinkData.nav_mode = MAV_STATE_STANDBY;
	mavlinkData.status = MAV_STATE_ACTIVE;

	chMBInit(mavlinkData.noticeQueue, (msg_t*)noticeBuf, MAVLINK_NOTICE_DEPTH);

	millis = chTimeNow();
	for (i = 1; i < 13; i++) {
		mavlinkData.streamInterval[i] = 1e3f;
		mavlinkData.streamNext[i] = millis + 5e3f + i * 5.0;
	}

	chThdCreateStatic(waThreadMavlink, sizeof(waThreadMavlink), MAVLINK_PRIORITY, ThreadMavlink, NULL);
}
