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

#include "comm.h"

#include "config.h"
#include "util.h"
#include "radio/radio.h"
#include "sensors/sensors.h"

#include "mavlink.h"
#include "mavlink_types.h"

#include "attitude_estimation/estimation.h"

extern sensorData gSensorData;
extern AHRS_state_data gStateData;

mavlinkStruct_t mavlinkData;
mavlink_system_t mavlink_system;

static uint32_t last_idle_ticks = 0;

Mailbox mbNotice;
Mailbox mbImu;
Mailbox mbMagn;
Mailbox mbGps;
static msg_t mbNoticeBuf[8];
static msg_t mbImuBuf[1];
static msg_t mbMagnBuf[1];
static msg_t mbGpsBuf[1];

mavlink_statustext_t mavlink_statustext_struct;

static Mail statustext_mail = {NULL, MAVLINK_MSG_ID_STATUSTEXT, NULL};
static BinarySemaphore semStatustext;

/**
 * Signal processing thread about end of data processing
 */
void ReleaseMail(Mail* mailp){
  mailp->payload = NULL;
  if (mailp->semp != NULL)
    chBSemSignal(mailp->semp);
}

/**
 *
 */
void MsgInit(void){
	chBSemInit(&semStatustext, FALSE);
	chMBInit(&mbNotice,
		   mbNoticeBuf,
		   (sizeof(mbNoticeBuf)/sizeof(msg_t)));
	chMBInit(&mbImu,
  		   mbImuBuf,
  		   (sizeof(mbImuBuf)/sizeof(msg_t)));
	chMBInit(&mbMagn,
  		   mbMagnBuf,
  		   (sizeof(mbMagnBuf)/sizeof(msg_t)));
	chMBInit(&mbGps,
  		   mbGpsBuf,
  		   (sizeof(mbGpsBuf)/sizeof(msg_t)));
}

void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
	if (chan == MAVLINK_COMM_0) {
		sdWrite(mavlinkData.serialPort, &ch, 1);
	}
}

msg_t mavlinkNotice(uint8_t severity, const char *text) {
	uint32_t n = sizeof(mavlink_statustext_struct.text);
	msg_t status = RDY_RESET;

	status = chBSemWaitTimeout(&semStatustext, MS2ST(1000));
	if (status != RDY_OK){
		chBSemSignal(&semStatustext);
		return status;
	}

	mavlink_statustext_struct.severity = severity;
	memset(mavlink_statustext_struct.text, 0, n);
	memcpy(mavlink_statustext_struct.text, text, n);

	statustext_mail.payload = &mavlink_statustext_struct;
	statustext_mail.semp = &semStatustext;
	status = chMBPost(&mbNotice, (msg_t)&statustext_mail, TIME_IMMEDIATE);
	if (status != RDY_OK){
		chBSemSignal(&semStatustext);
		return status;
	}
	return status;
}

systime_t GetTimeInterval(systime_t *last){
  systime_t t = 0;

  if (chTimeNow() >= *last)
    t = chTimeNow() - *last;
  else /* overflow happens */
    t = chTimeNow() + (0xFFFFFFFF - *last);
  /* refresh last value */
  *last = chTimeNow();
  return t;
}

uint16_t get_cpu_load(Thread *idleThread){

  uint32_t i, s;
  systime_t ticks;

  ticks = chThdGetTicks(idleThread);

  if (ticks >= last_idle_ticks)
    i = ticks - last_idle_ticks;
  else /* overflow */
    i = ticks + (0xFFFFFFFF - last_idle_ticks);

  last_idle_ticks = ticks;

  s = GetTimeInterval(0);

  return ((s - i) * 1000) / s;
}

static WORKING_AREA(waThreadMavlink, MAVLINK_STACK_SIZE);
static msg_t ThreadMavlink(void *arg) {
        (void)arg;
        chRegSetThreadName("mavlink");

	static unsigned long lastMillis = 0;
	unsigned long millis;
	mavlinkNotice(MAV_SEVERITY_INFO, "MavLink Initialized!");

	static Thread *IdleThread_p = NULL;
	IdleThread_p = chSysGetIdleThread();

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
			mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, get_cpu_load(IdleThread_p), -1, -1, -1, 0, mavlinkData.packetDrops, 0, 0, 0, 0);
			mavlinkData.nextHeartbeat = millis + MAVLINK_HEARTBEAT_INTERVAL;
		} else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS]) && mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS] < millis) {
			mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, millis, 0, RADIO_THROT+1024, RADIO_ROLL+1024, RADIO_PITCH+1024, RADIO_RUDD+1024, RADIO_GEAR+1024, RADIO_FLAPS+1024, RADIO_AUX2+1024, RADIO_AUX3+1024, RADIO_QUALITY);
			mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0, millis, 0, (RADIO_THROT-750)*13, RADIO_ROLL*13, RADIO_PITCH*13, RADIO_RUDD*13, RADIO_GEAR*13, RADIO_FLAPS*13, RADIO_AUX2*13, RADIO_AUX3*13, RADIO_QUALITY);
			mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS] = millis + mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS] + 400;
		} else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_SENSORS]) && mavlinkData.streamNext[MAV_DATA_STREAM_RAW_SENSORS] < millis) {
			mavlink_msg_raw_imu_send(MAVLINK_COMM_0, millis*1000, gSensorData.raw_acc_x, gSensorData.raw_acc_y, gSensorData.raw_acc_z, gSensorData.raw_gyr_x, gSensorData.raw_gyr_y, gSensorData.raw_gyr_z, gSensorData.raw_mag_x, gSensorData.raw_mag_y, gSensorData.raw_mag_z);
			mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, millis, gSensorData.scaled_acc_x*1000.0f, gSensorData.scaled_acc_y * 1000.0f, gSensorData.scaled_acc_z * 1000.0f, gSensorData.scaled_gyr_x * 1000.0f, gSensorData.scaled_gyr_y * 1000.0f, gSensorData.scaled_gyr_z * 1000.0f, gSensorData.scaled_mag_x, gSensorData.scaled_mag_y, gSensorData.scaled_mag_z);
			mavlink_msg_scaled_pressure_send(MAVLINK_COMM_0, millis, gSensorData.barPressure, 0.0f, gSensorData.barTemperature);
			mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, gSensorData.utc_time, gSensorData.fix, gSensorData.latitude, gSensorData.longitude, gSensorData.altitude*1000, gSensorData.hdop, 65535, ((float)gSensorData.speed * 3.6f) * 100, gSensorData.heading * 100, gSensorData.satellites);
			mavlinkData.streamNext[MAV_DATA_STREAM_RAW_SENSORS] = millis + mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_SENSORS];
		} else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER]) && mavlinkData.streamNext[MAV_DATA_STREAM_RAW_CONTROLLER] < millis) {
			mavlink_msg_attitude_send(MAVLINK_COMM_0, millis, gStateData.roll, (float)gStateData.pitch, gStateData.yaw, gStateData.roll_rate, gStateData.pitch_rate, gStateData.yaw_rate);
			mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, millis, gStateData.qib.a, gStateData.qib.b, gStateData.qib.c, gStateData.qib.d, gStateData.roll_rate, gStateData.pitch_rate, gStateData.yaw_rate);
			//mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, micros, 0, motorsData.value[0], motorsData.value[1], motorsData.value[2], motorsData.value[3], motorsData.value[4], motorsData.value[5], motorsData.value[6], motorsData.value[7]);
			mavlinkData.streamNext[MAV_DATA_STREAM_RAW_CONTROLLER] = millis + mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER];
	    }

		mavlink_statustext_t *statustext = NULL;
		Mail *mailp;
		msg_t tmp = 0;

		if (chMBFetch(&mbNotice, &tmp, TIME_IMMEDIATE) == RDY_OK) {
			mailp = (Mail*)tmp;
			statustext = (mavlink_statustext_t *)mailp->payload;
			mavlink_msg_statustext_send(MAVLINK_COMM_0, statustext->severity, statustext->text);
			if (mailp->semp != NULL)
				chBSemSignal(mailp->semp);
		}

		chThdSleepMilliseconds(10);
	}

	return 0;
}

void mavlinkInit(void) {
	unsigned long millis;
	int i;

	memset((void *)&mavlinkData, 0, sizeof(mavlinkData));
	MsgInit();

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

	//random_int(); // discard first random
	//mavlink_system.sysid = (random_int() & 0xff); //sysid is random 0-255
	mavlink_system.sysid = p[SYSTEM_ID];
	mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER;
	mavlink_system.type = MAV_TYPE_HEXAROTOR;

	mavlinkData.mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	mavlinkData.nav_mode = MAV_STATE_STANDBY;
	mavlinkData.status = MAV_STATE_ACTIVE;

	millis = chTimeNow();
	for (i = 1; i < 13; i++) {
		mavlinkData.streamInterval[i] = 1e2f;
		mavlinkData.streamNext[i] = millis + 1e2f + i * 5.0;
	}

	chThdCreateStatic(waThreadMavlink, sizeof(waThreadMavlink), MAVLINK_PRIORITY, ThreadMavlink, NULL);
}
