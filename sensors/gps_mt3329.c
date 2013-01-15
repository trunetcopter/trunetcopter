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

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "comm.h"
#include "mavlink.h"

#include "sensors.h"
#include "gps_mt3329.h"

extern sensorData gSensorData;

void parse_gps_sentence (char *s)
{
  uint32_t tmp;
  uint32_t time_tmp;
  uint16_t header_msg_id;
  uint8_t ck_a=0, ck_b=0;
  int8_t i;

  for (i=2; i<35; ++i) {
    ck_a = ck_a + s[i];
    ck_b = ck_b + ck_a;
  }

  memcpy(&header_msg_id, &s[0], 2);
  if (header_msg_id == 0xddd0 && ck_a == s[35] && ck_b == s[36]) {
    memcpy(&gSensorData.latitude, &s[3], 4);

    memcpy(&gSensorData.longitude, &s[7], 4);

    memcpy(&tmp, &s[11], 4);
    gSensorData.altitude = (float)tmp/100.0;

    memcpy(&tmp, &s[15], 4);
    gSensorData.speed = (float)tmp/100.0;

	memcpy(&tmp, &s[19], 4);
	//data->heading = ((float)tmp/1.0e6)*M_PI/180.0;
	gSensorData.heading = (float)tmp/100.0;

	memcpy(&gSensorData.satellites, &s[23], 1);

	memcpy(&gSensorData.fix, &s[24], 1);

	memcpy(&gSensorData.utc_date, &s[25], 4);

	// time from gps is UTC, but we want milliseconds from epoch
	memcpy(&gSensorData.utc_time, &s[29], 4);
	time_tmp = gSensorData.utc_time;
	tmp = (time_tmp/10000000);
	time_tmp -= tmp*10000000;
	gSensorData.time = tmp * 3600000;
	tmp = (time_tmp/100000);
	time_tmp -= tmp*100000;
	gSensorData.time += tmp * 60000 + time_tmp;

	memcpy(&gSensorData.hdop, &s[33], 2);

	gSensorData.v_lat = cos(gSensorData.heading)*gSensorData.speed;

	gSensorData.v_lon = sin(gSensorData.heading)*gSensorData.speed;

    if (gSensorData.fix == FIX_3D) {
    	gSensorData.valid = 1;
    } else {
    	gSensorData.valid = 0;
    }
  }
  else gSensorData.valid = 0;
}

/**
 * Polling thread
 */
static WORKING_AREA(PollGPSThreadWA, 512);
static msg_t PollGPSThread(void *arg){
	(void)arg;
	chRegSetThreadName("PollGPS");

	EventListener elGps;
	chEvtRegisterMask((EventSource *)chnGetEventSource(&GPS_SERIAL_DEVICE), &elGps, 1);

	#define NMEA_BUF_SIZE 80
	char nmea_buf[NMEA_BUF_SIZE];
	int ptr=0;

	mavlinkNotice(MAV_SEVERITY_INFO, "GPS Polling Initialized!");

	while (TRUE) {
		flagsmask_t flags;
		int32_t c = 0;

		chEvtWaitOneTimeout(EVENT_MASK(1), MS2ST(10));
		flags = chEvtGetAndClearFlags(&elGps);

		if (flags & CHN_INPUT_AVAILABLE) {
			while (c != Q_TIMEOUT) {
				c = chnGetTimeout(&GPS_SERIAL_DEVICE, TIME_IMMEDIATE);
				if ((ptr==0 && c==0xd0) || (ptr==1 && c==0xdd) || (ptr > 1 && ptr < 36))
					nmea_buf[ptr++] = c;
				else if (ptr==36) {
					nmea_buf[ptr++] = c;
					nmea_buf[ptr] = 0;
					ptr = 0;
					parse_gps_sentence(nmea_buf);
				}
			}
		} else {
			chThdSleepMilliseconds(10);
		}
	}
	return 0;
}

void gps_mtk_start(void) {
	uint8_t i;
	const long baudrates[5] = {9600U, 19200U, 38400U, 57600U, 115200U};
	
	for (i=0; i<4; i++) {
		const SerialConfig GPSPortConfig = {
		    baudrates[i],
		    0,
		    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
		    0
		};
		sdStart(&GPS_SERIAL_DEVICE, &GPSPortConfig);

		// initialize serial port for binary protocol use
		chprintf((BaseSequentialStream *)&GPS_SERIAL_DEVICE, MTK_SET_BINARY);
		// set 5Hz update rate
		chprintf((BaseSequentialStream *)&GPS_SERIAL_DEVICE, MTK_OUTPUT_5HZ);
		// set SBAS on
		chprintf((BaseSequentialStream *)&GPS_SERIAL_DEVICE, SBAS_ON);
		// set WAAS on
		chprintf((BaseSequentialStream *)&GPS_SERIAL_DEVICE, WAAS_ON);
		// Set Nav Threshold to 0 m/s
		chprintf((BaseSequentialStream *)&GPS_SERIAL_DEVICE, MTK_NAVTHRES_OFF);

		int ptr = 0;
		int32_t c = 0;
		bool_t found_baud_rate = 0;
		while (c != Q_TIMEOUT) {
			c = chnGetTimeout(&GPS_SERIAL_DEVICE, MS2ST(100));
			if ((ptr==0 && c==0xd0) || (ptr==1 && c==0xdd))
                        	ptr++;
                        else if (ptr > 1) {
				found_baud_rate = 1;
				break;
                        } else
				ptr = 0;
		}
		if (found_baud_rate == 1)
			break;

		sdStop(&GPS_SERIAL_DEVICE);
	}

	chThdCreateStatic(PollGPSThreadWA,
			sizeof(PollGPSThreadWA),
			GPS_PRIORITY,
			PollGPSThread,
			NULL);
}
