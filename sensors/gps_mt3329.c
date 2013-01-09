#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

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

	const SerialConfig GPSPortConfig = {
	    38400,
	    0,
	    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
	    0
	};
	sdStart(&GPS_SERIAL_DEVICE, &GPSPortConfig);

	chThdSleepMilliseconds(500);

	// initialize serial port for binary protocol use
	chprintf((BaseSequentialStream *)&GPS_SERIAL_DEVICE, MTK_SET_BINARY);

	// set 4Hz update rate
	chprintf((BaseSequentialStream *)&GPS_SERIAL_DEVICE, MTK_OUTPUT_4HZ);

	chThdCreateStatic(PollGPSThreadWA,
			sizeof(PollGPSThreadWA),
			GPS_PRIORITY,
			PollGPSThread,
			NULL);
}
