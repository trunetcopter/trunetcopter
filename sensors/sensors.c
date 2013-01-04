#include "ch.h"
#include "hal.h"

#include <math.h>
#include <string.h>

#include "sensors.h"

#include "imu_mpu6050.h"
#include "magn_hmc5883l.h"
#include "press_ms561101ba.h"

#include "../attitude_estimation/estimation.h"

EventSource eventImuIrq;
EventSource eventMagnIrq;
EventSource eventImuRead;
EventSource eventMagnRead;
EventSource eventEKFDone;

extern RawSensorData gSensorData;
extern AHRS_state_data gStateData;

#define MAX_READINGS 4
int16_t readings[9];
int32_t sum[9];
uint8_t count[2];

/**
 * IMU Polling thread
 */
static WORKING_AREA(PollIMUThreadWA, 512);
static msg_t PollIMUThread(void *arg){
	(void)arg;
	chRegSetThreadName("PollIMU");

	EventListener self_el1, self_el2;
	chEvtRegister(&eventImuIrq, &self_el1, EVT_IMU_IRQ);
	//chEvtRegister(&eventEKFDone, &self_el2, EVT_EKF_DONE);

	while (TRUE) {
		chEvtWaitOne(EVENT_MASK(EVT_IMU_IRQ));// | EVENT_MASK(EVT_EKF_DONE));
		chEvtGetAndClearFlags(&self_el1);
		chEvtGetAndClearFlags(&self_el2);
		gSensorData.temperature = mpu6050_getTemperature();
		mpu6050_getMotion6(&readings[0], &readings[1], &readings[2], &readings[3], &readings[4], &readings[5]);
		gSensorData.new_accel_data = 1;
		gSensorData.new_gyro_data = 1;
		sum[0] += readings[0];
		sum[1] += readings[1];
		sum[2] += readings[2];
		sum[3] += readings[3];
		sum[4] += readings[4];
		sum[5] += readings[5];
		if (count[0] >= MAX_READINGS) {
			gSensorData.accel_x = sum[0] / count[0];
			gSensorData.accel_y = sum[1] / count[0];
			gSensorData.accel_z = sum[2] / count[0];
			gSensorData.gyro_x = sum[3] / count[0];
			gSensorData.gyro_y = sum[4] / count[0];
			gSensorData.gyro_z = sum[5] / count[0];
			sum[0] = 0;
			sum[1] = 0;
			sum[2] = 0;
			sum[3] = 0;
			sum[4] = 0;
			sum[5] = 0;
			chEvtBroadcastFlags(&eventImuRead, EVT_IMU_READ);
			count[0] = 0;
		} else {
			count[0]++;
		}
	}
	return 0;
}

/**
 * Magnetometer Polling thread
 */
static WORKING_AREA(PollMagnThreadWA, 512);
static msg_t PollMagnThread(void *arg){
	(void)arg;
	chRegSetThreadName("PollMagn");

	EventListener self_el1, self_el2;
	chEvtRegister(&eventMagnIrq, &self_el1, EVT_MAGN_IRQ);
	//chEvtRegister(&eventEKFDone, &self_el2, EVT_EKF_DONE);

	while (TRUE) {
		chEvtWaitOne(EVENT_MASK(EVT_MAGN_IRQ));// | EVENT_MASK(EVT_EKF_DONE));
		chEvtGetAndClearFlags(&self_el1);
		chEvtGetAndClearFlags(&self_el2);
		hmc5883l_getRawHeading(&readings[6], &readings[7], &readings[8]);
		gSensorData.new_mag_data = 1;
		sum[6] += readings[6];
		sum[7] += readings[7];
		sum[8] += readings[8];
		if (count[1] >= MAX_READINGS) {
			gSensorData.mag_x = sum[6] / count[1];
			gSensorData.mag_y = sum[7] / count[1];
			gSensorData.mag_z = sum[8] / count[1];
			sum[6] = 0;
			sum[7] = 0;
			sum[8] = 0;
			chEvtBroadcastFlags(&eventMagnRead, EVT_MAGN_READ);
			count[1] = 0;
		} else {
			count[1]++;
		}
	}
	return 0;
}

/**
 * Barometer Polling thread
 */
static WORKING_AREA(PollBaroThreadWA, 512);
static msg_t PollBaroThread(void *arg){
	(void)arg;
	chRegSetThreadName("PollBaro");

	while (TRUE) {
		ms561101ba_readValues(&gSensorData.pressure, &gSensorData.baroTemp, MS561101BA_OSR_4096);
		chThdSleepMilliseconds(100); // 10 Hz datasheet says 8.22ms on 4096 oversampling
	}
	return 0;
}

void startSensors(void) {
	memset((void *)&readings, 0, sizeof(readings));
	memset((void *)&sum, 0, sizeof(sum));
	memset((void *)&count, 0, sizeof(count));

	chThdCreateStatic(PollIMUThreadWA,
			sizeof(PollIMUThreadWA),
			NORMALPRIO+2,
			PollIMUThread,
			NULL);
	chThdCreateStatic(PollMagnThreadWA,
			sizeof(PollMagnThreadWA),
			NORMALPRIO+1,
			PollMagnThread,
			NULL);
	chThdCreateStatic(PollBaroThreadWA,
			sizeof(PollBaroThreadWA),
			NORMALPRIO-1,
			PollBaroThread,
			NULL);
}

void initSensors(void) {
	mpu6050_initialize();
	if (mpu6050_testConnection() != 1)
		chDbgPanic("MPU6050 not found");
	mpu6050_setRate(0x04); // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz
	mpu6050_setDLPFMode(MPU6050_DLPF_BW_98);
	mpu6050_setIntEnabled(1);
	mpu6050_setInterruptLatchClear(1);
	mpu6050_setI2CBypassEnabled(1);

	hmc5883l_initialize();
	if (hmc5883l_testConnection() != 1)
		chDbgPanic("HMC5883L not found");
	//hmc5883l_calibrate(HMC5883L_GAIN_1090);
	hmc5883l_setSampleAveraging(HMC5883L_AVERAGING_8);
	hmc5883l_setDataRate(HMC5883L_RATE_75);
	hmc5883l_setMeasurementBias(HMC5883L_BIAS_NORMAL);

	ms561101ba_initialize();
	ms561101ba_testConnection();
	ms561101ba_setOverSampleRate(MS561101BA_OSR_4096);
}
