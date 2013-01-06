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

sensorData gSensorData;
extern AHRS_state_data gStateData;

#define MAX_IMU_READINGS  1
#define MAX_MAGN_READINGS 1
int16_t readings[10];
int32_t sum[10];
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
	chEvtRegister(&eventEKFDone, &self_el2, EVT_EKF_DONE);

	uint32_t last_timer, current_timer;

	while (TRUE) {
		chEvtWaitAll(EVENT_MASK(EVT_IMU_IRQ) | EVENT_MASK(EVT_EKF_DONE));
		chEvtGetAndClearFlags(&self_el1);
		chEvtGetAndClearFlags(&self_el2);

		current_timer = TIM_GetCounter(TIM2);
		gSensorData.imuFrequency = 1.0 / ((current_timer - last_timer) / 1e6);
		last_timer = current_timer;

		mpu6050_getMotion6Temp(&readings[0], &readings[1], &readings[2], &readings[3], &readings[4], &readings[5], &readings[6]);
		sum[0] += readings[0];
		sum[1] += readings[1];
		sum[2] += readings[2];
		sum[3] += readings[3];
		sum[4] += readings[4];
		sum[5] += readings[5];
		sum[6] += readings[6];
		count[0]++;
		if (count[0] >= MAX_IMU_READINGS) {
			gSensorData.raw_acc_x = sum[0] / count[0];
			gSensorData.raw_acc_y = sum[1] / count[0];
			gSensorData.raw_acc_z = sum[2] / count[0];
			gSensorData.imuTemperature = mpu6050_convertTemperature((float)sum[6] / (float)count[0]);
			computeGyroTCBias();
			gSensorData.raw_gyr_x = (sum[3] / count[0]) - gSensorData.gyroRTBias[0] - gSensorData.gyroTCBias[0];
			gSensorData.raw_gyr_y = (sum[4] / count[0]) - gSensorData.gyroRTBias[1] - gSensorData.gyroTCBias[1];
			gSensorData.raw_gyr_z = (sum[5] / count[0]) - gSensorData.gyroRTBias[2] - gSensorData.gyroTCBias[2];

			sum[0] = 0;
			sum[1] = 0;
			sum[2] = 0;
			sum[3] = 0;
			sum[4] = 0;
			sum[5] = 0;
			sum[6] = 0;
			count[0] = 0;
			chEvtBroadcastFlags(&eventImuRead, EVT_IMU_READ);
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
	chEvtRegister(&eventEKFDone, &self_el2, EVT_EKF_DONE);

	uint32_t last_timer, current_timer;

	while (TRUE) {
		chEvtWaitAll(EVENT_MASK(EVT_MAGN_IRQ) | EVENT_MASK(EVT_EKF_DONE));
		chEvtGetAndClearFlags(&self_el1);
		chEvtGetAndClearFlags(&self_el2);

		current_timer = TIM_GetCounter(TIM2);
		gSensorData.magFrequency = 1.0 / ((current_timer - last_timer) / 1e6);
		last_timer = current_timer;

		hmc5883l_getRawHeading(&readings[7], &readings[8], &readings[9]);
		sum[7] += readings[7];
		sum[8] += readings[8];
		sum[9] += readings[9];
		count[1]++;
		if (count[1] >= MAX_MAGN_READINGS) {
			gSensorData.raw_mag_x = sum[7] / count[1];
			gSensorData.raw_mag_y = sum[8] / count[1];
			gSensorData.raw_mag_z = sum[9] / count[1];
			sum[8] = 0;
			sum[9] = 0;
			sum[0] = 0;
			chEvtBroadcastFlags(&eventMagnRead, EVT_MAGN_READ);
			count[1] = 0;
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

	uint32_t last_timer, current_timer;

	while (TRUE) {
		current_timer = TIM_GetCounter(TIM2);
		gSensorData.barFrequency = 1.0 / ((current_timer - last_timer) / 1e6);
		last_timer = current_timer;
		ms561101ba_readValues(&gSensorData.barPressure, &gSensorData.barTemperature, MS561101BA_OSR_4096);
		gSensorData.barAltitude = ms561101ba_getAltitude(gSensorData.barPressure, gSensorData.barTemperature);
		chThdSleepMilliseconds(10); // 100 Hz datasheet says 8.22ms on 4096 oversampling
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

void fullCalibrateGyro(void) {
	uint16_t numberOfGyroSamples = 2000;

	float gyroBias1[3] = { 0.0f, 0.0f, 0.0f };
	float gyroTemperature1 = 0.0f;

	float gyroBias2[3] = { 0.0f, 00.f, 0.0f };
	float gyroTemperature2 = 0.0f;

	float gyroTCBiasSlope[3], gyroTCBiasIntercept[3];

	int16_t gx, gy, gz;

	uint16_t index;

	for (index = 0; index < numberOfGyroSamples; index++) {
		mpu6050_getRotation(&gx, &gy, &gz);
		gyroBias1[0] += gx;
		gyroBias1[1] += gy;
		gyroBias1[2] += gz;
		gyroTemperature1 += mpu6050_getTemperature();
		chThdSleepMilliseconds(10);
	}

	gyroBias1[0] /= (float) numberOfGyroSamples;
	gyroBias1[1] /= (float) numberOfGyroSamples;
	gyroBias1[2] /= (float) numberOfGyroSamples;
	gyroTemperature1 /= (float) numberOfGyroSamples;

	chThdSleepMilliseconds(1000 * 10);

	for (index = 0; index < numberOfGyroSamples; index++) {
		mpu6050_getRotation(&gx, &gy, &gz);
		gyroBias2[0] += gx;
		gyroBias2[1] += gy;
		gyroBias2[2] += gz;
		gyroTemperature2 += mpu6050_getTemperature();
		chThdSleepMilliseconds(10);
	}

	gyroBias2[0] /= (float) numberOfGyroSamples;
	gyroBias2[1] /= (float) numberOfGyroSamples;
	gyroBias2[2] /= (float) numberOfGyroSamples;
	gyroTemperature2 /= (float) numberOfGyroSamples;

	gyroTCBiasSlope[0] = (gyroBias2[0] - gyroBias1[0]) / (gyroTemperature2 - gyroTemperature1);
	gyroTCBiasSlope[1] = (gyroBias2[1] - gyroBias1[1]) / (gyroTemperature2 - gyroTemperature1);
	gyroTCBiasSlope[2] = (gyroBias2[2] - gyroBias1[2]) / (gyroTemperature2 - gyroTemperature1);

	gyroTCBiasIntercept[0] = gyroBias2[0] - gyroTCBiasSlope[0] * gyroTemperature2;
	gyroTCBiasIntercept[1] = gyroBias2[1] - gyroTCBiasSlope[1] * gyroTemperature2;
	gyroTCBiasIntercept[2] = gyroBias2[2] - gyroTCBiasSlope[2] * gyroTemperature2;
}

void computeGyroTCBias(void) {
	gSensorData.gyroTCBias[0] = gSensorData.gyroTCBiasSlope[0] * gSensorData.imuTemperature + gSensorData.gyroTCBiasIntercept[0];
	gSensorData.gyroTCBias[1] = gSensorData.gyroTCBiasSlope[1] * gSensorData.imuTemperature + gSensorData.gyroTCBiasIntercept[1];
	gSensorData.gyroTCBias[2] = gSensorData.gyroTCBiasSlope[2] * gSensorData.imuTemperature + gSensorData.gyroTCBiasIntercept[2];
}

void computeGyroRTBias(void) {
	uint8_t axis;
	uint16_t samples;
	float gyroSum[3] = { 0.0f, 0.0f, 0.0f };

	int16_t gx, gy, gz;

	for (samples = 0; samples < 2000; samples++) {
		mpu6050_getRotation(&gx, &gy, &gz);
		gSensorData.imuTemperature = mpu6050_getTemperature();

		computeGyroTCBias();

		gyroSum[0] += gx - gSensorData.gyroTCBias[0];
		gyroSum[1] += gy - gSensorData.gyroTCBias[1];
		gyroSum[2] += gz - gSensorData.gyroTCBias[2];

		chThdSleepMilliseconds(10);
	}

	for (axis = 0; axis < 3; axis++) {
		gSensorData.gyroRTBias[axis] = (float) gyroSum[axis] / 2000.0f;
	}
}

void initSensors(void) {
	memset((void *)&gSensorData, 0, sizeof(gSensorData));
	gSensorData.gyroTCBiasSlope[0] = 4.79346609;
	gSensorData.gyroTCBiasSlope[1] = 0.256198853;
	gSensorData.gyroTCBiasSlope[2] = 0.467166692;
	gSensorData.gyroTCBiasIntercept[0] = -269.210388;
	gSensorData.gyroTCBiasIntercept[1] = -2.34643841;
	gSensorData.gyroTCBiasIntercept[2] = 32.0404129;

	mpu6050_initialize();
	if (mpu6050_testConnection() != 1)
		chDbgPanic("MPU6050: not found");
	mpu6050_setRate(0x04); // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz
	mpu6050_setDLPFMode(MPU6050_DLPF_BW_98);
	mpu6050_setIntEnabled(1);
	mpu6050_setInterruptLatchClear(1);
	mpu6050_setI2CBypassEnabled(1);

	computeGyroRTBias();

	hmc5883l_initialize();
	if (hmc5883l_testConnection() != 1)
		chDbgPanic("HMC5883L: not found");
	hmc5883l_calibrate(HMC5883L_GAIN_1090);
	hmc5883l_setSampleAveraging(HMC5883L_AVERAGING_8);
	hmc5883l_setDataRate(HMC5883L_RATE_75);
	hmc5883l_setMeasurementBias(HMC5883L_BIAS_NORMAL);

	ms561101ba_initialize();
	//if (ms561101ba_testConnection() != 1)
	//	chDbgPanic("MS5611-01BA: not found");
	//if (ms561101ba_crc() != 1)
	//	chDbgPanic("MS5611-01BA: PROM CRC mismatch");
	ms561101ba_setOverSampleRate(MS561101BA_OSR_4096);
}
