/* ------------------------------------------------------------------------------
  File: estimation.c
  Author: CH Robotics
  Version: 1.0

  Description: Function definitions for CHR-6dm state estimation.
------------------------------------------------------------------------------ */

#include <arm_math.h>
#include <math.h>
#include <string.h>
#include "estimation.h"
#include "matrix.h"
#include "quat.h"

#include "stm32f4xx_tim.h"

#include "../util.h"
#include "../sensors/sensors.h"

AHRS_state_data gStateData;
extern sensorData gSensorData;

extern EventSource eventImuIrq;
extern EventSource eventMagnIrq;
extern EventSource eventImuRead;
extern EventSource eventMagnRead;
extern EventSource eventEKFDone;

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
        if (isnan(v)) {
                return 0.0;
        }
        if (v >= 1.0) {
                return PI/2;
        }
        if (v <= -1.0) {
                return -PI/2;
        }
        return asin(v);
}

void compute_euler_angles( AHRS_state_data* estimated_states )
{
	// APM
	estimated_states->roll = -(atan2(2.0*(estimated_states->qib.a*estimated_states->qib.b + estimated_states->qib.c*estimated_states->qib.d), 1 - 2.0*(estimated_states->qib.b*estimated_states->qib.b + estimated_states->qib.c*estimated_states->qib.c)));
	estimated_states->pitch = -safe_asin(2.0*(estimated_states->qib.a*estimated_states->qib.c - estimated_states->qib.d*estimated_states->qib.b));
	estimated_states->yaw = atan2(2.0*(estimated_states->qib.a*estimated_states->qib.d + estimated_states->qib.b*estimated_states->qib.c), 1 - 2.0*(estimated_states->qib.c*estimated_states->qib.c + estimated_states->qib.d*estimated_states->qib.d));
}

void compute_yaw_pitch_roll( AHRS_state_data* estimated_states ) {
	float gx, gy, gz; // estimated gravity direction

	gx = 2 * (estimated_states->qib.b*estimated_states->qib.d - estimated_states->qib.a*estimated_states->qib.c);
	gy = 2 * (estimated_states->qib.a*estimated_states->qib.b + estimated_states->qib.c*estimated_states->qib.d);
	gz = estimated_states->qib.a*estimated_states->qib.a - estimated_states->qib.b*estimated_states->qib.b - estimated_states->qib.c*estimated_states->qib.c + estimated_states->qib.d*estimated_states->qib.d;

	estimated_states->yaw = atan2(2 * estimated_states->qib.b * estimated_states->qib.c - 2 * estimated_states->qib.a * estimated_states->qib.d, 2 * estimated_states->qib.a*estimated_states->qib.a + 2 * estimated_states->qib.b * estimated_states->qib.b - 1) * 180/PI;
	estimated_states->pitch = atan(gx / sqrt(gy*gy + gz*gz))  * 180/PI;
	estimated_states->roll = atan(gy / sqrt(gx*gx + gz*gz))  * 180/PI;
}

float invSqrt(float x) {
	// from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root
	unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
	float tmp = *(float*)&i;
	float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
	return y;
	// from madgwick
	/*
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
	*/
}

void MadgwickAHRSupdate(AHRS_state_data* estimated_states, sensorData* sensor_data) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((gSensorData.scaled_mag_x == 0.0f) && (gSensorData.scaled_mag_y == 0.0f) && (gSensorData.scaled_mag_z == 0.0f)) {
		MadgwickAHRSupdateIMU( estimated_states, sensor_data );
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-estimated_states->qib.b * sensor_data->scaled_gyr_x - estimated_states->qib.c * sensor_data->scaled_gyr_y - estimated_states->qib.d * sensor_data->scaled_gyr_z);
	qDot2 = 0.5f * (estimated_states->qib.a * sensor_data->scaled_gyr_x + estimated_states->qib.c * sensor_data->scaled_gyr_z - estimated_states->qib.d * sensor_data->scaled_gyr_y);
	qDot3 = 0.5f * (estimated_states->qib.a * sensor_data->scaled_gyr_y - estimated_states->qib.b * sensor_data->scaled_gyr_z + estimated_states->qib.d * sensor_data->scaled_gyr_x);
	qDot4 = 0.5f * (estimated_states->qib.a * sensor_data->scaled_gyr_z + estimated_states->qib.b * sensor_data->scaled_gyr_y - estimated_states->qib.c * sensor_data->scaled_gyr_x);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((sensor_data->scaled_acc_x == 0.0f) && (sensor_data->scaled_acc_y == 0.0f) && (sensor_data->scaled_acc_z == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(sensor_data->scaled_acc_x * sensor_data->scaled_acc_x + sensor_data->scaled_acc_y * sensor_data->scaled_acc_y + sensor_data->scaled_acc_z * sensor_data->scaled_acc_z);
		sensor_data->scaled_acc_x *= recipNorm;
		sensor_data->scaled_acc_y *= recipNorm;
		sensor_data->scaled_acc_z *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(sensor_data->scaled_mag_x * sensor_data->scaled_mag_x + sensor_data->scaled_mag_y * sensor_data->scaled_mag_y + sensor_data->scaled_mag_z * sensor_data->scaled_mag_z);
		sensor_data->scaled_mag_x *= recipNorm;
		sensor_data->scaled_mag_y *= recipNorm;
		sensor_data->scaled_mag_z *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * estimated_states->qib.a * sensor_data->scaled_mag_x;
		_2q0my = 2.0f * estimated_states->qib.a * sensor_data->scaled_mag_y;
		_2q0mz = 2.0f * estimated_states->qib.a * sensor_data->scaled_mag_z;
		_2q1mx = 2.0f * estimated_states->qib.b * sensor_data->scaled_mag_x;
		_2q0 = 2.0f * estimated_states->qib.a;
		_2q1 = 2.0f * estimated_states->qib.b;
		_2q2 = 2.0f * estimated_states->qib.c;
		_2q3 = 2.0f * estimated_states->qib.d;
		_2q0q2 = 2.0f * estimated_states->qib.a * estimated_states->qib.c;
		_2q2q3 = 2.0f * estimated_states->qib.c * estimated_states->qib.d;
		q0q0 = estimated_states->qib.a * estimated_states->qib.a;
		q0q1 = estimated_states->qib.a * estimated_states->qib.b;
		q0q2 = estimated_states->qib.a * estimated_states->qib.c;
		q0q3 = estimated_states->qib.a * estimated_states->qib.d;
		q1q1 = estimated_states->qib.b * estimated_states->qib.b;
		q1q2 = estimated_states->qib.b * estimated_states->qib.c;
		q1q3 = estimated_states->qib.b * estimated_states->qib.d;
		q2q2 = estimated_states->qib.c * estimated_states->qib.c;
		q2q3 = estimated_states->qib.c * estimated_states->qib.d;
		q3q3 = estimated_states->qib.d * estimated_states->qib.d;

		// Reference direction of Earth's magnetic field
		hx = sensor_data->scaled_mag_x * q0q0 - _2q0my * estimated_states->qib.d + _2q0mz * estimated_states->qib.c + sensor_data->scaled_mag_x * q1q1 + _2q1 * sensor_data->scaled_mag_y * estimated_states->qib.c + _2q1 * sensor_data->scaled_mag_z * estimated_states->qib.d - sensor_data->scaled_mag_x * q2q2 - sensor_data->scaled_mag_x * q3q3;
		hy = _2q0mx * estimated_states->qib.d + sensor_data->scaled_mag_y * q0q0 - _2q0mz * estimated_states->qib.b + _2q1mx * estimated_states->qib.c - sensor_data->scaled_mag_y * q1q1 + sensor_data->scaled_mag_y * q2q2 + _2q2 * sensor_data->scaled_mag_z * estimated_states->qib.d - sensor_data->scaled_mag_y * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * estimated_states->qib.c + _2q0my * estimated_states->qib.b + sensor_data->scaled_mag_z * q0q0 + _2q1mx * estimated_states->qib.d - sensor_data->scaled_mag_z * q1q1 + _2q2 * sensor_data->scaled_mag_y * estimated_states->qib.d - sensor_data->scaled_mag_z * q2q2 + sensor_data->scaled_mag_z * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - sensor_data->scaled_acc_x) + _2q1 * (2.0f * q0q1 + _2q2q3 - sensor_data->scaled_acc_y) - _2bz * estimated_states->qib.c * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - sensor_data->scaled_mag_x) + (-_2bx * estimated_states->qib.d + _2bz * estimated_states->qib.b) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - sensor_data->scaled_mag_y) + _2bx * estimated_states->qib.c * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - sensor_data->scaled_mag_z);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - sensor_data->scaled_acc_x) + _2q0 * (2.0f * q0q1 + _2q2q3 - sensor_data->scaled_acc_y) - 4.0f * estimated_states->qib.b * (1 - 2.0f * q1q1 - 2.0f * q2q2 - sensor_data->scaled_acc_z) + _2bz * estimated_states->qib.d * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - sensor_data->scaled_mag_x) + (_2bx * estimated_states->qib.c + _2bz * estimated_states->qib.a) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - sensor_data->scaled_mag_y) + (_2bx * estimated_states->qib.d - _4bz * estimated_states->qib.b) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - sensor_data->scaled_mag_z);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - sensor_data->scaled_acc_x) + _2q3 * (2.0f * q0q1 + _2q2q3 - sensor_data->scaled_acc_y) - 4.0f * estimated_states->qib.c * (1 - 2.0f * q1q1 - 2.0f * q2q2 - sensor_data->scaled_acc_z) + (-_4bx * estimated_states->qib.c - _2bz * estimated_states->qib.a) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - sensor_data->scaled_mag_x) + (_2bx * estimated_states->qib.b + _2bz * estimated_states->qib.d) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - sensor_data->scaled_mag_y) + (_2bx * estimated_states->qib.a - _4bz * estimated_states->qib.c) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - sensor_data->scaled_mag_z);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - sensor_data->scaled_acc_x) + _2q2 * (2.0f * q0q1 + _2q2q3 - sensor_data->scaled_acc_y) + (-_4bx * estimated_states->qib.d + _2bz * estimated_states->qib.b) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - sensor_data->scaled_mag_x) + (-_2bx * estimated_states->qib.a + _2bz * estimated_states->qib.c) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - sensor_data->scaled_mag_y) + _2bx * estimated_states->qib.b * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - sensor_data->scaled_mag_z);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= estimated_states->beta * s0;
		qDot2 -= estimated_states->beta * s1;
		qDot3 -= estimated_states->beta * s2;
		qDot4 -= estimated_states->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	estimated_states->qib.a += qDot1 * (1.0f / estimated_states->estFrequency);
	estimated_states->qib.b += qDot2 * (1.0f / estimated_states->estFrequency);
	estimated_states->qib.c += qDot3 * (1.0f / estimated_states->estFrequency);
	estimated_states->qib.d += qDot4 * (1.0f / estimated_states->estFrequency);

	// Normalise quaternion
	recipNorm = invSqrt(estimated_states->qib.a * estimated_states->qib.a + estimated_states->qib.b * estimated_states->qib.b + estimated_states->qib.c * estimated_states->qib.c + estimated_states->qib.d * estimated_states->qib.d);
	estimated_states->qib.a *= recipNorm;
	estimated_states->qib.b *= recipNorm;
	estimated_states->qib.c *= recipNorm;
	estimated_states->qib.d *= recipNorm;
}

void MadgwickAHRSupdateIMU(AHRS_state_data* estimated_states, sensorData* sensor_data) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-estimated_states->qib.b * sensor_data->scaled_gyr_x - estimated_states->qib.c * sensor_data->scaled_gyr_y - estimated_states->qib.d * sensor_data->scaled_gyr_z);
	qDot2 = 0.5f * (estimated_states->qib.a * sensor_data->scaled_gyr_x + estimated_states->qib.c * sensor_data->scaled_gyr_z - estimated_states->qib.d * sensor_data->scaled_gyr_y);
	qDot3 = 0.5f * (estimated_states->qib.a * sensor_data->scaled_gyr_y - estimated_states->qib.b * sensor_data->scaled_gyr_z + estimated_states->qib.d * sensor_data->scaled_gyr_x);
	qDot4 = 0.5f * (estimated_states->qib.a * sensor_data->scaled_gyr_z + estimated_states->qib.b * sensor_data->scaled_gyr_y - estimated_states->qib.c * sensor_data->scaled_gyr_x);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((sensor_data->scaled_acc_x == 0.0f) && (sensor_data->scaled_acc_y == 0.0f) && (sensor_data->scaled_acc_z == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(sensor_data->scaled_acc_x * sensor_data->scaled_acc_x + sensor_data->scaled_acc_y * sensor_data->scaled_acc_y + sensor_data->scaled_acc_z * sensor_data->scaled_acc_z);
		sensor_data->scaled_acc_x *= recipNorm;
		sensor_data->scaled_acc_y *= recipNorm;
		sensor_data->scaled_acc_z *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * estimated_states->qib.a;
		_2q1 = 2.0f * estimated_states->qib.b;
		_2q2 = 2.0f * estimated_states->qib.c;
		_2q3 = 2.0f * estimated_states->qib.d;
		_4q0 = 4.0f * estimated_states->qib.a;
		_4q1 = 4.0f * estimated_states->qib.b;
		_4q2 = 4.0f * estimated_states->qib.c;
		_8q1 = 8.0f * estimated_states->qib.b;
		_8q2 = 8.0f * estimated_states->qib.c;
		q0q0 = estimated_states->qib.a * estimated_states->qib.a;
		q1q1 = estimated_states->qib.b * estimated_states->qib.b;
		q2q2 = estimated_states->qib.c * estimated_states->qib.c;
		q3q3 = estimated_states->qib.d * estimated_states->qib.d;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * sensor_data->scaled_acc_x + _4q0 * q1q1 - _2q1 * sensor_data->scaled_acc_y;
		s1 = _4q1 * q3q3 - _2q3 * sensor_data->scaled_acc_x + 4.0f * q0q0 * estimated_states->qib.b - _2q0 * sensor_data->scaled_acc_y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * sensor_data->scaled_acc_z;
		s2 = 4.0f * q0q0 * estimated_states->qib.c + _2q0 * sensor_data->scaled_acc_x + _4q2 * q3q3 - _2q3 * sensor_data->scaled_acc_y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * sensor_data->scaled_acc_z;
		s3 = 4.0f * q1q1 * estimated_states->qib.d - _2q1 * sensor_data->scaled_acc_x + 4.0f * q2q2 * estimated_states->qib.d - _2q2 * sensor_data->scaled_acc_y;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= estimated_states->beta * s0;
		qDot2 -= estimated_states->beta * s1;
		qDot3 -= estimated_states->beta * s2;
		qDot4 -= estimated_states->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	estimated_states->qib.a += qDot1 * (1.0f / estimated_states->estFrequency);
	estimated_states->qib.b += qDot2 * (1.0f / estimated_states->estFrequency);
	estimated_states->qib.c += qDot3 * (1.0f / estimated_states->estFrequency);
	estimated_states->qib.d += qDot4 * (1.0f / estimated_states->estFrequency);

	// Normalise quaternion
	recipNorm = invSqrt(estimated_states->qib.a * estimated_states->qib.a + estimated_states->qib.b * estimated_states->qib.b + estimated_states->qib.c * estimated_states->qib.c + estimated_states->qib.d * estimated_states->qib.d);
	estimated_states->qib.a *= recipNorm;
	estimated_states->qib.b *= recipNorm;
	estimated_states->qib.c *= recipNorm;
	estimated_states->qib.d *= recipNorm;
}

/**
 * Attitude Estimation thread
 */
static WORKING_AREA(PollAttitudeThreadWA, 1024);
static msg_t PollAttitudeThread(void *arg){
	(void)arg;
	chRegSetThreadName("PollAttitude");

	memset((void *)&gStateData, 0, sizeof(gStateData));

	EventListener self_el1, self_el2;
	chEvtRegister(&eventImuRead, &self_el1, EVT_IMU_READ);
	chEvtRegister(&eventMagnRead, &self_el2, EVT_MAGN_READ);

	gStateData.qib.a = 1.0f;

	gStateData.beta = betaDef;

	uint32_t last_timer, current_timer;

	while (TRUE) {
		chEvtWaitAll(EVENT_MASK(EVT_IMU_READ) | EVENT_MASK(EVT_MAGN_READ));
		chEvtGetAndClearFlags(&self_el1);
		chEvtGetAndClearFlags(&self_el2);

		gSensorData.scaled_acc_x = (gSensorData.raw_acc_x / 4096.0f);
		gSensorData.scaled_acc_y = (gSensorData.raw_acc_y / 4096.0f);
		gSensorData.scaled_acc_z = (gSensorData.raw_acc_z / 4096.0f);

		gSensorData.scaled_gyr_x = ((gSensorData.raw_gyr_x / 16.4f) * PI/180);
		gSensorData.scaled_gyr_y = ((gSensorData.raw_gyr_y / 16.4f) * PI/180);
		gSensorData.scaled_gyr_z = ((gSensorData.raw_gyr_z / 16.4f) * PI/180);

		gSensorData.scaled_mag_x = gSensorData.raw_mag_x;
		gSensorData.scaled_mag_y = gSensorData.raw_mag_y;
		gSensorData.scaled_mag_z = gSensorData.raw_mag_z;

		current_timer = TIM_GetCounter(TIM2);
		gStateData.estFrequency = 1.0f / ((current_timer - last_timer) / 1e6f);
		last_timer = current_timer;

		MadgwickAHRSupdate(&gStateData, &gSensorData);
		compute_euler_angles( &gStateData );
		gStateData.estRuntime = TIM_GetCounter(TIM2) - last_timer;

		chEvtBroadcastFlags(&eventEKFDone, EVT_EKF_DONE);
	}
	return 0;
}

void startEstimation(void) {
	chThdCreateStatic(PollAttitudeThreadWA,
			sizeof(PollAttitudeThreadWA),
			NORMALPRIO+3,
			PollAttitudeThread,
			NULL);
}
