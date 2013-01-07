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

#ifndef _ESTIMATION__
#define _ESTIMATION__

#include "ch.h"

#include "../sensors/sensors.h"

#define betaDef		0.1f		// 2 * proportional gain

typedef struct _quat {
	 float a,b,c,d;
} quat;

typedef struct __AHRS_state_data {
	// YAW/PITCH/ROLL
	float roll;
	float pitch;
	float yaw;

	// Euler Angles
	float phi;
	float theta;
	float psi;

	float roll_rate;
	float pitch_rate;
	float yaw_rate;

	// Quaternion states "qib" = Quaternion from Inertial to Body
	quat qib;

	float beta;	// 2 * proportional gain (Kp)

	// Frequency Estimation
	float estFrequency;

	uint32_t estRuntime;
} AHRS_state_data;

extern AHRS_state_data gStateData;

void MadgwickAHRSupdate(AHRS_state_data* estimated_states, sensorData* sensor_data);
void MadgwickAHRSupdateIMU(AHRS_state_data* estimated_states, sensorData* sensor_data);

void compute_euler_angles( AHRS_state_data* estimated_states );
void compute_yaw_pitch_roll( AHRS_state_data* estimated_states );

float invSqrt(float x);
float safe_asin(float v);

void startEstimation(void);

#endif
