/* ------------------------------------------------------------------------------
  File: estimation.h
  Author: CH Robotics
  Version: 1.0

  Description: Function declarations for CHR-6dm state estimation.
------------------------------------------------------------------------------ */

#ifndef _ESTIMATION__
#define _ESTIMATION__

#include "ch.h"

#include "quat.h"
#include "matrix.h"
#include "../sensors/sensors.h"

#define betaDef		0.1f		// 2 * proportional gain

// Structure for storing AHRS states and other data related to state computation
// This structure is, in a way, redundant because all this data is also stored in the
// UM6_config or UM6_data structures.  However, in the config and data strucutres, the
// data is packaged as UInt32 entries into an array for convenience with communication.
// To use the data as floats, special formatting is required.  This structure provides
// a place to store that data in the expected format, which makes accessing it easier.
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
} AHRS_state_data;

extern AHRS_state_data gStateData;

void MadgwickAHRSupdate(AHRS_state_data* estimated_states, sensorData* sensor_data);
void MadgwickAHRSupdateIMU(AHRS_state_data* estimated_states, sensorData* sensor_data);

void compute_euler_angles( AHRS_state_data* estimated_states );
void compute_yaw_pitch_roll( AHRS_state_data* estimated_states );

float invSqrt(float x);

void startEstimation(void);

#endif
