/* ------------------------------------------------------------------------------
  File: estimation.h
  Author: CH Robotics
  Version: 1.0

  Description: Function declarations for CHR-6dm state estimation.
------------------------------------------------------------------------------ */

#ifndef _ESTIMATION__
#define _ESTIMATION__

#include "quat.h"
#include "matrix.h"

#include "ch.h"

#define UM6_QUAT_ESTIMATE_ENABLED 1

// Structure for holding raw sensor data
typedef struct __RawSensorData {
	 int16_t gyro_x;
	 int16_t gyro_y;
	 int16_t gyro_z;

	 int16_t new_gyro_data;

	 int16_t accel_x;
	 int16_t accel_y;
	 int16_t accel_z;

	 // Flag specifies whether there is new accel data in the sensor data structure
	 int16_t new_accel_data;

	 int16_t mag_x;
	 int16_t mag_y;
	 int16_t mag_z;

	 // Flag specifies whether there is new magnetometer data in the sensor data structure
	 int16_t new_mag_data;

      // Rate gyro temperature measurement
      int16_t temperature;

	  float GPS_latitude;
	  float GPS_longitude;
	  float GPS_altitude;
	  float GPS_course;
	  float GPS_speed;
	  uint8_t GPS_mode;
	  uint8_t GPS_satellite_count;
	  float GPS_Hdop;
	  float GPS_Vdop;

	  uint8_t GPS_SatID_1;
	  uint8_t GPS_SatID_2;
	  uint8_t GPS_SatID_3;
	  uint8_t GPS_SatID_4;
	  uint8_t GPS_SatID_5;
	  uint8_t GPS_SatID_6;
	  uint8_t GPS_SatID_7;
	  uint8_t GPS_SatID_8;
	  uint8_t GPS_SatID_9;
	  uint8_t GPS_SatID_10;
	  uint8_t GPS_SatID_11;
	  uint8_t GPS_SatID_12;

	  uint8_t GPS_SatSNR_1;
	  uint8_t GPS_SatSNR_2;
	  uint8_t GPS_SatSNR_3;
	  uint8_t GPS_SatSNR_4;
	  uint8_t GPS_SatSNR_5;
	  uint8_t GPS_SatSNR_6;
	  uint8_t GPS_SatSNR_7;
	  uint8_t GPS_SatSNR_8;
	  uint8_t GPS_SatSNR_9;
	  uint8_t GPS_SatSNR_10;
	  uint8_t GPS_SatSNR_11;
	  uint8_t GPS_SatSNR_12;


	  uint8_t new_GPS_position;
	  uint8_t new_GPS_course_speed;
	  uint8_t new_GPS_satellite_data;
	  uint8_t new_GPS_satellite_summary;

	  uint8_t new_GPS_data;

} RawSensorData;


// Structure for storing AHRS states and other data related to state computation
// This structure is, in a way, redundant because all this data is also stored in the
// UM6_config or UM6_data structures.  However, in the config and data strucutres, the
// data is packaged as UInt32 entries into an array for convenience with communication.
// To use the data as floats, special formatting is required.  This structure provides
// a place to store that data in the expected format, which makes accessing it easier.
typedef struct __AHRS_state_data {

	 // Orientation states
	 union {
		  float heading;
		  float yaw;
		  float psi;
	 };
	 union {
		  float pitch;
		  float theta;
	 };
	 union {
		  float roll;
		  float phi;
	 };

	 // Orientation rate states
	 union {
		  float heading_rate;
		  float yaw_rate;
		  float psi_dot;
	 };

	 union {
		  float pitch_rate;
		  float theta_dot;
	 };

	 union {
		  float roll_rate;
		  float phi_dot;
	 };

	 // Quaternion states "qib" = Quaternion from Inertial to Body
	 quat qib;

	 // Gyro biases
	 int16_t beta_p,beta_q,beta_r;

      // Gyro temperature compensation terms
      float beta_p0, beta_p1, beta_p2, beta_p3;
      float beta_q0, beta_q1, beta_q2, beta_q3;
      float beta_r0, beta_r1, beta_r2, beta_r3;

	 // Accelerometer biases
	 int16_t beta_acc_x, beta_acc_y, beta_acc_z;

	 // Magnetometer biases
	 int16_t beta_mag_x, beta_mag_y, beta_mag_z;

	 // Process noise matrix
	 fMatrix R;

	 // Accelerometer alignment matrix
	 fMatrix accel_cal;

	 // Gyro alignment matrix
	 fMatrix gyro_cal;

	 // Magnetometer calibration matrix
	 fMatrix mag_cal;

	 // EKF covariance
	 fMatrix Sigma;

	 // Magnetic field reference vector
	 float mag_ref_x;
	 float mag_ref_y;
	 float mag_ref_z;

	 // Accelerometer	reference vector
	 float accel_ref_x;
	 float accel_ref_y;
	 float accel_ref_z;

	 // accelerometer measurement variance
	 float accel_var;

	 // Magnetometer variance
	 float mag_var;

	 // Process variance
	 float process_var;

	 // Entries for storing processed sensor data
	 float gyro_x;
	 float gyro_y;
	 float gyro_z;

	 float accel_x;
	 float accel_y;
	 float accel_z;

	 float mag_x;
	 float mag_y;
	 float mag_z;

      float temperature;

	  // GPS stuff
	  float GPS_north;		// In meters
	  float GPS_east;
	  float GPS_h;
	  float GPS_speed;		// In m/s

	  float GPS_lat_home;
	  float GPS_lon_home;
	  float GPS_alt_home;
} AHRS_state_data;

extern RawSensorData gSensorData;
extern AHRS_state_data gStateData;

extern uint8_t gEKF_mode;

// Function declarations
void EKF_Init( AHRS_state_data* estimated_states );
void EKF_InitFromSensors( AHRS_state_data* estimated_states, RawSensorData* sensor_data );
void EKF_EstimateStates( AHRS_state_data* estimated_states, RawSensorData* sensor_data);//, uint16_t timer_value );
void EKF_Predict( AHRS_state_data* estimated_states, RawSensorData* sensor_data);//, uint16_t timer_value );
void EKF_Update( AHRS_state_data* estimated_states, RawSensorData* sensor_data );
void EKF_Correction( fMatrix* C, float sensor_data, float sensor_hat, float sensor_covariance, AHRS_state_data* estimated_states, int sensor_type );

void ConvertRawSensorData( AHRS_state_data* estimated_states, RawSensorData* sensor_data );

#define		MAG_UPDATE			0
#define		ACCEL_UPDATE		1

// EKF "mode" stored in the global variable gEKF_mode
#define		EKF_MODE_QUAT		0
#define		EKF_MODE_EULER		1


void compute_euler_angles( AHRS_state_data* estimated_states );

void unroll_states( AHRS_state_data* states );

#endif
