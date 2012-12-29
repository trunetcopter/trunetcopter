/* ------------------------------------------------------------------------------
  File: estimation.c
  Author: CH Robotics
  Version: 1.0

  Description: Function definitions for CHR-6dm state estimation.
------------------------------------------------------------------------------ */

#include <math.h>
#include "estimation.h"
#include "matrix.h"
#include "quat.h"

#include "stm32f4xx_tim.h"

#include "../util.h"

// Data structures for holding sensor data and estimated states.
RawSensorData gSensorData;
AHRS_state_data gStateData;

uint8_t gEKF_mode;

/*******************************************************************************
* Function Name  : EKF_Init
* Input          : None
* Output         : Pre-initialized state estimate structure
* Return         : None
* Description    : Fills an AHRS_states structure with zero initial values.
*******************************************************************************/
void EKF_Init( AHRS_state_data* estimated_states )
{
	 //int i, j;

	 estimated_states->phi = 0;
	 estimated_states->theta = 0;
	 estimated_states->psi = 0;

	 estimated_states->phi_dot = 0;
	 estimated_states->theta_dot = 0;
	 estimated_states->psi_dot = 0;

	 if( gEKF_mode == EKF_MODE_EULER )
	 {
		  mat_zero( &estimated_states->R, 3, 3 );
		  mat_zero( &estimated_states->Sigma, 3, 3 );

		  estimated_states->Sigma.data[0][0] = estimated_states->process_var;
		  estimated_states->Sigma.data[1][1] = estimated_states->process_var;
		  estimated_states->Sigma.data[2][2] = estimated_states->process_var;
		  estimated_states->Sigma.data[3][3] = estimated_states->process_var;

		  estimated_states->R.data[0][0] = estimated_states->process_var;
		  estimated_states->R.data[1][1] = estimated_states->process_var;
		  estimated_states->R.data[2][2] = estimated_states->process_var;
		  estimated_states->R.data[3][3] = estimated_states->process_var;
	 }
	 else
	 {
		  mat_zero( &estimated_states->R, 4, 4 );
		  mat_zero( &estimated_states->Sigma, 4, 4 );

		  // Process variance is scaled here so that the performance in Euler Angle mode and Quaternion mode is comparable
		  estimated_states->Sigma.data[0][0] = estimated_states->process_var*0.00001;
		  estimated_states->Sigma.data[1][1] = estimated_states->process_var*0.00001;
		  estimated_states->Sigma.data[2][2] = estimated_states->process_var*0.00001;
		  estimated_states->Sigma.data[3][3] = estimated_states->process_var*0.00001;

		  estimated_states->R.data[0][0] = estimated_states->process_var*0.00001;
		  estimated_states->R.data[1][1] = estimated_states->process_var*0.00001;
		  estimated_states->R.data[2][2] = estimated_states->process_var*0.00001;
		  estimated_states->R.data[3][3] = estimated_states->process_var*0.00001;
	 }

	 estimated_states->qib.a = 1;
	 estimated_states->qib.b = 0;
	 estimated_states->qib.c = 0;
	 estimated_states->qib.d = 0;

	 //CopyConfigArrayToStates();
}

/*******************************************************************************
* Function Name  : EKF_InitFromSensors
* Input          : AHRS_states* estimated_states, RawSensorData* sensor_data
* Output         : None
* Return         : None
* Description    : Uses sensor data to fill initial state estimate structure.
						 Specifically, accels are used to calculate an initial pitch and
						 roll guess, while the magnetomer is used to compute an initial
						 heading guess.
						 Gyro outputs are copied directly into estimated angular rate states.
*******************************************************************************/
void EKF_InitFromSensors( AHRS_state_data* estimated_states, RawSensorData* sensor_data )
{
	 //int i, j;
	(void)sensor_data;

	 // TODO: Write this function.  For now, initialization just calls EKF_Init(.)
	 EKF_Init( estimated_states );
}


/*******************************************************************************
* Function Name  : EKF_EstimateStates
* Input          : AHRS_states* estimated_states, RawSensorData* sensor_data
* Output         : None
* Return         : None
* Description    :
*******************************************************************************/
void EKF_EstimateStates( AHRS_state_data* state_data, RawSensorData* sensor_data )//, uint16_t timer_value )
{
	 static int initialized = 0;
	 //fConvert f2int;
	 //fConvert scale;

	 if( !initialized )
	 {
		  EKF_Init( state_data );
		  initialized = 1;
	 }

	 // First, convert raw sensor data to actual data (acceleration to gravities, gyro data
	 // to angular rates, magnetometer to unit-norm data
	 ConvertRawSensorData( state_data, sensor_data );

	 // Run EKF prediction step
	 EKF_Predict( state_data, sensor_data );//, timer_value );

	 // Run EKF update step
	 EKF_Update( state_data, sensor_data );

	 // Copy the new states into the communication interface structures
	 //CopyStatesToDataArray();
}


/*******************************************************************************
* Function Name  : ConvertRawSensorData
* Input          : RawSensorData* sensor_data
* Output         : AHRS_state_data* estimated_states
* Return         : None
* Description    :

Converts the raw sensor data in sensor_data to actual data (angular rates,
acceleration in gravities, etc. and stoes in state_data.  Also performs
calibration functions.

*******************************************************************************/
void ConvertRawSensorData( AHRS_state_data* state_data, RawSensorData* sensor_data )
{
	 fMatrix svec;
	 svec.rows = 3;
	 svec.columns = 1;

    // Convert temperature data
    float temp = sensor_data->temperature*0.00357143 + 70.00;
    float temp2 = temp*temp;
    float temp3 = temp2*temp;

    state_data->temperature = temp;

    // Rate gyros
    svec.data[0][0] = (float)((float)sensor_data->gyro_x - (float)state_data->beta_p - (state_data->beta_p0 + state_data->beta_p1*temp + state_data->beta_p2*temp2 + state_data->beta_p3*temp3) );
    svec.data[1][0] = (float)((float)sensor_data->gyro_y - (float)state_data->beta_q - (state_data->beta_q0 + state_data->beta_q1*temp + state_data->beta_q2*temp2 + state_data->beta_q3*temp3) );
    svec.data[2][0] = (float)((float)sensor_data->gyro_z - (float)state_data->beta_r - (state_data->beta_r0 + state_data->beta_r1*temp + state_data->beta_r2*temp2 + state_data->beta_r3*temp3) );

	 // Multiply gyro measurements by alignment matrix (fixes cross-axis alignment)
	 mat_mult( &state_data->gyro_cal, &svec, &svec );

	 // Copy new gyro data to state_data structure
	 state_data->gyro_x = svec.data[0][0];
	 state_data->gyro_y = svec.data[1][0];
	 state_data->gyro_z = svec.data[2][0];

	 // Now for accelerometers
	 svec.data[0][0] = ((float)(sensor_data->accel_x - state_data->beta_acc_x));
	 svec.data[1][0] = ((float)(sensor_data->accel_y - state_data->beta_acc_y));
	 svec.data[2][0] = ((float)(sensor_data->accel_z - state_data->beta_acc_z));

	 mat_mult( &state_data->accel_cal, &svec, &svec );

	 state_data->accel_x = svec.data[0][0];
	 state_data->accel_y = svec.data[1][0];
	 state_data->accel_z = svec.data[2][0];

	 // Now the magnetometer
	 svec.data[0][0] = ((float)(sensor_data->mag_x - state_data->beta_mag_x));
	 svec.data[1][0] = ((float)(sensor_data->mag_y - state_data->beta_mag_y));
	 svec.data[2][0] = ((float)(sensor_data->mag_z - state_data->beta_mag_z));

	 mat_mult( &state_data->mag_cal, &svec, &svec );

	 state_data->mag_x = svec.data[0][0];
	 state_data->mag_y = svec.data[1][0];
	 state_data->mag_z = svec.data[2][0];
}

/*******************************************************************************
* Function Name  : EKF_Predict
* Input          : AHRS_states* estimated_states, RawSensorData* sensor_data
* Output         : None
* Return         : None
* Description    : EKF prediction step.  Uses rate gyros to make new orientation
						 estimate.
*******************************************************************************/
void EKF_Predict( AHRS_state_data* estimated_states, RawSensorData* sensor_data )//, uint16_t timer_value )
{
	 float T,p,q,r;
	 uint16_t timer_value;
	 fMatrix A,Atranspose,temp1,temp2;
	 //fConvert i2f;

	 // Get elapsed time since last prediction (Timer3 should be configured
	 // to increment once every microsecond.  It is a 16-bit timer, which means
	 // that a maximum of 2^16 = 65536 microseconds can pass before overflow.
	 // The prediction step should thus be run at least once every 65 milliseconds (15.6 Hz),
	 // but preferably more quickly.  This shouldn't be a problem - the prediction step
	 // should nominally run at roughly 1000 Hz).
	 timer_value = TIM_GetCounter(TIM5);
	 TIM_SetCounter(TIM5,0);

	 T = (float)(0.000001)*(float)timer_value;

	 //i2f.float_val = T;

	 // Copy body frame angular rates to local variables for convenience
	 p = gStateData.gyro_x;
	 q = gStateData.gyro_y;
	 r = gStateData.gyro_z;

	 // Euler Angle Estimation
	 if( UM6_QUAT_ESTIMATE_ENABLED == 0 )
	 {
		  float cos_phi,sin_phi,cos_theta,tan_theta,sin_theta;

		  A.rows = 3;
		  A.columns = 3;
		  Atranspose.rows = 3;
		  Atranspose.columns = 3;
		  temp1.rows = 3;
		  temp1.columns = 3;
		  temp2.rows = 3;
		  temp2.columns = 3;

		  // Precompute trigonometric functions - these will be used more than once
		  cos_phi = cos(gStateData.phi*.01745329);
		  sin_phi = sin(gStateData.phi*.01745329);
		  cos_theta = cos(gStateData.theta*.01745329);
		  sin_theta = sin(gStateData.theta*.01745329);
		  tan_theta = tan(gStateData.theta*.01745329);

		  // Compute rotation rates based on body frame angular rates measured by the rate gyros
		  /*
		  phi_dot = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta)
		  theta_dot = q*cos(phi) - r*sin(phi)
		  psi_dot = (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta)
		  */
		  gStateData.phi_dot = p + r*cos_phi*tan_theta + q*sin_phi*tan_theta;
		  gStateData.theta_dot = q*cos_phi - r*sin_phi;
		  gStateData.psi_dot = (r*cos_phi)/cos_theta + (q*sin_phi)/cos_theta;

		  // Use measured rotation rates in the body frame to compute new angle estimates
		  gStateData.phi += T*gStateData.phi_dot;
		  gStateData.theta += T*gStateData.theta_dot;
		  gStateData.psi += T*gStateData.psi_dot;

		  // DISCRETE STATE TRANSITION
	 //	 [ T*q*cos(phi)*tan(theta) - T*r*sin(phi)*tan(theta) + 1,               T*r*cos(phi)*(tan(theta)^2 + 1) + T*q*sin(phi)*(tan(theta)^2 + 1), 0]
	 //	 [                         - T*q*sin(phi) - T*r*cos(phi),                                                                               1, 0]
	 //	 [ (T*q*cos(phi))/cos(theta) - (T*r*sin(phi))/cos(theta), (T*r*cos(phi)*sin(theta))/cos(theta)^2 + (T*q*sin(phi)*sin(theta))/cos(theta)^2, 1]
		  A.data[0][0] = T*(q*cos_phi*tan_theta - r*sin_phi*tan_theta) + 1;
		  A.data[0][1] = T*(r*cos_phi*(tan_theta*tan_theta + 1) + q*sin_phi*(tan_theta*tan_theta + 1));
		  A.data[0][2] = 0;
		  A.data[1][0] = T*(-r*cos_phi - q*sin_phi);
		  A.data[1][1] = 1;
		  A.data[1][2] = 0;
		  A.data[2][0] = T*((q*cos_phi)/cos_theta - (r*sin_phi)/cos_theta);
		  A.data[2][1] = T*((r*cos_phi*sin_theta)/(cos_theta*cos_theta) + (q*sin_phi*sin_theta)/(cos_theta*cos_theta));
		  A.data[2][2] = 1;

		  // Compute the new covariance estimate (discrete update: Sigma = A*Sigma*Atranspose + R
		  mat_transpose( &A, &Atranspose );
		  mat_mult( &A, &gStateData.Sigma, &temp1 );
		  mat_mult( &temp1, &Atranspose, &temp2 );
		  mat_add( &temp2, &gStateData.R, &gStateData.Sigma );

		  // Finally, "unroll" states so that they range from -360 to 360 degrees
		  unroll_states( &gStateData );
	 }
	 // Quaternion estimation
	 else
	 {
		  A.rows = 4;
		  A.columns = 4;
		  Atranspose.rows = 4;
		  Atranspose.columns = 4;
		  temp1.rows = 4;
		  temp1.columns = 4;
		  temp2.rows = 4;
		  temp2.columns = 4;

		  float a, b, c, d;

		  // Make local copies of the current attitude quaternion for convenience
		  a = gStateData.qib.a;
		  b = gStateData.qib.b;
		  c = gStateData.qib.c;
		  d = gStateData.qib.d;

		  // Convert p, q, and r to rad/s
		  p = p*3.14159/180;
		  q = q*3.14159/180;
		  r = r*3.14159/180;

		  // Create a quaternion to represent rotation rate
		  quat pqr_quat;
		  pqr_quat.a = 0;
		  pqr_quat.b = p;
		  pqr_quat.c = q;
		  pqr_quat.d = r;

		  // Predict new quaternion state based on gyro data
		  quat temp_quat;
		  quat_mult( &gStateData.qib, &pqr_quat, &temp_quat );
		  quat_scalar_mult( &temp_quat, 0.5*T, &temp_quat );
		  quat_add( &gStateData.qib, &temp_quat, &gStateData.qib );

		  // Normalize new predicted state
		  quat_norm( &gStateData.qib );

		  // PROPAGATE COVARIANCE
		  // Compute linearized state transition matrix
		  /*
				[       1, -(T*p)/2, -(T*q)/2, -(T*r)/2]
				[ (T*p)/2,        1,  (T*r)/2, -(T*q)/2]
				[ (T*q)/2, -(T*r)/2,        1,  (T*p)/2]
				[ (T*r)/2,  (T*q)/2, -(T*p)/2,        1]
		  */
		  A.data[0][0] = 1;
		  A.data[0][1] = -(T*p)/2;
		  A.data[0][2] = -(T*q)/2;
		  A.data[0][3] = -(T*r)/2;

		  A.data[1][0] = (T*p)/2;
		  A.data[1][1] = 1;
		  A.data[1][2] = (T*r)/2;
		  A.data[1][3] = -(T*q)/2;

		  A.data[2][0] = (T*q)/2;
		  A.data[2][1] = -(T*r)/2;
		  A.data[2][2] = 1;
		  A.data[2][3] = (T*p)/2;

		  A.data[3][0] = (T*r)/2;
		  A.data[3][1] = (T*q)/2;
		  A.data[3][2] = -(T*p)/2;
		  A.data[3][3] = 1;

		  // Compute the new covariance estimate (discrete update: Sigma = A*Sigma*Atranspose + R
		  mat_transpose( &A, &Atranspose );
		  mat_mult( &A, &gStateData.Sigma, &temp1 );
		  mat_mult( &temp1, &Atranspose, &temp2 );
		  mat_add( &temp2, &gStateData.R, &gStateData.Sigma );

		  // Now use the new quaternion to compute Euler Angles
		  compute_euler_angles( &gStateData );
	 }
}

/*******************************************************************************
* Function Name  : EKF_Update
* Input          : AHRS_states* estimated_states, RawSensorData* sensor_data
* Output         : None
* Return         : None
* Description    : EKF update step.  Uses accels to correct pitch and roll errors,
						 and magnetic sensors to correct yaw errors.  Compensation is
						 only applied when new data is available, as specified by the
						 new_mag_data and new_accel_data flags in the sensor_data structure.
*******************************************************************************/
void EKF_Update( AHRS_state_data* estimated_states, RawSensorData* sensor_data )
{

	 if( UM6_QUAT_ESTIMATE_ENABLED == 0 )
	 {

		  float cos_phi, cos_theta, cos_psi, sin_phi, sin_theta, sin_psi;

		  // If there is new accelerometer data available and accelerometer state updates are enabled,
		  // run an update using the new accelerometer sensor data
		  if( gSensorData.new_accel_data )
		  {
				float ax_hat, ay_hat, az_hat;
				float ax_ref, ay_ref, az_ref;

				fMatrix C;

				C.rows = 1;
				C.columns = 3;

				// Precompute trigonometric functions - these will be used more than once
				cos_phi = cos(gStateData.phi*.01745329);
				cos_theta = cos(gStateData.theta*.01745329);
				cos_psi = cos(gStateData.psi*.01745329);

				sin_phi = sin(gStateData.phi*.01745329);
				sin_theta = sin(gStateData.theta*.01745329);
				sin_psi = sin(gStateData.psi*.01745329);

				// Copy data into local variables for convenience
				ax_ref = gStateData.accel_ref_x;
				ay_ref = gStateData.accel_ref_y;
				az_ref = gStateData.accel_ref_z;

				// Compute expected accelerometer output based on current state estimates (the expected output is the accelerometer reference vector
				// rotated into the body-frame of the sensor
				ax_hat = ax_ref*cos_psi*cos_theta - az_ref*sin_theta + ay_ref*cos_theta*sin_psi;
				ay_hat = ay_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - ax_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) + az_ref*cos_theta*sin_phi;
				az_hat = ax_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - ay_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + az_ref*cos_phi*cos_theta;

				// Compute linearized state transition matrix for each axis independently
	 //		  [                                                                                                                                                  0,                          - az_ref*cos(theta) - ax_ref*cos(psi)*sin(theta) - ay_ref*sin(psi)*sin(theta),                                                                 ay_ref*cos(psi)*cos(theta) - ax_ref*cos(theta)*sin(psi)]
	 //		  [ ax_ref*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - ay_ref*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + az_ref*cos(phi)*cos(theta), ax_ref*cos(psi)*cos(theta)*sin(phi) - az_ref*sin(phi)*sin(theta) + ay_ref*cos(theta)*sin(phi)*sin(psi), - ax_ref*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - ay_ref*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))]
	 //		  [ ax_ref*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - ay_ref*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - az_ref*cos(theta)*sin(phi), ax_ref*cos(phi)*cos(psi)*cos(theta) - az_ref*cos(phi)*sin(theta) + ay_ref*cos(phi)*cos(theta)*sin(psi),   ax_ref*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + ay_ref*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))]

				// x-axis
				C.data[0][0] = 0;
				C.data[0][1] = -az_ref*cos_theta - ax_ref*cos_psi*sin_theta - ay_ref*sin_psi*sin_theta;
				C.data[0][2] = ay_ref*cos_psi*cos_theta - ax_ref*cos_theta*sin_psi;

				// Do correction
				EKF_Correction( &C, gStateData.accel_x, ax_hat, gStateData.accel_var, estimated_states, ACCEL_UPDATE );

				// y-axis
				C.data[0][0] = ax_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - ay_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + az_ref*cos_phi*cos_theta;
				C.data[0][1] = ax_ref*cos_psi*cos_theta*sin_phi - az_ref*sin_phi*sin_theta + ay_ref*cos_theta*sin_phi*sin_psi;
				C.data[0][2] = -ax_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - ay_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta);

				// Do correction
				EKF_Correction( &C, gStateData.accel_y, ay_hat, gStateData.accel_var, estimated_states, ACCEL_UPDATE );

				// z-axis
				C.data[0][0] = ax_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) - ay_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - az_ref*cos_theta*sin_phi;
				C.data[0][1] = ax_ref*cos_phi*cos_psi*cos_theta - az_ref*cos_phi*sin_theta + ay_ref*cos_phi*cos_theta*sin_psi;
				C.data[0][2] = ax_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + ay_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta);

				// Do correction
				EKF_Correction( &C, gStateData.accel_z, az_hat, gStateData.accel_var, estimated_states, ACCEL_UPDATE );

				// Finally, "unroll" states so that they range from -360 to 360 degrees
				unroll_states( &gStateData );
		  }

		  // Do magnetometer update if enabled and if there is new magnetic sensor data available
		  if( gSensorData.new_mag_data )
		  {
				float mx_hat, my_hat, mz_hat;
				float mx_ref, my_ref, mz_ref;
				float mx_v1, my_v1, mz_v1;

				fMatrix C;

				C.rows = 1;
				C.columns = 3;

				// Precompute trigonometric functions - these will be used more than once
				cos_phi = cos(gStateData.phi*.01745329);
				cos_theta = cos(gStateData.theta*.01745329);
				cos_psi = cos(gStateData.psi*.01745329);

				sin_phi = sin(gStateData.phi*.01745329);
				sin_theta = sin(gStateData.theta*.01745329);
				sin_psi = sin(gStateData.psi*.01745329);

				// Copy data into local variables for convenience
				mx_ref = gStateData.mag_ref_x;
				my_ref = gStateData.mag_ref_y;
				mz_ref = gStateData.mag_ref_z;

				// Rotate the sensor measurement into the vehicle-1 frame (undo pitch and roll)
				mx_v1 = gStateData.mag_x*cos_theta + gStateData.mag_z*cos_phi*sin_theta + gStateData.mag_y*sin_phi*sin_theta;
				my_v1 = gStateData.mag_y*cos_phi - gStateData.mag_z*sin_phi;
				mz_v1 =  gStateData.mag_z*cos_phi*cos_theta - gStateData.mag_x*sin_theta + gStateData.mag_y*cos_theta*sin_phi;

				// Compute expected magnetometer output based on current state estimates
				// (rotates the reference vector into the vehicle-1 frame)
				mx_hat = mx_ref*cos_psi + my_ref*sin_psi;
				my_hat = my_ref*cos_psi - mx_ref*sin_psi;
				mz_hat = mz_ref;

				// Compute linearized state transition matrix for each axis independently
				// x-axis
				C.data[0][0] = 0;
				C.data[0][1] = 0;
				C.data[0][2] = my_ref*cos_psi - mx_ref*sin_psi;

				// Do correction
				EKF_Correction( &C, mx_v1, mx_hat, gStateData.mag_var, estimated_states, MAG_UPDATE );

				// y-axis
				C.data[0][0] = 0;
				C.data[0][1] = 0;
				C.data[0][2] = -mx_ref*cos_psi - my_ref*sin_psi;

				// Do correction
				EKF_Correction( &C, my_v1, my_hat, gStateData.mag_var, estimated_states, MAG_UPDATE );

				/* z-axis doesn't do anything in the vehicle-1 frame
				// z-axis
				C.data[0][0] = 0;
				C.data[0][1] = 0;
				C.data[0][2] = 0;

				// Do correction
				EKF_Correction( &C, mz_v1, mz_hat, gStateData.mag_var, estimated_states, MAG_UPDATE );
				*/

				// Finally, "unroll" states so that they range from -360 to 360 degrees
				unroll_states( &gStateData );
		  }
	 }
	 // QUATERNION CODE
	 else
	 {
		  float a,b,c,d;

		  // Do accelerometer update if enabled and if there is new accelerometer sensor data available
		  if( gSensorData.new_accel_data )
		  {
				float sensor_norm = sqrt(gStateData.accel_x*gStateData.accel_x + gStateData.accel_y*gStateData.accel_y + gStateData.accel_z*gStateData.accel_z);
				fMatrix C;

				C.rows = 1;
				C.columns = 4;

				// Make sure this accelerometer measurement is close to 1 G.  If not, ignore it.
				if( abs(sensor_norm - 1) < 0.3 )
				{
					 float ax_hat, ay_hat, az_hat;
					 float ax_ref, ay_ref, az_ref;

					 // Make local copies of the current quaternion state estimate for convenience
					 a = gStateData.qib.a;
					 b = gStateData.qib.b;
					 c = gStateData.qib.c;
					 d = gStateData.qib.d;

					 // Make local copy of accelerometer reference vector for convenience
					 ax_ref = gStateData.accel_ref_x;
					 ay_ref = gStateData.accel_ref_y;
					 az_ref = gStateData.accel_ref_z;

					 // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
					 ax_hat = ay_ref*(2*a*d + 2*b*c) - az_ref*(2*a*c - 2*b*d) + ax_ref*(a*a + b*b - c*c - d*d);

					 // Create linearized update matrix for x-axis accelerometer
					 /* For all axes, C is given by:
					 [ 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d, 2*ay*b - 2*a*az - 2*ax*c, 2*a*ay + 2*az*b - 2*ax*d]
					 [ 2*a*ay + 2*az*b - 2*ax*d, 2*a*az - 2*ay*b + 2*ax*c, 2*ax*b + 2*ay*c + 2*az*d, 2*az*c - 2*a*ax - 2*ay*d]
					 [ 2*a*az - 2*ay*b + 2*ax*c, 2*ax*d - 2*az*b - 2*a*ay, 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d]
					 */
					 C.data[0][0] = 2*a*ax_ref - 2*az_ref*c + 2*ay_ref*d;
					 C.data[0][1] = 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d;
					 C.data[0][2] = 2*ay_ref*b - 2*a*az_ref - 2*ax_ref*c;
					 C.data[0][3] = 2*a*ay_ref + 2*az_ref*b - 2*ax_ref*d;

					 // Do correction
					 EKF_Correction( &C, gStateData.accel_x, ax_hat, gStateData.accel_var, &gStateData, ACCEL_UPDATE );

					 // REPEAT FOR Y-AXIS
					 // Make local copies of the current quaternion state estimate for convenience
					 a = gStateData.qib.a;
					 b = gStateData.qib.b;
					 c = gStateData.qib.c;
					 d = gStateData.qib.d;

					 // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
					 ay_hat = az_ref*(2*a*b + 2*c*d) - ax_ref*(2*a*d - 2*b*c) + ay_ref*(a*a - b*b + c*c - d*d);
					 az_hat = ax_ref*(2*a*c + 2*b*d) - ay_ref*(2*a*b - 2*c*d) + az_ref*(a*a - b*b - c*c + d*d);

					 C.data[0][0] = 2*a*ay_ref + 2*az_ref*b - 2*ax_ref*d;
					 C.data[0][1] = 2*a*az_ref - 2*ay_ref*b + 2*ax_ref*c;
					 C.data[0][2] = 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d;
					 C.data[0][3] = 2*az_ref*c - 2*a*ax_ref - 2*ay_ref*d;

					 // Do correction
					 EKF_Correction( &C, gStateData.accel_y, ay_hat, gStateData.accel_var, &gStateData, ACCEL_UPDATE );

					 // REPEAT FOR Z-AXIS
					 // Make local copies of the current quaternion state estimate for convenience
					 a = gStateData.qib.a;
					 b = gStateData.qib.b;
					 c = gStateData.qib.c;
					 d = gStateData.qib.d;

					 // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
					 az_hat = ax_ref*(2*a*c + 2*b*d) - ay_ref*(2*a*b - 2*c*d) + az_ref*(a*a - b*b - c*c + d*d);

					 C.data[0][0] = 2*a*az_ref - 2*ay_ref*b + 2*ax_ref*c;
					 C.data[0][1] = 2*ax_ref*d - 2*az_ref*b - 2*a*ay_ref;
					 C.data[0][2] = 2*a*ax_ref - 2*az_ref*c + 2*ay_ref*d;
					 C.data[0][3] = 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d;

					 // Do correction
					 EKF_Correction( &C, gStateData.accel_z, az_hat, gStateData.accel_var, &gStateData, ACCEL_UPDATE );
				}
		  }

		  // Do magnetometer update if enabled and if there is new magnetic sensor data available
		  if( gSensorData.new_mag_data )
		  {
				fMatrix C;

				C.rows = 1;
				C.columns = 4;

				float mx_hat, my_hat, mz_hat;
				float mx_ref, my_ref, mz_ref;

				// Make local copies of the current quaternion state estimate for convenience
				a = gStateData.qib.a;
				b = gStateData.qib.b;
				c = gStateData.qib.c;
				d = gStateData.qib.d;

				// Make local copy of accelerometer reference vector for convenience
				mx_ref = gStateData.mag_ref_x;
				my_ref = gStateData.mag_ref_y;
				mz_ref = gStateData.mag_ref_z;

				// Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
				mx_hat = my_ref*(2*a*d + 2*b*c) - mz_ref*(2*a*c - 2*b*d) + mx_ref*(a*a + b*b - c*c - d*d);

				// Create linearized update matrix for x-axis accelerometer
				/* For all axes, C is given by:
				[ 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d, 2*ay*b - 2*a*az - 2*ax*c, 2*a*ay + 2*az*b - 2*ax*d]
				[ 2*a*ay + 2*az*b - 2*ax*d, 2*a*az - 2*ay*b + 2*ax*c, 2*ax*b + 2*ay*c + 2*az*d, 2*az*c - 2*a*ax - 2*ay*d]
				[ 2*a*az - 2*ay*b + 2*ax*c, 2*ax*d - 2*az*b - 2*a*ay, 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d]
				*/
				C.data[0][0] = 2*a*mx_ref - 2*mz_ref*c + 2*my_ref*d;
				C.data[0][1] = 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d;
				C.data[0][2] = 2*my_ref*b - 2*a*mz_ref - 2*mx_ref*c;
				C.data[0][3] = 2*a*my_ref + 2*mz_ref*b - 2*mx_ref*d;

				// Do correction
				EKF_Correction( &C, gStateData.mag_x, mx_hat, gStateData.mag_var, &gStateData, MAG_UPDATE );

				// REPEAT FOR Y-AXIS
				// Make local copies of the current quaternion state estimate for convenience
				a = gStateData.qib.a;
				b = gStateData.qib.b;
				c = gStateData.qib.c;
				d = gStateData.qib.d;

				// Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
				my_hat = mz_ref*(2*a*b + 2*c*d) - mx_ref*(2*a*d - 2*b*c) + my_ref*(a*a - b*b + c*c - d*d);
				mz_hat = mx_ref*(2*a*c + 2*b*d) - my_ref*(2*a*b - 2*c*d) + mz_ref*(a*a - b*b - c*c + d*d);

				C.data[0][0] = 2*a*my_ref + 2*mz_ref*b - 2*mx_ref*d;
				C.data[0][1] = 2*a*mz_ref - 2*my_ref*b + 2*mx_ref*c;
				C.data[0][2] = 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d;
				C.data[0][3] = 2*mz_ref*c - 2*a*mx_ref - 2*my_ref*d;

				// Do correction
				EKF_Correction( &C, gStateData.mag_y, my_hat, gStateData.mag_var, &gStateData, MAG_UPDATE );

				// REPEAT FOR Z-AXIS
				// Make local copies of the current quaternion state estimate for convenience
				a = gStateData.qib.a;
				b = gStateData.qib.b;
				c = gStateData.qib.c;
				d = gStateData.qib.d;

				// Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
				mz_hat = mx_ref*(2*a*c + 2*b*d) - my_ref*(2*a*b - 2*c*d) + mz_ref*(a*a - b*b - c*c + d*d);

				C.data[0][0] = 2*a*mz_ref - 2*my_ref*b + 2*mx_ref*c;
				C.data[0][1] = 2*mx_ref*d - 2*mz_ref*b - 2*a*my_ref;
				C.data[0][2] = 2*a*mx_ref - 2*mz_ref*c + 2*my_ref*d;
				C.data[0][3] = 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d;

				// Do correction
				EKF_Correction( &C, gStateData.mag_z, mz_hat, gStateData.mag_var, &gStateData, MAG_UPDATE );
		  }
	 }
}

void EKF_Correction( fMatrix* C, float sensor_data, float sensor_hat, float sensor_covariance, AHRS_state_data* estimated_states, int sensor_type )
{
	 fMatrix Ctranspose,temp1,temp2,L;
	 float gain_scale, error;

	 //if( (gConfig.r[UM6_MISC_CONFIG] & UM6_QUAT_ESTIMATE_ENABLED) == 0 )
	 //{
	//	  mat_zero( &temp1, 3, 1 );
	//	  mat_zero( &temp2, 3, 3 );
	// }
	// else
	// {
		  mat_zero( &temp1, 4, 1 );
		  mat_zero( &temp2, 4, 4 );
	// }

	 mat_transpose( C, &Ctranspose );

	 // Compute Kalman Gain (L = Sigma*C'*(C*Sigma*C' + Q)^-1 )
	 mat_mult(&estimated_states->Sigma,&Ctranspose,&temp1);
	 mat_mult(C,&estimated_states->Sigma,&temp2);
	 mat_mult(&temp2,&Ctranspose,&temp2);
	 gain_scale = 1/(temp2.data[0][0] + sensor_covariance);
	 mat_scalar_mult(gain_scale,&temp1,&L);

	 // Update state estimates
	 error = sensor_data - sensor_hat;

	 //if( (gConfig.r[UM6_MISC_CONFIG] & UM6_QUAT_ESTIMATE_ENABLED) == 0 )
	 //{
	//	  estimated_states->phi += L.data[0][0]*error;
	//	  estimated_states->theta += L.data[1][0]*error;
	//	  estimated_states->psi += L.data[2][0]*error;
//
	//	  mat_create_identity( &temp1, 3, 3 );
	 //}
	 //else
	 //{
		  estimated_states->qib.a += L.data[0][0]*error;
		  estimated_states->qib.b += L.data[1][0]*error;
		  estimated_states->qib.c += L.data[2][0]*error;
		  estimated_states->qib.d += L.data[3][0]*error;

		  mat_create_identity( &temp1, 4, 4 );
	 //}

	 // Now update the covariance estimate (Sigma = (I - L*C)*Sigma
	 mat_mult(&L,C,&temp2);
	 mat_scalar_mult(-1,&temp2,&temp2);
	 mat_add(&temp1,&temp2,&temp1);
	 mat_mult(&temp1,&estimated_states->Sigma, &estimated_states->Sigma);
}

/*******************************************************************************
* Function Name  : compute_euler_angles
* Input          : AHRS_states* states
* Output         : None
* Return         : None
* Description    : Converts quaternion attitude estimate to euler angles (yaw, pitch, roll)
*******************************************************************************/
void compute_euler_angles( AHRS_state_data* estimated_states )
{
	 float q0, q1, q2, q3;

	 q0 = estimated_states->qib.a;
	 q1 = estimated_states->qib.b;
	 q2 = estimated_states->qib.c;
	 q3 = estimated_states->qib.d;

	 estimated_states->phi = atan2(2*(q0*q1 + q2*q3),q3*q3 - q2*q2 - q1*q1 + q0*q0)*180/3.14159;
	 estimated_states->theta = -asin(2*(q1*q3 - q0*q2))*180/3.14159;
	 estimated_states->psi = atan2(2*(q0*q3+q1*q2),q1*q1 + q0*q0 - q3*q3 - q2*q2)*180/3.14159;
}

/*******************************************************************************
* Function Name  : unroll_states
* Input          : AHRS_states* states
* Output         : None
* Return         : None
* Description    : Keeps all angle estimates in the range of -360 to 360 degrees
*******************************************************************************/
void unroll_states( AHRS_state_data* states )
{
	 while( states->phi > 360 )
	 {
		  states->phi -= 360;
	 }
	 while( states->phi < -360 )
	 {
		  states->phi += 360;
	 }

	 while( states->theta > 360 )
	 {
		  states->theta -= 360;
	 }
	 while( states->theta < -360 )
	 {
		  states->theta += 360;
	 }

	 while( states->psi > 360 )
	 {
		  states->psi -= 360;
	 }
	 while( states->psi < -360 )
	 {
		  states->psi += 360;
	 }

}
