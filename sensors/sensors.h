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

#ifndef SENSORS_H_
#define SENSORS_H_

#define EVT_IMU_IRQ		1
#define EVT_MAGN_IRQ	2
#define EVT_IMU_READ	4
#define EVT_MAGN_READ	8
#define EVT_EKF_DONE	16

extern EventSource eventImuIrq;
extern EventSource eventMagnIrq;
extern EventSource eventImuRead;
extern EventSource eventMagnRead;
extern EventSource eventEKFDone;

typedef struct __sensorData {
	int16_t raw_acc_x;
	int16_t raw_acc_y;
	int16_t raw_acc_z;
	int16_t raw_gyr_x;
	int16_t raw_gyr_y;
	int16_t raw_gyr_z;
	int16_t raw_mag_x;
	int16_t raw_mag_y;
	int16_t raw_mag_z;

	float scaled_acc_x; // mg
	float scaled_acc_y; // mg
	float scaled_acc_z; // mg
	float scaled_gyr_x; // millirad / sec
	float scaled_gyr_y; // millirad / sec
	float scaled_gyr_z; // millirad / sec
	float scaled_mag_x; // gauss
	float scaled_mag_y; // gauss
	float scaled_mag_z; // gauss

    // Rate gyro temperature measurement
    float imuTemperature;

    // Gyro Temperature Compensation Bias
    float gyroTCBias[3];
    float gyroTCBiasSlope[3]; // Calculated on long time calibration step
    float gyroTCBiasIntercept[3]; // Calculated on long time calibration step
    // Gyro Runtime Bias
    float gyroRTBias[3];

    // Barometer data
    float barTemperature;
    float barPressure;
    float barAltitude;

    // Reading frequency from sensors
    float imuFrequency;
    float magFrequency;
    float barFrequency;

    // GPS Data
	int32_t longitude;   // longitude*1e6
	int32_t latitude;   // latitude*1e6
	float altitude; // altitude in meters
	float speed; // speed in km/h
	float heading; // heading
	uint8_t satellites; // number of satellites
	uint8_t fix; // fix type
	uint32_t utc_date;
	uint32_t utc_time;
	uint32_t time; // milliseconds from epoch
	uint16_t hdop; //Horizontal Dilution of Precision

	float v_lon;
	float v_lat;

	char  valid;
} sensorData;

extern sensorData gSensorData;

void initSensors(void);
void startSensors(void);

void fullCalibrateGyro(void);
void computeGyroTCBias(void);
void computeGyroRTBias(void);

#endif
