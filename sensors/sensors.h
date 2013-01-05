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

    // Barometer data
    float barTemperature;
    float barPressure;
    float barAltitude;

    // Reading frequency from sensors
    float imuFrequency;
    float magFrequency;
    float barFrequency;
} sensorData;

extern sensorData gSensorData;

void initSensors(void);
void startSensors(void);

#endif
