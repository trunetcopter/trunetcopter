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

void initSensors(void);
void startSensors(void);

#endif
