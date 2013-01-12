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

#include "motors.h"
#include "../config.h"
#include "../util.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "comm.h"
#include "mavlink.h"

motorsStruct_t motorsData __attribute__((section(".ccm")));

// Plus configuration
const motorsPowerStruct_t motorsDefaultPlus[] = {//  MOTOR PORT
    { 100.0,  100.0,    0.0, -100.0},	//  1
    { 100.0,    0.0,  100.0,  100.0},	//  2
    //{   0.0,    0.0,    0.0,    0.0},	//  3
    //{   0.0,    0.0,    0.0,    0.0},	//  4
    //{   0.0,    0.0,    0.0,    0.0},	//  5
    //{   0.0,    0.0,    0.0,    0.0},	//  6
    {   0.0,    0.0,    0.0,    0.0},	//  7
    { 100.0, -100.0,    0.0, -100.0},	//  8
    { 100.0,    0.0, -100.0,  100.0},	//  9
    //{   0.0,    0.0,    0.0,    0.0},	//  10
    //{   0.0,    0.0,    0.0,    0.0},	//  11
    {   0.0,    0.0,    0.0,    0.0},	//  12
    {   0.0,    0.0,    0.0,    0.0},	//  13
    {   0.0,    0.0,    0.0,    0.0},	//  14
};

// X configuration
const motorsPowerStruct_t motorsDefaultX[] = {	//  MOTOR PORT
    {   0.0,    0.0,    0.0,    0.0},	//  1
    { 100.0,   50.0,   50.0,  100.0},	//  2
    //{   0.0,    0.0,    0.0,    0.0},	//  3
    //{   0.0,    0.0,    0.0,    0.0},	//  4
    { 100.0,  -50.0,   50.0, -100.0},	//  5
    {   0.0,    0.0,    0.0,    0.0},	//  6
    //{   0.0,    0.0,    0.0,    0.0},	//  7
    //{   0.0,    0.0,    0.0,    0.0},	//  8
    { 100.0,   50.0,  -50.0, -100.0},	//  9
    {   0.0,    0.0,    0.0,    0.0},	//  10
    //{   0.0,    0.0,    0.0,    0.0},	//  11
    //{   0.0,    0.0,    0.0,    0.0},	//  12
    { 100.0,  -50.0,  -50.0,  100.0},	//  13
    {   0.0,    0.0,    0.0,    0.0},	//  14
};

// Hex configuration
const motorsPowerStruct_t motorsDefaultHex[] = {//  MOTOR PORT
    //{   0.0,    0.0,    0.0,    0.0},	//  1
    { 100.0, -100.0,  100.0, -100.0},	//  2
    //{   0.0,    0.0,    0.0,    0.0},	//  3
    { 100.0,  -75.0,    0.0,  100.0},	//  4
    { 100.0,  100.0,  100.0,  100.0},	//  5
    //{   0.0,    0.0,    0.0,    0.0},	//  6
    //{   0.0,    0.0,    0.0,    0.0},	//  7
    {   0.0,    0.0,    0.0,    0.0},	//  8
    { 100.0,  +75.0,    0.0, -100.0},	//  9
    {   0.0,    0.0,    0.0,    0.0},	//  10
    //{   0.0,    0.0,    0.0,    0.0},	//  11
    { 100.0, -100.0, -100.0, -100.0},	//  12
    { 100.0,  100.0, -100.0,  100.0},	//  13
    //{   0.0,    0.0,    0.0,    0.0},	//  14
};

void motorsSendValues(void) {
    int i;

    for (i = 0; i < MOTORS_NUM; i++)
	if (motorsData.active[i]) {
	    // ensure motor output is constrained
		motorsData.value[i] = constrainInt(motorsData.value[i], p[MOT_START], p[MOT_MAX]);
		//*motorsData.pwm[i]->ccr = motorsData.value[i];
		pwmEnableChannel(motorsData.pwm[i], motorsData.channel[i], motorsData.value[i]);
	}
}

void motorsOff(void) {
    int i;

    for (i = 0; i < MOTORS_NUM; i++)
	if (motorsData.active[i]) {
	    motorsData.value[i] = p[MOT_MIN];
	    //*motorsData.pwm[i]->ccr = motorsData.value[i];
	    pwmEnableChannel(motorsData.pwm[i], motorsData.channel[i], motorsData.value[i]);
    }

    motorsData.throttle = 0;
    motorsData.throttleLimiter = 0.0f;
}

void motorsCommands(float throtCommand, float pitchCommand, float rollCommand, float ruddCommand) {
    float throttle;
    float expFactor;
    //float voltageFactor;
    float value;
    //float nominalBatVolts;
    int i;

    // throttle limiter to prevent control saturation
    throttle = constrainFloat(throtCommand - motorsData.throttleLimiter, 0.0f, p[MOT_MAX]);

    // scale commands based on throttle setting
    expFactor = constrainFloat((p[MOT_HOV_THROT] - throttle) * p[MOT_EXP_FACT], p[MOT_EXP_MIN], p[MOT_EXP_MAX]);
    pitchCommand += (pitchCommand * expFactor);
    rollCommand += (rollCommand * expFactor);
    ruddCommand += (ruddCommand * expFactor);

    // calculate voltage factor
    //nominalBatVolts = MOTORS_CELL_VOLTS*adcData.batCellCount;
    //voltageFactor = 1.0f + (nominalBatVolts - adcData.vIn) / nominalBatVolts;

    // calculate and set each motor value
    for (i = 0; i < MOTORS_NUM; i++) {
	if (motorsData.active[i]) {
	    motorsPowerStruct_t *d = &motorsData.distribution[i];

	    value = 0.0f;
	    value += (throttle * d->throttle * 0.01f);
	    value += (pitchCommand * d->pitch * 0.01f);
	    value += (rollCommand * d->roll * 0.01f);
	    value += (ruddCommand * d->yaw * 0.01f);

	    //motorsData.value[i] = value*voltageFactor + p[MOT_START];
	    pwmEnableChannel(motorsData.pwm[i], motorsData.channel[i], value + p[MOT_START]);
	    motorsData.value[i] = value + p[MOT_START];

	    // check for over throttle
	    if (motorsData.value[i] == p[MOT_MAX])
	    	motorsData.throttleLimiter += MOTORS_THROTTLE_LIMITER;
	}
    }

    motorsSendValues();

    // decay throttle limit
    motorsData.throttleLimiter = constrainFloat(motorsData.throttleLimiter - MOTORS_THROTTLE_LIMITER, 0.0f, p[MOT_MAX]/2);

    motorsData.pitch = pitchCommand;
    motorsData.roll = rollCommand;
    motorsData.yaw = ruddCommand;
    motorsData.throttle = throttle;
}

static PWMConfig pwmcfg2 = {1000000,
							2500, // 400 Hz
							NULL,
                            { {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH1 - GPIOA 0
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH2 - GPIOA 1
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH3 - GPIOB 10
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL} }, //CH4 - GPIOB 11
                            0
                          };
static PWMConfig pwmcfg3 = {1000000,
							2500, // 400 Hz
							NULL,
                            { {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH1 - GPIOC 6
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH2 - GPIOC 7
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL},   //CH3 - GPIOC 8
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL} }, //CH4 - GPIOC 9
                            0
                          };

void motorsInit(void) {
    float sumPitch, sumRoll, sumYaw;
    int i;

    mavlinkNotice(MAV_SEVERITY_INFO, "Motors Initialized!");

    memset((void *)&motorsData, 0, sizeof(motorsData));

    switch ((int)p[MOT_FRAME]) {
		// custom
		case 0:
			motorsData.distribution = (motorsPowerStruct_t *)&p[MOT_PWRD_01_T];
			break;
		case 1:
			motorsData.distribution = (motorsPowerStruct_t *)motorsDefaultPlus;
			break;
		case 2:
			motorsData.distribution = (motorsPowerStruct_t *)motorsDefaultX;
			break;
		case 3:
			motorsData.distribution = (motorsPowerStruct_t *)motorsDefaultHex;
			break;
    }

    sumPitch = 0.0f;
    sumRoll = 0.0f;
    sumYaw = 0.0f;

    pwmInit();
	pwmObjectInit(&PWMD2);
	pwmStart(&PWMD2, &pwmcfg2);
	pwmObjectInit(&PWMD3);
	pwmStart(&PWMD3, &pwmcfg3);

    for (i = 0; i < MOTORS_NUM; i++) {
		motorsPowerStruct_t *d = &motorsData.distribution[i];

		if (d->throttle != 0.0 || d->pitch != 0.0 || d->roll != 0.0 || d->yaw != 0.0) {

			if (i < 4) {
				motorsData.pwm[i] = &PWMD2;
				motorsData.channel[i] = i;
				pwmEnableChannel(motorsData.pwm[i], motorsData.channel[i], p[MOT_START]);
			} else {
				motorsData.pwm[i] = &PWMD3;
				motorsData.channel[i] = i - 4;
				pwmEnableChannel(motorsData.pwm[i], motorsData.channel[i], p[MOT_START]);
			}
			motorsData.active[i] = 1;

			sumPitch += d->pitch;
			sumRoll += d->roll;
			sumYaw += d->yaw;
		}
    }

    if (fabsf(sumPitch) > 0.01f)
    	mavlinkNotice(MAV_SEVERITY_WARNING, "Motors: Warning pitch control imbalance");

    if (fabsf(sumRoll) > 0.01f)
    	mavlinkNotice(MAV_SEVERITY_WARNING, "Motors: Warning roll control imbalance");

    if (fabsf(sumYaw) > 0.01f)
    	mavlinkNotice(MAV_SEVERITY_WARNING, "Motors: Warning yaw control imbalance");

    chThdSleepMilliseconds(100);

    motorsOff();
}

