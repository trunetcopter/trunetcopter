/*
 * press_ms561101ba.h
 *
 *  Created on: Dec 29, 2012
 *      Author: trunet
 */

#ifndef PRESS_MS561101BA_H_
#define PRESS_MS561101BA_H_

#include "ch.h"

// The MS5611-01BA address is 111011Cx, where C
// is the complementary value of the pin CSB
#define MS561101BA_ADDRESS_CSB_LOW  0b1110111
#define MS561101BA_ADDRESS_CSB_HIGH 0b1110110
#define MS561101BA_DEFAULT_ADDRESS  0b1110110

// Register's address
#define MS561101BA_RA_D1     0x40
#define MS561101BA_RA_D2     0x50
#define MS561101BA_D1D2_SIZE 3 // (bytes)
#define MS561101BA_RA_RESET  0x1E

// PROM
#define MS561101BA_RA_PROM 0xA0
#define MS561101BA_PROM_NUM_REGISTERS 8
#define MS561101BA_PROM_BITS_PER_REGISTER 16

// Oversample rates
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

// ADC max conversion time (µs)
#define MS561101BA_MAX_CONVERSION_TIME_OSR_256   600
#define MS561101BA_MAX_CONVERSION_TIME_OSR_512  1170
#define MS561101BA_MAX_CONVERSION_TIME_OSR_1024 2280
#define MS561101BA_MAX_CONVERSION_TIME_OSR_2048 4540
#define MS561101BA_MAX_CONVERSION_TIME_OSR_4096 9040

static const uint8_t  POW_2_7  = 1 << 7;
static const uint16_t POW_2_8  = 1 << 8;
static const uint16_t POW_2_15 = 1 << 15;
static const uint32_t POW_2_21 = 1L << 21;
static const uint32_t POW_2_23 = 1L << 23;
static const uint32_t POW_2_31 = 1L << 31;

extern uint8_t defaultOsr;
extern uint16_t prom[MS561101BA_PROM_NUM_REGISTERS];

void ms561101ba_initialize(void);
bool_t ms561101ba_testConnection(void);
bool_t ms561101ba_reset(void);
bool_t ms561101ba_readPROM(void);
bool_t ms561101ba_setOverSampleRate(uint8_t osr);

bool_t ms561101ba_readValues(
		float * pressure,
		float * temperature,
		int8_t osr);

int32_t ms561101ba_readD1(int8_t osr);
int32_t ms561101ba_readD2(int8_t osr);

int32_t ms561101ba_readConversion(uint8_t regAddr, uint8_t osr);

uint16_t ms561101ba_getSENST1(void);
uint16_t ms561101ba_getOFFT1(void);
uint16_t ms561101ba_getTCS(void);
uint16_t ms561101ba_getTCO(void);
uint16_t ms561101ba_getTREF(void);
uint16_t ms561101ba_getTEMPSENS(void);

#endif /* PRESS_MS561101BA_H_ */
