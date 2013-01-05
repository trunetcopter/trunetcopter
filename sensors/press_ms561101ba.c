#include "press_ms561101ba.h"
#include "../i2c_local.h"

#include <math.h>

uint8_t defaultOsr;
uint16_t prom[MS561101BA_PROM_NUM_REGISTERS];

/** Power on and prepare for general usage.
 */
void ms561101ba_initialize(void) {
	// Reset the device
	ms561101ba_reset();
	// Wait for it populates its internal PROM registers
	chThdSleepMilliseconds(250);
	// Read PROM registers
	ms561101ba_readPROM();
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool_t ms561101ba_testConnection(void) {
	uint16_t aux;
	return i2c_readWord(MS561101BA_DEFAULT_ADDRESS, MS561101BA_RA_PROM, &aux);
}

/** Send a reset command.
 * The Reset sequence shall be sent once after power-on to make
 * sure that the calibration PROM gets loaded into the internal
 * register. It can be also used to reset the device ROM from
 * an unknown condition.
 * @return Status of operation (true = success)
 */
bool_t ms561101ba_reset(void) {
	return i2c_writeBytes(MS561101BA_DEFAULT_ADDRESS, MS561101BA_RA_RESET, 0, NULL);
}

/**
 * Read read the content of the calibration PROM.
 * The read command for PROM shall be executed once after reset
 * by the user to calculate the calibration coefficients.
 * @return Status of operation (true = success)
 */
bool_t ms561101ba_readPROM(void) {
	bool_t success = 1;
	uint8_t add;
	for (add = 0; success && add < MS561101BA_PROM_NUM_REGISTERS; add++) {
		uint8_t regAddr = MS561101BA_RA_PROM + (add << 1);
		success = i2c_readWord(MS561101BA_DEFAULT_ADDRESS, regAddr, &prom[add]);
		if (success == 0)
			success = 1;
	}
	return success;
}

/**
 * Check PROM data with CRC register
 * @return Status of operation (false = failure)
 */
bool_t ms561101ba_crc(void) {
	int32_t i, j;
	uint32_t res = 0;
	uint8_t crc = prom[7] & 0xF;
	prom[7] &= 0xFF00;
	for (i = 0; i < 16; i++) {
		if (i & 1) res ^= ((prom[i>>1]) & 0x00FF);
		else res ^= (prom[i>>1]>>8);
		for (j = 8; j > 0; j--) {
			if (res & 0x8000) res ^= 0x1800;
			res <<= 1;
		}
	}
	prom[7] |= crc;
	if (crc == ((res >> 12) & 0xF)) return 1;
	else return -1;
}

/**
 * Set the default oversample rate (osr)
 * @param osr Oversample rate. Optional.
 * @return Status of operation (true = success)
 */
bool_t ms561101ba_setOverSampleRate(uint8_t osr) {
	if (osr == MS561101BA_OSR_256
		|| osr == MS561101BA_OSR_512
		|| osr == MS561101BA_OSR_1024
		|| osr == MS561101BA_OSR_2048
		|| osr == MS561101BA_OSR_4096) {
		defaultOsr = osr;
		return 1;
	} else {
		return 0;
	}
}

/**
 * Read the pressure and temperature.
 * This method use the second order temperature
 * compensation algorithm described in the datasheet.
 * @param pressure Pressure pointer. hPa
 * @param temperature Temperature pointer. ºC
 * @param osr Oversample rate. Optional.
 * @return Status of operation (true = success)
 */
bool_t ms561101ba_readValues(
		float * pressure,
		float * temperature,
		int8_t osr) {

	// Read the sensors
	int32_t d1 = ms561101ba_readD1(osr);
	int32_t d2 = ms561101ba_readD2(osr);

	// Check for errors
	if (d1 < 0 || d2 < 0) return 0;

	int32_t dT = d2 - (((uint32_t) ms561101ba_getTREF()) << 8);
	double t = 2000.0 + ((int64_t) dT) * ms561101ba_getTEMPSENS() / (1L << 23);

	int64_t off  = (((int64_t) ms561101ba_getOFFT1())  << 16) +
			((int64_t) dT) * ms561101ba_getTCO() / (1 << 7);
	int64_t sens = (((int64_t) ms561101ba_getSENST1()) << 15) +
			((int64_t) dT) * ms561101ba_getTCS() / (1 << 8);

	// Second order temperature compensation
	if (t < 2000) {
		double square = pow (dT,2);
		double t2 = square / (1L << 31);
		square = pow (t-2000,2);
		double off2  = square * 5 / 2;
		double sens2 = square * 5 / 4;
		if (t < 15) {
			square = pow(t+1500,2);
			off2  += square * 7;
			sens2 += square * 11 / 2;
		}

		t    -= t2;
		off  -= off2;
		sens -= sens2;

	}

	double p = ((sens * d1 / (1L << 21)) - off) / (1 << 15);

	*temperature = (float)t/100.0f;
	*pressure    = (float)p/100.0f;

	return 1;

}

float ms561101ba_getAltitude(float pressure, float temperature) {
	//float tmp_float;
	//float Altitude;

	//tmp_float = (pressure / 101325.0);
	//tmp_float = pow(tmp_float, 0.190295);
	//Altitude = 44330.0 * (1.0 - tmp_float);

	//return (Altitude);
	return ((pow((SEA_LEVEL_PRESSURE / pressure), 1/5.257) - 1.0) * (temperature + 273.15)) / 0.0065;
}

/**
 * Read the content of the D1 register (pressure).
 * @param osr Oversample rate. Optional.
 * @return content of the D1 register. -1 in case of error.
 */
int32_t ms561101ba_readD1(int8_t osr) {
	if (osr == -1) osr = defaultOsr;
	return ms561101ba_readConversion(MS561101BA_RA_D1, osr);
}

/**
 * Read the content of the D2 register (temperature).
 * @param osr Oversample rate. Optional.
 * @return content of the D1 register. -1 in case of error.
 */
int32_t ms561101ba_readD2(int8_t osr) {
	if (osr == -1) osr = defaultOsr;
	return ms561101ba_readConversion(MS561101BA_RA_D2, osr);
}

int32_t ms561101ba_readConversion(uint8_t regAddr, uint8_t osr) {
	uint16_t maxConversionTime;
	switch (osr) {
		case MS561101BA_OSR_256:
			maxConversionTime = MS561101BA_MAX_CONVERSION_TIME_OSR_256;
			break;
		case MS561101BA_OSR_512:
			maxConversionTime = MS561101BA_MAX_CONVERSION_TIME_OSR_512;
			break;
		case MS561101BA_OSR_1024:
			maxConversionTime = MS561101BA_MAX_CONVERSION_TIME_OSR_1024;
			break;
		case MS561101BA_OSR_2048:
			maxConversionTime = MS561101BA_MAX_CONVERSION_TIME_OSR_2048;
			break;
		case MS561101BA_OSR_4096:
			maxConversionTime = MS561101BA_MAX_CONVERSION_TIME_OSR_4096;
			break;
		default:
			return -1;
	}

	if (i2c_writeBytes(MS561101BA_DEFAULT_ADDRESS, regAddr + osr, 0, NULL) == RDY_OK) {
		chThdSleepMicroseconds(maxConversionTime);
		uint8_t adcOutput[3];
		if (i2c_readBytes(MS561101BA_DEFAULT_ADDRESS, 0, 3, adcOutput) == RDY_OK) {
			return (((int32_t) adcOutput[0]) << 16) | (((int32_t) adcOutput[1]) << 8)  | ((int32_t) adcOutput[2]);
		}
	}
	return -1;
}

uint16_t ms561101ba_getSENST1(void) {
	return prom[1];
}
uint16_t ms561101ba_getOFFT1(void) {
	return prom[2];
}
uint16_t ms561101ba_getTCS(void) {
	return prom[3];
}
uint16_t ms561101ba_getTCO(void) {
	return prom[4];
}
uint16_t ms561101ba_getTREF(void) {
	return prom[5];
}
uint16_t ms561101ba_getTEMPSENS(void) {
	return prom[6];
}
