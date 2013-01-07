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

#include "magn_hmc5883l.h"
#include "../i2c_local.h"
#include "../util.h"

#include <string.h>

uint8_t hmc5883l_buffer[6];
uint8_t hmc5883l_mode;
uint8_t hmc5883l_gain;
float hmc5883l_scaleFactors[8][3];

/** Power on and prepare for general usage.
 * This will prepare the magnetometer with default settings, ready for single-
 * use mode (very low power requirements). Default settings include 8-sample
 * averaging, 15 Hz data output rate, normal measurement bias, a,d 1090 gain (in
 * terms of LSB/Gauss). Be sure to adjust any settings you need specifically
 * after initialization, especially the gain settings if you happen to be seeing
 * a lot of -4096 values (see the datasheet for mor information).
 */
void hmc5883l_initialize(void) {
	memset((void *)&hmc5883l_buffer, 0, sizeof(hmc5883l_buffer));

	// We need to wait a bit...
	chThdSleepMicroseconds(HMC5883L_READY_FOR_I2C_COMMAND);

    // write CONFIG_A register
    i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

    // write CONFIG_B register
    hmc5883l_setGain(HMC5883L_GAIN_1090);

    // write MODE register
    hmc5883l_setMode(HMC5883L_MODE_SINGLE);

    // TODO: Maybe it would be a good idea to use the EEPROM
    // to store the scale factors and recover the last valid
    // value in case of a calibration fail.
    uint8_t gain;
    for (gain = HMC5883L_GAIN_1370; gain <= HMC5883L_GAIN_220; gain ++) {
    	hmc5883l_scaleFactors[gain][0] = 1.0f;
    	hmc5883l_scaleFactors[gain][1] = 1.0f;
    	hmc5883l_scaleFactors[gain][2] = 1.0f;
    }
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool_t hmc5883l_testConnection(void) {
    if (i2c_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_ID_A, 3, hmc5883l_buffer) == RDY_OK) {
        return (hmc5883l_buffer[0] == 'H' && hmc5883l_buffer[1] == '4' && hmc5883l_buffer[2] == '3');
    }
    return 0;
}

// CONFIG_A register

/** Get number of samples averaged per measurement.
 * @return Current samples averaged per measurement (0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_AVERAGING_8
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
uint8_t hmc5883l_getSampleAveraging(void) {
    i2c_readBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}
/** Set number of samples averaged per measurement.
 * @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
void hmc5883l_setSampleAveraging(uint8_t averaging) {
    i2c_writeBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}
/** Get data output rate value.
 * The Table below shows all selectable output rates in continuous measurement
 * mode. All three channels shall be measured within a given output rate. Other
 * output rates with maximum rate of 160 Hz can be achieved by monitoring DRDY
 * interrupt pin in single measurement mode.
 *
 * Value | Typical Data Output Rate (Hz)
 * ------+------------------------------
 * 0     | 0.75
 * 1     | 1.5
 * 2     | 3
 * 3     | 7.5
 * 4     | 15 (Default)
 * 5     | 30
 * 6     | 75
 * 7     | Not used
 *
 * @return Current rate of data output to registers
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
uint8_t hmc5883l_getDataRate(void) {
    i2c_readBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}
/** Set data output rate value.
 * @param rate Rate of data output to registers
 * @see getDataRate()
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
void hmc5883l_setDataRate(uint8_t rate) {
    i2c_writeBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}
/** Get measurement bias value.
 * @return Current bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
uint8_t hmc5883l_getMeasurementBias(void) {
    i2c_readBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}
/** Set measurement bias value.
 * @param bias New bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
void hmc5883l_setMeasurementBias(uint8_t bias) {
    i2c_writeBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

// CONFIG_B register

/** Get magnetic field gain value.
 * The table below shows nominal gain settings. Use the "Gain" column to convert
 * counts to Gauss. Choose a lower gain value (higher GN#) when total field
 * strength causes overflow in one of the data output registers (saturation).
 * The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
 *
 * Value | Field Range | Gain (LSB/Gauss)
 * ------+-------------+-----------------
 * 0     | +/- 0.88 Ga | 1370
 * 1     | +/- 1.3 Ga  | 1090 (Default)
 * 2     | +/- 1.9 Ga  | 820
 * 3     | +/- 2.5 Ga  | 660
 * 4     | +/- 4.0 Ga  | 440
 * 5     | +/- 4.7 Ga  | 390
 * 6     | +/- 5.6 Ga  | 330
 * 7     | +/- 8.1 Ga  | 230
 *
 * @return Current magnetic field gain value
 * @see HMC5883L_GAIN_1090
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
uint8_t hmc5883l_getGain(void) {
    i2c_readBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, HMC5883L_CRB_GAIN_BIT, HMC5883L_CRB_GAIN_LENGTH, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}
/** Set magnetic field gain value.
 * @param gain New magnetic field gain value
 * @see getGain()
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
void hmc5883l_setGain(uint8_t newGain) {
    // use this method to guarantee that bits 4-0 are set to zero, which is a
    // requirement specified in the datasheet; it's actually more efficient than
    // using the I2Cdev.writeBits method
    if (i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, newGain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1))) {
    	hmc5883l_gain = newGain; // track to select the scale factor
    }
}

// MODE register

/** Get measurement mode.
 * In continuous-measurement mode, the device continuously performs measurements
 * and places the result in the data register. RDY goes high when new data is
 * placed in all three registers. After a power-on or a write to the mode or
 * configuration register, the first measurement set is available from all three
 * data output registers after a period of 2/fDO and subsequent measurements are
 * available at a frequency of fDO, where fDO is the frequency of data output.
 *
 * When single-measurement mode (default) is selected, device performs a single
 * measurement, sets RDY high and returned to idle mode. Mode register returns
 * to idle mode bit values. The measurement remains in the data output register
 * and RDY remains high until the data output register is read or another
 * measurement is performed.
 *
 * @return Current measurement mode
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
uint8_t hmc5883l_getMode(void) {
    i2c_readBits(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT, HMC5883L_MODEREG_LENGTH, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}
/** Set measurement mode.
 * @param newMode New measurement mode
 * @see getMode()
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
void hmc5883l_setMode(uint8_t newMode) {
    // use this method to guarantee that bits 7-2 are set to zero, which is a
    // requirement specified in the datasheet; it's actually more efficient than
    // using the I2Cdev.writeBits method
    i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, hmc5883l_mode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    hmc5883l_mode = newMode; // track to tell if we have to clear bit 7 after a read
}

// DATA* registers

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode is active.
 * @param x 16-bit signed integer container for X-axis heading
 * @param y 16-bit signed integer container for Y-axis heading
 * @param z 16-bit signed integer container for Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
void hmc5883l_getHeading(int16_t *x, int16_t *y, int16_t *z, bool_t does_not_get) {
	int16_t rawx, rawy, rawz;
	if (does_not_get != 1) {
		hmc5883l_getRawHeading(&rawx, &rawy, &rawz);
	} else {
		rawx = (int16_t)*x;
		rawy = (int16_t)*y;
		rawz = (int16_t)*z;
	}
	*x = (int16_t) (hmc5883l_scaleFactors[hmc5883l_gain][0]*rawx);
	*y = (int16_t) (hmc5883l_scaleFactors[hmc5883l_gain][1]*rawy);
	*z = (int16_t) (hmc5883l_scaleFactors[hmc5883l_gain][2]*rawz);
}
/** Get X-axis heading measurement.
 * @return 16-bit signed integer with X-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
int16_t hmc5883l_getHeadingX(void) {
	int16_t x,y,z;
	hmc5883l_getHeading(&x,&y,&z, 0);
	return x;
}
/** Get Y-axis heading measurement.
 * @return 16-bit signed integer with Y-axis heading
 * @see HMC5883L_RA_DATAY_H
 */
int16_t hmc5883l_getHeadingY(void) {
	int16_t x,y,z;
	hmc5883l_getHeading(&x,&y,&z, 0);
	return y;
}
/** Get Z-axis heading measurement.
 * @return 16-bit signed integer with Z-axis heading
 * @see HMC5883L_RA_DATAZ_H
 */
int16_t hmc5883l_getHeadingZ(void) {
	int16_t x,y,z;
	hmc5883l_getHeading(&x,&y,&z, 0);
	return z;
}

/** Get raw 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode is active.
 * @param x 16-bit signed integer container for X-axis heading
 * @param y 16-bit signed integer container for Y-axis heading
 * @param z 16-bit signed integer container for Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
void hmc5883l_getRawHeading(int16_t *x, int16_t *y, int16_t *z) {
	if (hmc5883l_mode == HMC5883L_MODE_SINGLE) {
		/*
		 * When single-measurement mode is selected, device performs a single
		 * measurement, sets RDY high and returned to idle mode. Mode register
		 * returns to idle mode bit values. The measurement remains in the
		 * data output register and RDY remains high until the data output
		 * register is read or another measurement is performed.
		 */
		i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
		chThdSleepMilliseconds(HMC5883L_MEASUREMENT_PERIOD);
	} else {
		/*
		 * In continuous-measurement mode, the device continuously
		 * performs measurements and places the result in the data register.
		 * RDY goes high when new data is placed in all three registers.
		 * After a power-on or a write to the mode or configuration register,
		 * the first measurement set is available from all three data output
		 * registers after a period of 2/fDO and subsequent measurements are
		 * available at a frequency of fDO, where fDO is the frequency of
		 * data output.
		 *
		 * The data output register lock bit is set when this some
		 * but not all for of the six data output registers have been read.
		 * When this bit is set, the six data output registers are locked
		 * and any new data will not be placed in these register until
		 * one of three conditions are met: one, all six bytes have been
		 * read or the mode changed, two, the mode is changed, or three,
		 * the measurement configuration is changed.
		 */
	}
	i2c_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 6, hmc5883l_buffer);
    *x = (((int16_t) hmc5883l_buffer[0]) << 8) | hmc5883l_buffer[1];
    *y = (((int16_t) hmc5883l_buffer[4]) << 8) | hmc5883l_buffer[5];
    *z = (((int16_t) hmc5883l_buffer[2]) << 8) | hmc5883l_buffer[3];
}

/** Get raw X-axis heading measurement.
 * @return 16-bit signed integer with X-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
int16_t hmc5883l_getRawHeadingX(void) {
	int16_t x,y,z;
	hmc5883l_getRawHeading(&x,&y,&z);
    return x;
}

/** Get raw Y-axis heading measurement.
 * @return 16-bit signed integer with Y-axis heading
 * @see HMC5883L_RA_DATAY_H
 */
int16_t hmc5883l_getRawHeadingY(void) {
	int16_t x,y,z;
	hmc5883l_getRawHeading(&x,&y,&z);
    return y;
}

/** Get raw Z-axis heading measurement.
 * @return 16-bit signed integer with Z-axis heading
 * @see HMC5883L_RA_DATAZ_H
 */
int16_t hmc5883l_getRawHeadingZ(void) {
	int16_t x,y,z;
	hmc5883l_getRawHeading(&x,&y,&z);
    return z;
}

// STATUS register

/** Get data output register lock status.
 * This bit is set when this some but not all for of the six data output
 * registers have been read. When this bit is set, the six data output registers
 * are locked and any new data will not be placed in these register until one of
 * three conditions are met: one, all six bytes have been read or the mode
 * changed, two, the mode is changed, or three, the measurement configuration is
 * changed.
 * @return Data output register lock status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_LOCK_BIT
 */
bool_t hmc5883l_getLockStatus(void) {
    i2c_readBit(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}
/** Get data ready status.
 * This bit is set when data is written to all six data registers, and cleared
 * when the device initiates a write to the data output registers and after one
 * or more of the data output registers are written to. When RDY bit is clear it
 * shall remain cleared for 250 us. DRDY pin can be used as an alternative to
 * the status register for monitoring the device for measurement data.
 * @return Data ready status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_READY_BIT
 */
bool_t hmc5883l_getReadyStatus(void) {
    i2c_readBit(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}

// ID_* registers

/** Get identification byte A
 * @return ID_A byte (should be 01001000, ASCII value 'H')
 */
uint8_t hmc5883l_getIDA(void) {
    i2c_readByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_ID_A, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}
/** Get identification byte B
 * @return ID_A byte (should be 00110100, ASCII value '4')
 */
uint8_t hmc5883l_getIDB(void) {
    i2c_readByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_ID_B, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}
/** Get identification byte C
 * @return ID_A byte (should be 00110011, ASCII value '3')
 */
uint8_t hmc5883l_getIDC(void) {
    i2c_readByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_ID_C, hmc5883l_buffer);
    return hmc5883l_buffer[0];
}

bool_t hmc5883l_calibrate(int8_t testGain) {

	// Keep the current status ...
	uint8_t previousGain = hmc5883l_getGain();

	// Set the gain
	if (testGain < 0) {
		testGain = hmc5883l_gain;
	}
	hmc5883l_setGain(testGain);

	// To check the HMC5883L for proper operation, a self test
	// feature in incorporated in which the sensor offset straps
	// are excited to create a nominal field strength (bias field)
	// to be measured. To implement self test, the least significant
	// bits (MS1 and MS0) of configuration register A are changed
	// from 00 to 01 (positive bias) or 10 (negetive bias)
	hmc5883l_setMeasurementBias(HMC5883L_BIAS_POSITIVE);

	// Then, by placing the mode register into single-measurement mode ...
	hmc5883l_setMode(HMC5883L_MODE_SINGLE);

	// Two data acquisition cycles will be made on each magnetic vector.
	// The first acquisition will be a set pulse followed shortly by
	// measurement data of the external field. The second acquisition
	// will have the offset strap excited (about 10 mA) in the positive
	// bias mode for X, Y, and Z axes to create about a ±1.1 gauss self
	// test field plus the external field.
	// The first acquisition values will be subtracted from the
	// second acquisition, and the net measurement will be placed into
	// the data output registers.
	int16_t x,y,z;
	hmc5883l_getRawHeading(&x,&y,&z);

	// In the event the ADC reading overflows or underflows for the
	// given channel, or if there is a math overflow during the bias
	// measurement, this data register will contain the value -4096.
	// This register value will clear when after the next valid
	// measurement is made.
	if (min(x,min(y,z)) == -4096) {
		hmc5883l_scaleFactors[testGain][0] = 1.0f;
		hmc5883l_scaleFactors[testGain][1] = 1.0f;
		hmc5883l_scaleFactors[testGain][2] = 1.0f;
		return 0;
	}
	hmc5883l_getRawHeading(&x,&y,&z);

	if (min(x,min(y,z)) == -4096) {
		hmc5883l_scaleFactors[testGain][0] = 1.0f;
		hmc5883l_scaleFactors[testGain][1] = 1.0f;
		hmc5883l_scaleFactors[testGain][2] = 1.0f;
		return 0;
	}

	// Since placing device in positive bias mode
	// (or alternatively negative bias mode) applies
	// a known artificial field on all three axes,
	// the resulting ADC measurements in data output
	// registers can be used to scale the sensors.
	float xExpectedSelfTestValue =
			HMC5883L_SELF_TEST_X_AXIS_ABSOLUTE_GAUSS *
			HMC5883L_LSB_PER_GAUS[testGain];
	float yExpectedSelfTestValue =
			HMC5883L_SELF_TEST_Y_AXIS_ABSOLUTE_GAUSS *
			HMC5883L_LSB_PER_GAUS[testGain];
	float zExpectedSelfTestValue =
			HMC5883L_SELF_TEST_Z_AXIS_ABSOLUTE_GAUSS *
			HMC5883L_LSB_PER_GAUS[testGain];

	hmc5883l_scaleFactors[testGain][0] = xExpectedSelfTestValue/x;
	hmc5883l_scaleFactors[testGain][1] = yExpectedSelfTestValue/y;
	hmc5883l_scaleFactors[testGain][2] = zExpectedSelfTestValue/z;

	hmc5883l_setGain(previousGain);
	hmc5883l_setMeasurementBias(HMC5883L_BIAS_NORMAL);

	return 1;
}
