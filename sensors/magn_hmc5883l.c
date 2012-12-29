#include "magn_hmc5883l.h"
#include "../i2c_local.h"

#include <string.h>

uint8_t hmc5883l_buffer[6];
uint8_t hmc5883l_mode;

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

    // write CONFIG_A register
    i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

    // write CONFIG_B register
    hmc5883l_setGain(HMC5883L_GAIN_1090);

    // write MODE register
    hmc5883l_setMode(HMC5883L_MODE_SINGLE);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool_t hmc5883l_testConnection(void) {
    if (i2c_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_ID_A, 3, hmc5883l_buffer) == 3) {
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
void hmc5883l_setGain(uint8_t gain) {
    // use this method to guarantee that bits 4-0 are set to zero, which is a
    // requirement specified in the datasheet; it's actually more efficient than
    // using the I2Cdev.writeBits method
    i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
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
void hmc5883l_getHeading(int16_t *x, int16_t *y, int16_t *z) {
    i2c_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 6, hmc5883l_buffer);
    if (hmc5883l_mode == HMC5883L_MODE_SINGLE) i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    *x = (((int16_t)hmc5883l_buffer[0]) << 8) | hmc5883l_buffer[1];
    *y = (((int16_t)hmc5883l_buffer[4]) << 8) | hmc5883l_buffer[5];
    *z = (((int16_t)hmc5883l_buffer[2]) << 8) | hmc5883l_buffer[3];
}
/** Get X-axis heading measurement.
 * @return 16-bit signed integer with X-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
int16_t hmc5883l_getHeadingX(void) {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    i2c_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 6, hmc5883l_buffer);
    if (hmc5883l_mode == HMC5883L_MODE_SINGLE) i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    return (((int16_t)hmc5883l_buffer[0]) << 8) | hmc5883l_buffer[1];
}
/** Get Y-axis heading measurement.
 * @return 16-bit signed integer with Y-axis heading
 * @see HMC5883L_RA_DATAY_H
 */
int16_t hmc5883l_getHeadingY(void) {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    i2c_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 6, hmc5883l_buffer);
    if (hmc5883l_mode == HMC5883L_MODE_SINGLE) i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    return (((int16_t)hmc5883l_buffer[4]) << 8) | hmc5883l_buffer[5];
}
/** Get Z-axis heading measurement.
 * @return 16-bit signed integer with Z-axis heading
 * @see HMC5883L_RA_DATAZ_H
 */
int16_t hmc5883l_getHeadingZvoid(void) {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    i2c_readBytes(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_DATAX_H, 6, hmc5883l_buffer);
    if (hmc5883l_mode == HMC5883L_MODE_SINGLE) i2c_writeByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    return (((int16_t)hmc5883l_buffer[2]) << 8) | hmc5883l_buffer[3];
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
