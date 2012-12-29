#include "ch.h"
#include "hal.h"

#include "i2c_local.h"
#include <string.h>

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
//extern uint32_t GlobalFlags;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
/* interface #1 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000, //100000,
    FAST_DUTY_CYCLE_2, //STD_DUTY_CYCLE, //FAST_DUTY_CYCLE_16_9,
};

/*
 *******************************************************************************
 * INTERNAL FUNCTIONS
 *******************************************************************************
 */
static systime_t calc_timeout(I2CDriver *i2cp, size_t txbytes, size_t rxbytes){
  const uint32_t bitsinbyte = 10;
  uint32_t tmo;
  tmo = ((txbytes + rxbytes + 1) * bitsinbyte * 1000);
  tmo /= i2cp->config->clock_speed;
  tmo += 5; /* some additional time to be safer */
  return MS2ST(tmo);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
void I2CInitLocal(void){
  i2cStart(&I2C_BUS, &i2cfg1);
}

msg_t i2c_transmit(i2caddr_t addr, const uint8_t *txbuf, size_t txbytes,
                   uint8_t *rxbuf, size_t rxbytes){
  msg_t status = RDY_OK;
  systime_t tmo = calc_timeout(&I2C_BUS, txbytes, rxbytes);

  i2cAcquireBus(&I2C_BUS);
  status = i2cMasterTransmitTimeout(&I2C_BUS, addr, txbuf, txbytes, rxbuf, rxbytes, tmo);
  i2cReleaseBus(&I2C_BUS);
  if (status != RDY_OK){
	i2cAcquireBus(&I2C_BUS);
    i2cStop(&I2C_BUS);
    I2C1->CR1 |= I2C_CR1_SWRST;
    chThdSleepMilliseconds(1);
    i2cStart(&I2C_BUS, &i2cfg1);
    i2cReleaseBus(&I2C_BUS);
    return status;
  }
  return status;
}

msg_t i2c_receive(i2caddr_t addr, uint8_t *rxbuf, size_t rxbytes){
  msg_t status = RDY_OK;
  systime_t tmo = calc_timeout(&I2C_BUS, 0, rxbytes);

  i2cAcquireBus(&I2C_BUS);
  status = i2cMasterReceiveTimeout(&I2C_BUS, addr, rxbuf, rxbytes, tmo);
  i2cReleaseBus(&I2C_BUS);
  if (status != RDY_OK){
	i2cAcquireBus(&I2C_BUS);
    i2cStop(&I2C_BUS);
    I2C1->CR1 |= I2C_CR1_SWRST;
    chThdSleepMilliseconds(1);
    i2cStart(&I2C_BUS, &i2cfg1);
    i2cReleaseBus(&I2C_BUS);
    return status;
  }
  return status;
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
int8_t i2c_readBit(i2caddr_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = i2c_readByte(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t i2c_readWord(i2caddr_t devAddr, uint8_t regAddr, uint16_t *data) {
    return i2c_readWords(devAddr, regAddr, 1, data);
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
int8_t i2c_readBits(i2caddr_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = i2c_readByte(devAddr, regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (true = success)
 */
int8_t i2c_readByte(i2caddr_t devAddr, uint8_t regAddr, uint8_t *data) {
    return i2c_readBytes(devAddr, regAddr, 1, data);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t i2c_readBytes(i2caddr_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
	return i2c_transmit(devAddr, &regAddr, 1, data, length);
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Number of words read (0 indicates failure)
 */
int8_t i2c_readWords(i2caddr_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data) {
	uint16_t intermediate[(uint8_t)length];
	msg_t status;
	uint8_t i;

	status = i2c_transmit(devAddr, &regAddr, 1, (uint8_t *)intermediate, (uint8_t)(length * 2));
	if (status == RDY_OK) {
		for (i = 0; i < length; i++) {
			data[i] = (intermediate[2*i] << 8) | intermediate[2*i + 1];
		}
	}
	return status;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool_t i2c_writeBit(i2caddr_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    i2c_readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2c_writeByte(devAddr, regAddr, b);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool_t i2c_writeBits(i2caddr_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (i2c_readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return i2c_writeByte(devAddr, regAddr, b);
    } else {
        return 0;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool_t i2c_writeByte(i2caddr_t devAddr, uint8_t regAddr, uint8_t data) {
    return i2c_writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool_t i2c_writeWord(i2caddr_t devAddr, uint8_t regAddr, uint16_t data) {
    return i2c_writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool_t i2c_writeBytes(i2caddr_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
	uint8_t i;
	uint8_t txbuf[20], rxbuf[1];

	memset((void *)&txbuf, 0, sizeof(txbuf));
	memset((void *)&rxbuf, 0, sizeof(rxbuf));

	txbuf[0] = regAddr;
	for (i=0; i<14; i++)
		txbuf[i+1] = data[i];

	return i2c_transmit(devAddr, txbuf, length+1, rxbuf, 0);
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool_t i2c_writeWords(i2caddr_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
	uint8_t i;
	uint8_t txbuf[20], rxbuf[1];

	memset((void *)&txbuf, 0, sizeof(txbuf));
	memset((void *)&rxbuf, 0, sizeof(rxbuf));

	txbuf[0] = regAddr;
	for (i = 0; i < length * 2; i++) {
		txbuf[i+1] = (uint8_t)(data[i] >> 8);
		i++;
		txbuf[i+1] = (uint8_t)data[i];
	}

	return i2c_transmit(devAddr, txbuf, (length*2)+1, rxbuf, 0);
}
