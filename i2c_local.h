#ifndef I2C_LOCAL_H_
#define I2C_LOCAL_H_

#define I2C_BUS I2CD1

void I2CInitLocal(void);
msg_t i2c_transmit(i2caddr_t addr, const uint8_t *txbuf,
                  size_t txbytes, uint8_t *rxbuf, size_t rxbytes);
msg_t i2c_receive(i2caddr_t addr, uint8_t *rxbuf, size_t rxbytes);

#endif /* I2C_LOCAL_H_ */
