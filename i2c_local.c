#include "ch.h"
#include "hal.h"

#include "i2c_local.h"

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
extern uint32_t GlobalFlags;

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
  if (status == RDY_TIMEOUT){
    i2cStop(&I2C_BUS);
    chThdSleepMilliseconds(1);
    i2cStart(&I2C_BUS, &i2cfg1);
    //setGlobalFlag(I2C_RESTARTED_FLAG);
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
  chDbgAssert(status == RDY_OK, "i2c_transmit(), #1", "error in driver");
  if (status == RDY_TIMEOUT){
    i2cStop(&I2C_BUS);
    chThdSleepMilliseconds(1);
    i2cStart(&I2C_BUS, &i2cfg1);
    //setGlobalFlag(I2C_RESTARTED_FLAG);
    return status;
  }
  return status;
}
