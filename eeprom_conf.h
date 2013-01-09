#ifndef EEPROM_CONF_H_
#define EEPROM_CONF_H_


#define USE_EEPROM_TEST_SUIT    FALSE        /* build test suit for EEPROM abstraction layer */

#define EEPROM_PAGE_SIZE        64          /* page size in bytes. Consult datasheet. */
#define EEPROM_SIZE             32768       /* total amount of memory in bytes */
#define EEPROM_I2CD             I2CD1       /* ChibiOS I2C driver used to communicate with EEPROM */
#define EEPROM_I2C_ADDR         0b1010000   /* EEPROM address on bus */
#define EEPROM_WRITE_TIME_MS    5           /* time to write one page in mS. Consult datasheet! */
#define EEPROM_TX_DEPTH         (EEPROM_PAGE_SIZE + 2)/* temporal transmit buffer depth for eeprom driver */

/* region for settings file */
#define EEPROM_SETTINGS_START         0
#define EEPROM_SETTINGS_SIZE          4096
#define EEPROM_SETTINGS_END           (EEPROM_SETTINGS_START + EEPROM_SETTINGS_SIZE)

#endif /* EEPROM_CONF_H_ */
