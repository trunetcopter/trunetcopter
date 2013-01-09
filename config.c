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

#include "config.h"
//#include "filer.h"
//#include "notice.h"
//#include "supervisor.h"
//#include "util.h"

#include "24aa/eeprom.h"
#include "eeprom_conf.h"

#include CONFIG_HEADER
#include <string.h>
//#include <stdio.h>
#include <math.h>

float p[CONFIG_NUM_PARAMS] __attribute__((section(".ccm")));

EepromFileStream EepromFile;
static uint8_t eeprom_buf[EEPROM_TX_DEPTH];

static const I2CEepromFileConfig eeprom_settings_cfg = {
  &EEPROM_I2CD,
  EEPROM_SETTINGS_START,
  EEPROM_SETTINGS_END,
  EEPROM_SIZE,
  EEPROM_PAGE_SIZE,
  EEPROM_I2C_ADDR,
  MS2ST(EEPROM_WRITE_TIME_MS),
  eeprom_buf,
};

const char *configParameterStrings[] = {
    "CONFIG_VERSION",
    "SYSTEM_ID",
    "RADIO_TYPE",
    "RADIO_THRO_CH",
    "RADIO_ROLL_CH",
    "RADIO_PITC_CH",
    "RADIO_RUDD_CH",
    "RADIO_GEAR_CH",
    "RADIO_FLAP_CH",
    "RADIO_AUX2_CH",
    "RADIO_AUX3_CH",
    "RADIO_AUX4_CH",
    "RADIO_AUX5_CH",
    "RADIO_AUX6_CH",
    "RADIO_AUX7_CH"
};

void configLoadDefault(void) {
    p[CONFIG_VERSION] = DEFAULT_CONFIG_VERSION;
    p[SYSTEM_ID] = DEFAULT_SYSTEM_ID;
    p[RADIO_TYPE] = DEFAULT_RADIO_TYPE;
    p[RADIO_THRO_CH] = DEFAULT_RADIO_THRO_CH;
    p[RADIO_ROLL_CH] = DEFAULT_RADIO_ROLL_CH;
    p[RADIO_PITC_CH] = DEFAULT_RADIO_PITC_CH;
    p[RADIO_RUDD_CH] = DEFAULT_RADIO_RUDD_CH;
    p[RADIO_GEAR_CH] = DEFAULT_RADIO_GEAR_CH;
    p[RADIO_FLAP_CH] = DEFAULT_RADIO_FLAP_CH;
    p[RADIO_AUX2_CH] = DEFAULT_RADIO_AUX2_CH;
    p[RADIO_AUX3_CH] = DEFAULT_RADIO_AUX3_CH;
    p[RADIO_AUX4_CH] = DEFAULT_RADIO_AUX4_CH;
    p[RADIO_AUX5_CH] = DEFAULT_RADIO_AUX5_CH;
    p[RADIO_AUX6_CH] = DEFAULT_RADIO_AUX6_CH;
    p[RADIO_AUX7_CH] = DEFAULT_RADIO_AUX7_CH;
}

void configEepromRead(void) {
	uint32_t i = 0;

	chFileStreamSeek(&EepromFile, 0);
	for (i=0; i < (sizeof(p)/4); i++) {
		p[i] = EepromReadWord(&EepromFile);
	}
}

void configEepromWrite(void) {
	uint32_t i = 0;

	chFileStreamSeek(&EepromFile, 0);
	for (i=0; i < (sizeof(p)/4); i++) {
		EepromWriteWord(&EepromFile, p[i]);
	}
}

void configInit(void) {
    float ver = 0.0f;

    EepromFileOpen(&EepromFile, &eeprom_settings_cfg);

    // start with what's in flash
    configEepromRead();

    // get flash version
    ver = p[CONFIG_VERSION];
    if (isnan(ver))
    	ver = 0.0f;

    // if compiled defaults are greater than flash version and loaded version
    if (DEFAULT_CONFIG_VERSION > ver && DEFAULT_CONFIG_VERSION > p[CONFIG_VERSION])
    	configLoadDefault();
    // if flash version greater than or equal to currently loaded version
    else if (ver >= p[CONFIG_VERSION])
    	configEepromRead();

    // if loaded version greater than flash version
    if (p[CONFIG_VERSION] > ver)
    	configEepromWrite();
}

unsigned int configParameterRead(void *data) {
    paramStruct_t *par = (paramStruct_t *)data;

    if (par->paramId + par->num > CONFIG_NUM_PARAMS)
	par->num = CONFIG_NUM_PARAMS - par->paramId;

    memcpy((char *)par->values, (char *)&p[par->paramId], par->num * sizeof(float));

    return par->num * sizeof(float);
}

unsigned int configParameterWrite(void *data) {
    paramStruct_t *par = (paramStruct_t *)data;

    memcpy((char *)&p[par->paramId], (char *)par->values, par->num * sizeof(float));

    return configParameterRead(data);
}
