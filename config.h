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

#ifndef _config_h
#define _config_h

#include "24aa/eeprom.h"

#define CONFIG_HEADER	    "config_default.h"

enum configParameters {
    CONFIG_VERSION = 0,
    SYSTEM_ID,
    RADIO_TYPE,
    RADIO_THRO_CH,
    RADIO_ROLL_CH,
    RADIO_PITC_CH,
    RADIO_RUDD_CH,
    RADIO_GEAR_CH,
    RADIO_FLAP_CH,
    RADIO_AUX2_CH,
    RADIO_AUX3_CH,
    RADIO_AUX4_CH,
    RADIO_AUX5_CH,
    RADIO_AUX6_CH,
    RADIO_AUX7_CH,
    CONFIG_NUM_PARAMS
};

typedef struct {
    unsigned int paramId;
    unsigned int num;
    float values[CONFIG_NUM_PARAMS];
} __attribute__((packed)) paramStruct_t;

extern float p[CONFIG_NUM_PARAMS];
extern const char *configParameterStrings[];
extern EepromFileStream EepromFile;

extern void configInit(void);
extern void configEepromRead(void);
extern void configEepromWrite(void);
//extern void configFlashRead(void);
//extern unsigned char configFlashWrite(void);
extern void configLoadDefault(void);
extern unsigned int configParameterRead(void *data);
extern unsigned int configParameterWrite(void *data);

#endif
