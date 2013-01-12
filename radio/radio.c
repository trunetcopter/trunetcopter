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
#include "radio.h"
#include <string.h>

#include "comm.h"
#include "mavlink.h"

radioStruct_t radioData __attribute__((section(".ccm")));

// calculate radio reception quality
void radioReceptionQuality(int q) {
    radioData.quality = utilFilter(&radioData.qualityFilter, (float)(q + 1)) * 0.5f * 100.0f;
}

static WORKING_AREA(waThreadRadio, RADIO_STACK_SIZE);
static msg_t ThreadRadio(void *arg) {
	(void)arg;
	chRegSetThreadName("radio");

	EventListener elRadio;
	if (radioData.radioType == 1 || radioData.radioType == 2) {
		chEvtRegisterMask((EventSource *)chnGetEventSource(radioData.serialPort), &elRadio, 1);
	}

	mavlinkNotice(MAV_SEVERITY_INFO, "Radio Polling Initialized!");

	while (TRUE) {
		if (radioData.radioType == 1 || radioData.radioType == 2) {
			flagsmask_t flags;
		    int q;

			chEvtWaitOneTimeout(EVENT_MASK(1), MS2ST(10));
			flags = chEvtGetAndClearFlags(&elRadio);

			if (flags & CHN_INPUT_AVAILABLE) {
				int32_t c = 0;
				while (c != Q_TIMEOUT) {
					c = chnGetTimeout(radioData.serialPort, TIME_IMMEDIATE);
					switch (radioData.radioType) {
							case 2:
								q = futabaCharIn(c);
								radioData.lastUpdate = chTimeNow();
								radioReceptionQuality(q);
								break;
							case 3:
								//TODO
								break;
					}
				}
			} else {
				if (chTimeNow() - radioData.lastUpdate > 50)
					radioReceptionQuality(-1);
			}
		} else {
			chThdSleepMilliseconds(100);
		}
	}

	return 0;
}

void radioInit(void) {
	mavlinkNotice(MAV_SEVERITY_INFO, "Radio Initialized!");

    memset((void *)&radioData, 0, sizeof(radioData));

    utilFilterInit(&radioData.qualityFilter, (1.0f / 50.0f), 0.75f, 0.0f);

    radioData.radioType = (int8_t)p[RADIO_TYPE]; //TODO

    switch (radioData.radioType) {
		case 0:
		case 1:
			//spektrumInit();
			break;
		case 2:
			futabaInit();
			break;
		case 3:
			//ppmInit();
			break;
		case 4:
			// TODO
			break;
    }

    chThdCreateStatic(waThreadRadio, sizeof(waThreadRadio), RADIO_PRIORITY, ThreadRadio, NULL);
}
