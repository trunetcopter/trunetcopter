#include "config.h"
#include "radio.h"
#include <string.h>

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
			chThdSleepMilliseconds(10);
		}
	}

	return 0;
}

void radioInit(void) {
    //AQ_NOTICE("Radio init\n");

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
