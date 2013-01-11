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

#include "radio.h"
#include "futaba.h"

#include <string.h>

futabaStruct_t futabaData __attribute__((section(".ccm")));

int futabaDecode(void) {
    if (futabaData.u.rawBuf[22] & 0b0100) {
    	return -1;
    } else {
		radioData.channels[0] = 1696 - futabaData.u.channels.channel3;
		radioData.channels[1] = futabaData.u.channels.channel1 - 1024;
		radioData.channels[2] = futabaData.u.channels.channel2 - 1024;
		radioData.channels[3] = futabaData.u.channels.channel4 - 1024;
		radioData.channels[4] = 1024 - futabaData.u.channels.channel5;
		radioData.channels[5] = 1024 - futabaData.u.channels.channel6;
		radioData.channels[6] = 1024 - futabaData.u.channels.channel7;
		radioData.channels[7] = 1024 - futabaData.u.channels.channel8;
		radioData.channels[8] = 1024 - futabaData.u.channels.channel9;
		radioData.channels[9] = 1024 - futabaData.u.channels.channel10;
		radioData.channels[10] = 1024 - futabaData.u.channels.channel11;
		radioData.channels[11] = 1024 - futabaData.u.channels.channel12;
		radioData.channels[12] = 1024 - futabaData.u.channels.channel13;
		radioData.channels[13] = 1024 - futabaData.u.channels.channel14;
		radioData.channels[14] = 1024 - futabaData.u.channels.channel15;
		radioData.channels[15] = 1024 - futabaData.u.channels.channel16;
		// Two digital channels (0 or 1, all or nothing)
		radioData.channels[16] = futabaData.u.rawBuf[22] & 0b0001 ? 800 : -800;
		radioData.channels[17] = futabaData.u.rawBuf[22] & 0b0010 ? 800 : -800;

		return 1;
    }
}

int futabaCharIn(unsigned char c) {
    switch (futabaData.state) {
		case FUTABA_WAIT_SYNC:
			if (c == FUTABA_START_CHAR) {
				futabaData.state = FUTABA_WAIT_DATA;
				futabaData.dataCount = 0;
			}
			break;

		case FUTABA_WAIT_DATA:
			futabaData.u.rawBuf[futabaData.dataCount++] = c;
			if (futabaData.dataCount == 23)
				futabaData.state = FUTABA_WAIT_END;
			break;

		case FUTABA_WAIT_END:
			futabaData.state = FUTABA_WAIT_SYNC;
			if (c == FUTABA_END_CHAR) {
				return futabaDecode();
			}
			break;
    }

    return 0;
}

void futabaInit(void) {
    memset((void *)&futabaData, 0, sizeof(futabaData));

	const SerialConfig sbusPortConfig = {
			FUTABA_BAUD,
	        (1 << 10),
	        USART_CR2_STOP2_BITS,
	        0//(1 << 11)
	};
	radioData.serialPort = &FUTABA_SERIAL_DEVICE;
	sdStart(radioData.serialPort, &sbusPortConfig);
}
