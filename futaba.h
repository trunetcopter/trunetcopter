#ifndef futaba_h
#define futaba_h

#include "ch.h"

#define FUTABA_BAUD		100000

#define FUTABA_START_CHAR	0x0f
#define FUTABA_END_CHAR		0x00

#define FUTABA_WAIT_SYNC	0x00
#define FUTABA_WAIT_DATA	0x01
#define FUTABA_WAIT_END		0x02

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

typedef struct {
    unsigned channel1  : 11;
    unsigned channel2  : 11;
    unsigned channel3  : 11;
    unsigned channel4  : 11;
    unsigned channel5  : 11;
    unsigned channel6  : 11;
    unsigned channel7  : 11;
    unsigned channel8  : 11;
    unsigned channel9  : 11;
    unsigned channel10 : 11;
    unsigned channel11 : 11;
    unsigned channel12 : 11;
    unsigned channel13 : 11;
    unsigned channel14 : 11;
    unsigned channel15 : 11;
    unsigned channel16 : 11;
    }  __attribute__((packed)) futabaChannelStruct_t;

typedef struct {
    unsigned char state;
    unsigned char dataCount;

    union {
	uint8_t rawBuf[23];
	futabaChannelStruct_t channels;
    } u;
} futabaStruct_t;

extern futabaStruct_t futabaData;

extern void futabaInit(void);
extern int futabaCharIn(unsigned char c);

#endif
