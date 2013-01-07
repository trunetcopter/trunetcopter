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

#ifndef _util_h
#define _util_h

#include <stdlib.h>
#include "ch.h"

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(a)   ((a)<0?(-(a)):(a))
#define sgn(a)   ((a)<0?(-1):(1))

// first order filter
typedef struct {
    float tc;
    float z1;
} utilFilter_t;

extern int constrainInt(int i, int lo, int hi);
extern float constrainFloat(float i, float lo, float hi);
extern void dumpFloat(unsigned char n, float *floats);
extern void dumpInt(unsigned char n, int *ints);
extern void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint);
extern void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint);
extern float utilFilter(utilFilter_t *f, float signal);
extern float utilFilter3(utilFilter_t *f, float signal);
extern void utilFilterReset(utilFilter_t *f, float setpoint);
extern void utilFilterReset3(utilFilter_t *f, float setpoint);
extern int ftoa(char *buf, float f, unsigned int digits);

extern void randomInit(void);
extern uint32_t random_int(void);

#endif
