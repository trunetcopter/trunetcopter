/* ------------------------------------------------------------------------------
  File: quat.h
  Author: CH Robotics
  Version: 1.0

  Description: Functions for operating on quaternions
------------------------------------------------------------------------------ */

#ifndef __quat_h
#define __quat_h

typedef struct _quat {
	 float a,b,c,d;
} quat;

int quat_mult( quat* src1, quat* src2, quat* dest );
int quat_conj( quat* src, quat* dest );
int quat_norm( quat* src );
int quat_add( quat* src1, quat* src2, quat* dest );
int quat_subtract( quat* src1, quat*src2, quat* dest );
int quat_scalar_mult( quat* qsrc, float scalar, quat* dest );

#endif
