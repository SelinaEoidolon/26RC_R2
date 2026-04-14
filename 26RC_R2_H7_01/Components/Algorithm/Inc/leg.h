#ifndef __LEG_H
#define __LEG_H

#include "struct_typedef.h"


typedef struct{
	float theta1;
	float theta2;
}leg;

extern leg lf_leg;
extern leg rf_leg;
extern float lb_leg;
extern float rb_leg;

extern int8_t LEG_Cmd ;
extern float bleg_theta ;
extern float fleg_high ;

float Q_rsqrt(float number);
int InverseKinematics_Continuous(float x, float y,
                                 float last_theta1, float last_theta2,
                                 float *theta1, float *theta2);
void backLeg_Left(float h,float*theta);
void backLeg_Right(float h,float*theta);


#endif



