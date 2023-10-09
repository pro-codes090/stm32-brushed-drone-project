/*
 * pid.h
 *
 *  Created on: Aug 2, 2023
 *      Author: pro
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>



typedef struct {

float p_gain ;
float i_gain ;
float d_gain ;

float sampling_time;
float filter_sampling_time ;

float prevErrro ;
float prevMeasurment ;
float intgrator ;
float derevative ;

// limits for controller output
float limitMax ;
float limitMin ;

// limits for integral output
float limitMaxInt ;
float limitMinInt ;

float out ;
}pidController_t;


void pid_init(pidController_t* pidController ) ;
float pid_update(pidController_t* pidController , float setPoint ,float measuremnt ) ;
#endif /* PID_H_ */
