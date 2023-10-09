/*
 * pid.c
 *
 *  Created on: Aug 2, 2023
 *      Author: pro
 */

#include "pid.h"


void pid_init(pidController_t* pidController) {
pidController->prevErrro      = 0 ;
pidController->prevMeasurment = 0 ;
pidController->intgrator 	  = 0 ;
pidController->derevative 	  = 0 ;
pidController->out      	  = 0 ;

}


float pid_update(pidController_t* pidController , float setPoint ,float measuremnt ) {

	// get the Error
	float error = setPoint = measuremnt ;
	// proportional controller
	float proportional = error*pidController->p_gain ;
	// integrator controller
	pidController->intgrator = pidController->intgrator + ((pidController->i_gain*pidController->sampling_time)/ 2)*(error + pidController->prevErrro) ;
	// integral anti wind up
	// clamping the integrator
	if (pidController->intgrator  > pidController->limitMaxInt ) {
		pidController->intgrator = pidController->limitMaxInt  ;
	}else if (pidController->intgrator < pidController->limitMinInt) {
		pidController->intgrator = pidController->limitMinInt ;
	}

	// Derivative controller using derevative on measurement
	pidController->derevative = ((2*pidController->d_gain)*(measuremnt -pidController->prevMeasurment)
								+ pidController->derevative*(2*pidController->filter_sampling_time - pidController->sampling_time))
								/ (pidController->sampling_time + 2*pidController->filter_sampling_time) ;

	// compute the final output
	pidController->out = proportional + pidController->intgrator + pidController->derevative ;

	// set the limits
   if (pidController->out > pidController->limitMax) {

		pidController->out = pidController->limitMax;

	} else if (pidController->out < pidController->limitMin) {

		pidController->out = pidController->limitMin;

	}
pidController->prevErrro = error ;
pidController->prevMeasurment = measuremnt;

}
