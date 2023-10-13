/*
 * Rc_Filters.c
 *
 *  Created on: Oct 13, 2023
 *      Author: pratham
 */

#include "RC_Filters.h"


void filter_init (filter_t*filter , float cutoff_freq , float sample_time) {

// compute the rc time constant fro the cutoff freq
float RC = ( 1.0f / (6.283185307f * cutoff_freq)) ;
// calculate the (RC/(T + RC))
filter->coeff1 = (RC/(sample_time + RC)) ;
// calculate the (T/(T + RC))
filter->coeff2 = (sample_time / (sample_time + RC)) ;

filter->out_now = 0 ;
filter->out_prev= 0 ;

}

float filter_update(filter_t*filter  , float measuremnt ) {

filter->out_prev = filter->out_now ;
filter->out_now = ( filter->coeff1*(filter->out_prev) + filter->coeff2*(measuremnt) );

return (filter->out_now) ;

}
