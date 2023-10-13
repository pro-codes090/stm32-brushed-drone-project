/*
 * RC_Filters.h
 *
 *  Created on: Oct 13, 2023
 *      Author: pratham
 */

#ifndef RC_FILTERS_H_
#define RC_FILTERS_H_

typedef struct{
	float out_now ;
	float out_prev;

	float coeff1;
	float coeff2;


}filter_t;

void filter_init (filter_t*filter , float cutoff_freq , float sample_time) ;

float filter_update(filter_t*filter  , float measuremnt ) ;




#endif /* RC_FILTERS_H_ */
