/*
 * my_functions.h
 *
 *  Created on: Jan 9, 2021
 *      Author: kostasdeligiorgis
 */

#ifndef INC_DWM_FUNCTIONS_H_
#define INC_DWM_FUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

void leds(int led_mode);



//void Sleep(uint32_t x);
#define Sleep(__TIME)   delay_ms(__TIME);
#define delay_ms(__ms)    HAL_Delay(__ms)



/*! ------------------------------------------------------------------------------------------------------------------
 * Function: deca_sleep()
 *
 * Wait for a given amount of time.
 * /!\ This implementation is designed for a single threaded application and is blocking.
 *
 * param  time_ms  time to wait in milliseconds
 */
void deca_sleep(unsigned int time_ms);

#ifdef __cplusplus
}
#endif



void port_set_dw1000_slowrate(void);
void port_set_dw1000_fastrate(void);



int new_dwt_initialise(uint16_t config);


void deca_reset(void);



#endif /* INC_DWM_FUNCTIONS_H_ */
