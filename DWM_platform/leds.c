/*
 * leds.c
 *
 *  Created on: Dec 4, 2020
 *      Author: kostasdeligiorgis
 */

#include "main.h"
#include "deca_device_api.h"
#include "deca_regs.h"



void leds(int led_mode)
{
	/*
	 * GPIO5 is SPIPOL and GPIO6 is SPIPHA
	 */
	int count = 0;
	dwt_enablegpioclocks();

	if (led_mode == 0 )  // RxLED
	{
		dwt_setgpiodirection(DWT_GxM2 | DWT_GxM6 | DWT_GxM5, DWT_GxP6 | DWT_GxP5);
		while (count <= 5)
			{

				dwt_setgpiovalue(DWT_GxM2, DWT_GxP2); /* set GPIO2 high (LED will light up)*/
				/* set GPIO6 is high use short Sleep ON period*/
				if(dwt_getgpiovalue(DWT_GxP6))  /* only one GPIO should be tested at a time */
					HAL_Delay(100);
				else
					HAL_Delay(400);


				dwt_setgpiovalue(DWT_GxM2, 0); /* set GPIO2 low (LED will be off)*/
				/* set GPIO5 is high use short Sleep OFF period*/
				if(dwt_getgpiovalue(DWT_GxP5))
					HAL_Delay(100);
				else
					HAL_Delay(400);

				count ++;
			}

	}
	else if (led_mode == 1)  // TxLED
	{
		dwt_setgpiodirection(DWT_GxM3 | DWT_GxM6 | DWT_GxM5, DWT_GxP6 | DWT_GxP5);
		while (count <= 5)
			{

				dwt_setgpiovalue(DWT_GxM3, DWT_GxP3); /* set GPIO2 high (LED will light up)*/
				/* set GPIO6 is high use short Sleep ON period*/
				if(dwt_getgpiovalue(DWT_GxP6))  /* only one GPIO should be tested at a time */
					HAL_Delay(100);
				else
					HAL_Delay(400);


				dwt_setgpiovalue(DWT_GxM3, 0); /* set GPIO2 low (LED will be off)*/
				/* set GPIO5 is high use short Sleep OFF period*/
				if(dwt_getgpiovalue(DWT_GxP5))
					HAL_Delay(100);
				else
					HAL_Delay(400);

				count ++;
			}


	}


}

