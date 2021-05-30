/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Simple RX example code
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

/*
 * simple_Rx.c
 *
 *  Created on: Jan 18, 2021
 *      Author: kostasdeligiorgis
 */

#include <DWM_functions.h>
#include "main.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "usbd_cdc_if.h"


/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1024 + 1 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

///* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;


static volatile int commandStatus = DWT_SUCCESS;

/**
 * Application entry point.
 */

int simple_rx(void)
{
	/* Reset and initialise DW1000. See NOTE 2 below.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	 * performance.
	 */
	deca_reset(); /* Target specific drive of RSTn line into DW1000 low for a period. */

	port_set_dw1000_slowrate();

	if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
	{
		while (1){};
	}

	port_set_dw1000_fastrate();

	/* Configure LEDs management. */
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
	dwt_setleds(DWT_LEDS_ENABLE);

    /* Configure DW1000. */
	dwt_configure(&config);

	/* Loop forever receiving frames. */
    while(1)
    {
    	int i;

    	/* TESTING BREAKPOINT LOCATION #1 */

    	/* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
    	 * the RX buffer.
    	 * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
    	 * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame.
    	 */
    	for (i = 0 ; i < FRAME_LEN_MAX; i++ )
    	{
    		rx_buffer[i] = 0;
    	}

    	// Activate reception immediately.
    	commandStatus = dwt_rxenable(DWT_START_RX_IMMEDIATE);
    	if (commandStatus == DWT_ERROR) {
    		while(1){};
    	}

    	/* Poll until a frame is properly received or an error/timeout occurs.
    	 * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
    	 * function to access it.
    	 */

    	status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    	while (!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
    			status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    		}

//    	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
//    	{} //{ j++;};
//    	//j = 0;

    	if (status_reg & SYS_STATUS_RXFCG) {
    			// A frame has been received, copy it to our local buffer.
    			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    			if (frame_len <= FRAME_LEN_MAX)
    			{
    				dwt_readrxdata(rx_buffer, frame_len, 0);
    			}

    			// Clear good RX frame event in the DW1000 status register.
    			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
    	}
    	else
    	{
            /* Clear RX error events in the DW1000 status register. */
    		 dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    	}
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW1000 supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. In this example, LDE microcode is not loaded upon calling dwt_initialise(). This will prevent the IC from generating an RX timestamp. If
 *    time-stamping is required, DWT_LOADUCODE parameter should be used. See two-way ranging examples (e.g. examples 5a/5b).
 * 3. Manual reception activation is performed here but DW1000 offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 4. We use polled mode of operation here to keep the example as simple as possible but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW1000 User Manual for more details on "interrupts".
 * 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
