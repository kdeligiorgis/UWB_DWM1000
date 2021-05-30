/*
 * my_functions.c
 *
 *  Created on: Jan 9, 2021
 *      Author: kostasdeligiorgis
 */

#include <DWM_functions.h>
#include "main.h"
#include "deca_device_api.h"
#include "deca_regs.h"


/*! ----------------------------------------------------------------------------
 * @file    deca_sleep.c
 * @brief   platform dependent sleep implementation
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
__INLINE void deca_sleep(unsigned int time_ms)
{
	Sleep(time_ms);
}

/****************************************/

extern  SPI_HandleTypeDef hspi1;


/* @fn      port_set_dw1000_slowrate
 * @brief   set 2.25MHz
 *          note: hspi1 is clocked from 72MHz
 */
void port_set_dw1000_slowrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;  // 500 MBits/s
//	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;   // 1 MHz
//	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;   // 2 MHz


    if (HAL_SPI_Init(&hspi1) != HAL_OK) {

    	while(1){};
	}
    //HAL_SPI_Init(&hspi1);
}


/* @fn      port_set_dw1000_fastrate
 * @brief   set 18MHz
 *          note: hspi1 is clocked from 72MHz
 *          1Mhz
 */

void port_set_dw1000_fastrate(void)
{
//	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;    // 1  MHz
//	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;    // 2  MHz
//	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;    // 4  MHz
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;     // 8  MHz
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;     // 16 MHz
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		while(1){};
	}
	//HAL_SPI_Init(&hspi1);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success
 */
//#pragma GCC optimize ("O3")
//int writetospi(uint16_t headerLength,
//               const    uint8_t *headerBuffer,
//               uint32_t bodyLength,
//               const    uint8_t *bodyBuffer)
//{
//
//    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
//
//
//    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */
//
//    HAL_SPI_Transmit(&hspi1, (uint8_t *)&headerBuffer[0], headerLength, HAL_MAX_DELAY);    /* Send header in polling mode */
//    HAL_SPI_Transmit(&hspi1, (uint8_t *)&bodyBuffer[0], bodyLength, HAL_MAX_DELAY);        /* Send data in polling mode */
//
//    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */
//
//
//    return 0;
//} // end writetospi()

//This is a new WriteSPI form my friend Beacon.
#pragma GCC optimize ("03")
int writetospi(uint16 headerLength,
			   const uint8 *headerBuffer,
			   uint32 bodyLength,
			   const uint8 *bodyBuffer)
{
	decaIrqStatus_t stat;
	stat = decamutexon();

	uint8_t headBuf[headerLength];
	for (int i = 0; i < headerLength; ++i) {
		headBuf[i] = headerBuffer[i];
	}
	uint8_t bodyBuf[bodyLength];
	for (int i = 0; i < bodyLength; ++i) {
		bodyBuf[i] = bodyBuffer[i];
	}

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, headBuf, headerLength, 5);
//	HAL_SPI_Transmit(&hspi1, headBuf, headerLength, HAL_MAX_DELAY);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) {}

	HAL_SPI_Transmit(&hspi1, bodyBuf, bodyLength, 5);
//	HAL_SPI_Transmit(&hspi1, bodyBuf, bodyLength, HAL_MAX_DELAY);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) {}

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);

	decamutexoff(stat);
	return 0;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns 0
 */

//#pragma GCC optimize ("O3")
////int readfromspi(uint16_t headerLength,
////                uint8_t *headerBuffer,
////                uint32_t readlength,
////                uint8_t *readBuffer)
//
//int readfromspi(uint16 headerLength,
//                 uint8 *headerBuffer,
//                uint32 readlength,
//                uint8 *readBuffer)
//
//{
//    int i;
//
//    /* Blocking: Check whether previous transfer has been finished */
//    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
//
//    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */
//
//    /* Send header */
//    for(i=0; i<headerLength; i++)
//    {
//        HAL_SPI_Transmit(&hspi1, &headerBuffer[i], 1, HAL_MAX_DELAY);
//    }
//
//    /* for the data buffer use LL functions directly as the HAL SPI read function
//     * has issue reading single bytes */
//
//    while(readlength-- > 0)
//    {
//        /* Wait until TXE flag is set to send data */
//        while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET)
//        {
//        }
//
//        hspi1.Instance->DR = 0; /* set output to 0 (MOSI), this is necessary for
//        e.g. when waking up DW1000 from DEEPSLEEP via dwt_spicswakeup() function.
//        */
//
//        /* Wait until RXNE flag is set to read data */
//        while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET)
//        {
//        }
//
//        (*readBuffer++) = hspi1.Instance->DR;  //copy data read form (MISO)
//    }
//
//    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */
//
//    return 0;
//} // end readfromspi()


// This is a new READSPI form my friend Beacon.

#pragma GCC optimize ("O3")
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
	decaIrqStatus_t stat;
	stat = decamutexon();

	uint8_t headBuf[headerLength]; //make a copy
	for (int i = 0; i < headerLength; ++i)
	{
		headBuf[i] = headerBuffer[i];
	}

	for (int i = 0; i < readlength; ++i)
	{
		readBuffer[i] = 0;
	}

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, headBuf, headerLength, 5);
//	HAL_SPI_Transmit(&hspi1, headBuf, headerLength, HAL_MAX_DELAY);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) {}

	HAL_SPI_Receive(&hspi1, readBuffer, readlength, 5);
//	HAL_SPI_Receive(&hspi1, readBuffer, readlength, HAL_MAX_DELAY);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) {}

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);

	decamutexoff(stat);

	return 0;
}





#include "main.h"
#include "deca_device_api.h"
#include "port.h"
#include <DWM_functions.h>
#include <sys/types.h>
#include "stm32l4xx_hal_conf.h"



/* @fn    usleep
 * @brief precise usleep() delay
 * */
#pragma GCC optimize ("O0")
void usleep_1(useconds_t usec)
{
    int i,j;
#pragma GCC ivdep
    for(i=0;i<usec;i++)
    {
#pragma GCC ivdep
        for(j=0;j<2;j++)
        {
            __NOP();
            __NOP();
        }
    }
}


// This is mine deca reset.
void deca_reset(void)
{

	GPIO_InitTypeDef GPIO_InitStruct ;

	/* ********* */
	// make a header for the reset => KOSTAS
	/* ********* */

	// Configure DW1000 reset pin as open drain output
	GPIO_InitStruct.Pin = DW_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);


	// Pull the reset pin low for 1 ms
	HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	// Configure DW1000 reset pin as input
	GPIO_InitStruct.Pin = DW_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

	/* Wait at least 505 μs (for DWM1000 v1). 5 ms is a very conservative,
	 * allowing to support slow clocks like TCXOs
	 * Note: for DWM1000 V2 at least 1000μs (1ms) should be used.
	 * You can also read the reset pin or enable the MSLP2INIT IRQ to detect
	 * the INIT state being reached */

//	HAL_Delay(1);

	usleep_1(1);

	//put the pin back to output open-drain (not active)
	setup_DW1000RSTnIRQ(0);

	Sleep(2);


}


