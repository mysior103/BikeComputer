/******************************************************************************
**************************Hardware interface layer*****************************
* | file      	:	DEV_Config.c
* |	version		:	V1.0
* | date		:	2017-08-14
* | function	:	
	Provide the hardware underlying interface	
******************************************************************************/
#include <DEV_Config.h>
#include "stm32l0xx_hal_spi.h"
#include "main.h"
#include <stdio.h>		//printf()
#include <string.h>
#include <stdlib.h>

/********************************************************************************
function:	Hardware interface
	SPI4W_Write_Byte(value) : 
		HAL library hardware SPI
********************************************************************************/
void SPI4W_Write_Byte(uint8_t value)
{
    HAL_SPI_Transmit(&hspi1, &value, 1, 500);
}

/********************************************************************************
function:	Delay function
note:
	Driver_Delay_ms(xms) : Delay x ms
	Driver_Delay_us(xus) : Delay x us
********************************************************************************/
void Driver_Delay_ms(uint32_t xms)
{
    HAL_Delay(xms);
}

void Driver_Delay_us(uint32_t xus)
{
    for(int j=xus; j > 0; j--);
}
