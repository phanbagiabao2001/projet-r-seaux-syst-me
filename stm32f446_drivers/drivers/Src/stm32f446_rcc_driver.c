/*
 * stm32f446_rcc_driver.c
 *
 *  Created on: Oct 19, 2022
 *      Author: phanb
 */

#include "stm32f446.h"
#include "stm32f446_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};


uint32_t  RCC_GetPLLOutputClock()
{
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, systemClk;
	uint8_t clksrc, temp, ahbp, apb1p;
	clksrc = ((RCC->CFGR >> 2) & 0x3); //RCC->CFGR >> 2: bring the bit number second and third to the bit position zero and one.
						//this will mark out all other bits except the bit number zero and bit number one where out value is stored

	if(clksrc == 0)
	{
		systemClk = 16000000;
	}else if(clksrc == 1)
	{
		systemClk = 8000000;
	}else if(clksrc == 2)
	{
		systemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//for apb1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 = (systemClk / ahbp) / apb1p;
	return pclk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}
