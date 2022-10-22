/*
 * stm32f446_gpio_driver.c
 *
 *  Created on: Jul 26, 2022
 *      Author: phanb
 */

#include "stm32f446_gpio_driver.h"

/*
		Peripheral clock setup
*/
/**************************************************************************************
 * @fn				- GPIO_PeriClockControl
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[1]		- Base address of the GPIO peripheral
 * @param[2]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA){
			GPIOA_PLCK_EN();
		}else if (pGPIOx == GPIOB){
			GPIOB_PLCK_EN();
		}else if (pGPIOx == GPIOC){
			GPIOC_PLCK_EN();
		}else if (pGPIOx == GPIOD){
			GPIOD_PLCK_EN();
		}else if (pGPIOx == GPIOE){
			GPIOE_PLCK_EN();
		}else if (pGPIOx == GPIOF){
			GPIOF_PLCK_EN();
		}else if (pGPIOx == GPIOG){
			GPIOG_PLCK_EN();
		}else if (pGPIOx == GPIOH){
			GPIOH_PLCK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PLCK_DI();
		}else if (pGPIOx == GPIOB){
			GPIOB_PLCK_DI();
		}else if (pGPIOx == GPIOC){
			GPIOC_PLCK_DI();
		}else if (pGPIOx == GPIOD){
			GPIOD_PLCK_DI();
		}else if (pGPIOx == GPIOE){
			GPIOE_PLCK_DI();
		}else if (pGPIOx == GPIOF){
			GPIOF_PLCK_DI();
		}else if (pGPIOx == GPIOG){
			GPIOG_PLCK_DI();
		}else if (pGPIOx == GPIOH){
			GPIOH_PLCK_DI();
		}
	}
}
/*
		Init and de-init
*/
/**************************************************************************************
 * @fn				- GPIO_Init
 * @brief			-
 *
 * @param[1]		-
 * @param[2]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOBaseAddr, ENABLE);

	//1. configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOBaseAddr->MODER &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
																	//clear. we put 00 so it is ~11 meaning that ~0x3
		pGPIOHandle->pGPIOBaseAddr->MODER |= temp;

	}else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure the Falling trigger selection register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure the Rising trigger selection register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. Configure both
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIOA_BASEADDR_TO_CODE(pGPIOHandle->pGPIOBaseAddr);
		SYSCFG_PLCK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		// 3. Enable the exit interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOBaseAddr->OSPEEDR &= ~(0x3 << 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOBaseAddr->OSPEEDR |= temp;

	temp = 0;

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOBaseAddr->PUPDR &= ~(0x3 << 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOBaseAddr->PUPDR |= temp;

	temp = 0;

	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOBaseAddr->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOBaseAddr->OTYPER |= temp;

	temp = 0;

	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOBaseAddr->AFR[temp1]  &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOBaseAddr->AFR[temp1]  |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}
}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}

	/*
			Data read write
	*/
	/**************************************************************************************
	 * @fn				- GPIO_ReadFromInputPin
	 * @brief			-
	 *
	 * @param[1]		-
	 * @param[2]		-
	 *
	 * @return			- 0 or 1
	 *
	 * @note			- none
	 *
	 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){ //type uint8 or boolean because the base addr of the GPIO
																	// peripheral and the return value must be either 0 or 1

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
/**************************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 * @brief			-
 *
 * @param[1]		-
 * @param[2]		-
 *
 * @return			- 0 or 1
 *
 * @note			- none
 *
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
/**************************************************************************************
 * @fn				- GPIO_WriteToOutPutPin
 * @brief			-
 *
 * @param[1]		-
 * @param[2]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_WriteToOutPutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1<<PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
/**************************************************************************************
 * @fn				- GPIO_WriteToOutPutPort
 * @brief			-
 *
 * @param[1]		-
 * @param[2]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_WriteToOutPutPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}
/**************************************************************************************
 * @fn				- GPIO_ToggleOutPutPin
 * @brief			-
 *
 * @param[1]		-
 * @param[2]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_ToggleOutPutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

//IRQ configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32) );
		}else if(IRQNumber >= 64 && IRQNumber < 96)	//64 to 95
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32) );
		}else if(IRQNumber >= 64 && IRQNumber < 96)	//64 to 95
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64) );
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint64_t IRQPriority)
{
	//1. First lets find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8* iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}

