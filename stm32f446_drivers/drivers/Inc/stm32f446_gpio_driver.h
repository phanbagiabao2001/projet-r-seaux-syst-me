/*
 * stm32f446_gpio_driver.h
 *
 *  Created on: Jul 26, 2022
 *      Author: phanb
 */

#ifndef INC_STM32F446_GPIO_DRIVER_H_
#define INC_STM32F446_GPIO_DRIVER_H_


#include "stm32f446.h"


typedef struct
{
	uint8_t GPIO_PinNumber;			//possible values from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;			//possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;	//possible values from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;			//possible values from @GPIO_PIN_OP
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

//this is a handle structure for a GPIO pin
typedef struct
{
	GPIO_RegDef_t *pGPIOBaseAddr;	//it holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	//it holds GPIO pin configuration settings
}GPIO_Handle_t;


// @GPIO_PIN_NUMBER
// GPIO pin possible number
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15	 	15


// @GPIO_PIN_MODES
// GPIO pin possible modes
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT 	4
#define GPIO_MODE_IT_RT 	5
#define GPIO_MODE_IT_RFT 	6


// @GPIO_PIN_OP
// GPIO pin possible output types
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


// @GPIO_PIN_SPEED
// GPIO pin possible output speeds
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


// @GPIO_PIN_PUPD
//GPIO pin pull-up and pull-down configuration macros
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2


/********************************************************************************************************************************
 * 					APIs supported by the driver for more information about the APIs check the function definitions
 ********************************************************************************************************************************/

//peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Init and de-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//data read write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber); //type uint8 or boolean because the base addr of the GPIO
																	// peripheral and the return value must be either 0 or 1
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutPutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutPutPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutPutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint64_t IRQPriority);


#endif /* INC_STM32F446_GPIO_DRIVER_H_ */
