/*
 * 001LEDToggle.c
 *
 *  Created on: Jul 29, 2022
 *      Author: phanb
 */
#include "stm32f446.h"
#include "stm32f446_gpio_driver.h"

void delay(void){
	for(uint32_t i=0; i<500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOBaseAddr = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;  //push pull configuration
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; //no_pupd because OP_TYPE is push pull

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutPutPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
