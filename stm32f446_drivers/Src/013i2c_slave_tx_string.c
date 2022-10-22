/*
 *
 *  Created on: Oct 18, 2022
 *      Author: phanb
 */
#include <string.h>
#include "stm32f446.h"
#include <stdio.h>

#define SLAVE_ADDR 0x69
#define MY_ADDR SLAVE_ADDR
void delay(void){
	for(uint32_t i=0; i<500000/2; i++);
}

I2C_Handle_t I2C1Handle;

//receive buffer
uint8_t Tx_buf[32] = "STM32 slave mode testing";

/*
 * PB6 -> SCL
 * PB9 ->SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOBaseAddr = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;
	GpioLed.pGPIOBaseAddr = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

//	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOBaseAddr = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

int main(void)
{

	GPIO_ButtonInit();
	//i2c pin init
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//I2C IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);
	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE); //PE=1

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);


	while(1);

}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	static uint8_t count = 0;
	static uint8_t commandCode = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data, slave has to send it
		if(commandCode == 0x51)
		{
			//send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));
		}else if(commandCode == 0x52)
		{
			//send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[count++]);
		}
	}else if(AppEv == I2C_EV_DATA_RCV)
	{
		//data is waiting for the slave to read, slave has to read it
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if(AppEv == I2C_ERROR_AF)
	{
		//this happens only during slave txing
		//master has sent the NACK so slave should understand that master does not need more data
		commandCode = 0xff;
		count = 0;
	}else if(AppEv == I2C_EV_STOP)
	{
		//this happens only during slave reception
		//master has ended the I2C communication with the slave


	}
}
