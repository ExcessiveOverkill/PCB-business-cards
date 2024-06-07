#include "ledDrivers.h"


void disableLEDs(){
	// force all outputs off
	HAL_GPIO_WritePin(LED_BLANK_GPIO_Port, LED_BLANK_Pin, GPIO_PIN_SET);
}

void enableLEDs(){
	// enable outputs
	HAL_GPIO_WritePin(LED_BLANK_GPIO_Port, LED_BLANK_Pin, GPIO_PIN_RESET);
}

void setLEDs(uint32_t data){
	// write data to leds

	uint32_t tempData = data;

	for (int i=0; i < 32; i++){

		/*
		GPIO_PinState dataPinState = (tempData & 0b1) ? GPIO_PIN_SET : GPIO_PIN_RESET;

		// write to data pin
		HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, dataPinState);

		// cycle clock to increment data
		HAL_GPIO_WritePin(LED_DATA_CLOCK_GPIO_Port, LED_DATA_CLOCK_Pin, GPIO_PIN_SET);
		// might need a delay here or slower GPIO clock
		HAL_GPIO_WritePin(LED_DATA_CLOCK_GPIO_Port, LED_DATA_CLOCK_Pin, GPIO_PIN_RESET);

		tempData = tempData >> 1;
		*/

		if(tempData & 0b1){
			LED_DATA_GPIO_Port->ODR |= (1 << 8);
		}
		else{
			LED_DATA_GPIO_Port->ODR &= ~(1 << 8);
		}

		LED_DATA_CLOCK_GPIO_Port->ODR |= (1 << 7);
		LED_DATA_CLOCK_GPIO_Port->ODR &= ~(1 << 7);

		tempData = tempData >> 1;
	}

	// cycle latch to update outputs
	HAL_GPIO_WritePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin, GPIO_PIN_SET);
	// might need a delay here or slower GPIO clock
	HAL_GPIO_WritePin(LED_DATA_LATCH_GPIO_Port, LED_DATA_LATCH_Pin, GPIO_PIN_RESET);

}
