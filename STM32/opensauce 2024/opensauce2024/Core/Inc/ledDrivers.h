/*
 * ledDrivers.h
 *
 *  Created on: May 13, 2024
 *      Author: voids
 */

#ifndef INC_LEDDRIVERS_H_
#define INC_LEDDRIVERS_H_

#include "stm32c0xx_hal.h"

#define LED_BLANK_Pin GPIO_PIN_5
#define LED_BLANK_GPIO_Port GPIOA
#define KEEP_POWER_ON_Pin GPIO_PIN_0
#define KEEP_POWER_ON_GPIO_Port GPIOB
#define LED_DATA_LATCH_Pin GPIO_PIN_5
#define LED_DATA_LATCH_GPIO_Port GPIOB
#define LED_DATA_CLOCK_Pin GPIO_PIN_7
#define LED_DATA_CLOCK_GPIO_Port GPIOB
#define LED_DATA_Pin GPIO_PIN_8
#define LED_DATA_GPIO_Port GPIOB

void disableLEDs();
void enableLEDs();
void setLEDs(uint32_t data);

#endif /* INC_LEDDRIVERS_H_ */
