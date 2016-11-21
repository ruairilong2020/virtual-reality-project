#ifndef LED_H
#define LED_H

#include "stm32l1xx.h"
#define RED_LED 	2
#define GREEN_LED       3
#define BLUE_LED	4
#define LED_GPIO_Port	GPIOC

void LED_init(void);
void LED_on(uint16_t LEDx);
void LED_off(uint16_t LEDx);


#endif