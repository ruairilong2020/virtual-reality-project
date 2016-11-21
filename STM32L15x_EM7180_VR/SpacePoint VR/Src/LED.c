/**
******************************************************************************
* File Name          : firmware.c
* Description        : specifies the location in the MCU's flash memory of the 
* sensor hub's firmware image which is loaded after a power on reset.
******************************************************************************
*
* COPYRIGHT(c) 2016 Ruairi Long gitruairilong@gmail.com
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

/*******************************************************************************
* File Name          : LED.c
* Author             : RL
* Date               : April 21 2016
* Description        : LED Functions for VR board
********************************************************************************/

#include "LED.h"
#include "common.h"
uint8_t LED_flag =0;
/**LED_on
* Pass   :      LED number
* Return :      nothing
*
*       Turns on specified LED
**/

void LED_on(uint16_t LEDx){
  LED_flag  |= (1 << LEDx);
  HAL_GPIO_WritePin(LED_GPIO_Port,LED_flag, GPIO_PIN_RESET);
  
}

/**LED_off
* Pass   :      LED number
* Return :      nothing
*
*       Turns off specified LED
**/

void LED_off(uint16_t LEDx){
  
  LED_flag &= ~(1 << LEDx);
  HAL_GPIO_WritePin(LED_GPIO_Port, (LEDx), GPIO_PIN_SET);
  
}



