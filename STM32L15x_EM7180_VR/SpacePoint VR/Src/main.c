/**
******************************************************************************
* File Name          : main.c
* Description        : Main program body
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
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "usb_device.h"
#include "stm32l1xx_hal_gpio.h"
#include "common.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
extern uint8_t *USBD_PRODUCT_STRING_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t USBD_HID_SendReport     (USBD_HandleTypeDef  *pdev, 
                                        uint8_t *report,
                                        uint16_t len);
extern short SentralI2C_ReadMultiReg(unsigned char regaddr, unsigned char *regcontent, unsigned char RegCount);

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{  
  uint8_t usbBuffer[64];
  static uint8_t eventStatusRegisterValue, warmStartFlags, gBiasTimeout = TRUE, magTimeout = TRUE;
  static uint32_t gBiasTimeoutStart, timeSinceLastTrigger, magTimeoutStart, gBiasTriggerCount, currentTime;  
  /* MCU Configuration----------------------------------------------------------*/  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();  
  /* Configure the system clock */
  SystemClock_Config();  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  
  MX_USART1_UART_Init();  
  /* USER CODE BEGIN 2 */
  setup_sentral(); 
  /* USER CODE END 2 */  
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {    
    currentTime = ((uint32_t) HAL_GetTick()) / 1000;
    
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
      SentralI2C_ReadMultiReg(WARMSTART_FLAGS_REGISTER, &warmStartFlags, 1);     
      /*
      gBiasUpdate
      Note the ready flag will trigger approx every 2 sec when still. 
      This method should result in the best possible gyro bias being saved, 
      without excessive param saves.
      1)	Save on n=2 triggers in a row.
      2)	For n > 2 triggers, at end of still period (period of repeated ready triggers),
      perform another save.
      a)	This will require checking the timing of the ready triggers. 
      b)	When still, the triggers come in every ~2 sec. 
      c)	If ~4 sec have gone by since the last trigger, then perform a save.
      3)	Wait for timeout (1 min)
      4)	Go back to (1) */
      
      /* 3 */  
      if (gBiasTimeout == FALSE && ((currentTime - gBiasTimeoutStart) >  60)) {
        gBiasTimeout = TRUE;
        gBiasTriggerCount = 0;
        gBiasTimeoutStart = currentTime;   
      }
      
      if (CHECK_BIT(warmStartFlags,WARMSTART_GBIAS_BIT)) {           
        gBiasTriggerCount++;
        
        /* 1 */       
        if (gBiasTriggerCount == 2) {
          save_gbias_params();
          gBiasTimeoutStart = currentTime;
        }  
        
        /* 2 */       
        if (gBiasTriggerCount > 2 && (currentTime - timeSinceLastTrigger) > 4 &&  gBiasTimeout == TRUE) {
          save_gbias_params();
          gBiasTimeout = FALSE;
          gBiasTimeoutStart = currentTime;
        } 
        
        timeSinceLastTrigger = currentTime;
      }
      
      /*
      magCalUpdate
      Note ready flag will trigger for initial cal complete or a bg cal update.
      1)	Save on first trigger.
      2)	Track if new trigger comes in.
      3)	Wait for timeout (1-2 min)
      4)	If 1 or more triggers have occurred since last save, then perform new save.
      5)	Go to (2) */
      
      /* 3 */       
      if (magTimeout == FALSE && ((currentTime - magTimeoutStart) >  60)) {
        magTimeout = TRUE;
      }   
      
      /* 1, 2, 4 */
      if (CHECK_BIT(warmStartFlags, WARMSTART_MAG_BIT) && magTimeout == TRUE) { 
        magTimeout = FALSE;
        magTimeoutStart = currentTime;
        save_mag_params();  
      }
      
      /* Read in quaternions and timestamps */
      SentralI2C_ReadMultiReg(0x00, usbBuffer, 18);
      /* Unlatch the sentral interrupt by reading from the event status register. */
      SentralI2C_ReadMultiReg(EVENT_STATUS_REGISTER, &eventStatusRegisterValue, 1);       
      USBD_HID_SendReport(&hUsbDeviceFS, usbBuffer, sizeof(usbBuffer));      
    }  
  }
  /* USER CODE END 3 */
  
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  
  __HAL_RCC_PWR_CLK_ENABLE();
  
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
  
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  
}

/** Configure pins as 
* Analog 
* Input 
* Output
* EVENT_OUT
* EXTI
*/
static void MX_GPIO_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  
  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PC4 PC3 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIOC->BSRR = 0x1f;   
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
  
}

#endif

/**
* @}
*/ 

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
