/**
******************************************************************************
* File Name          : sentral.c
* Description        : Functions for initializing and communicating with the sensor fusion hub.
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
/*******************************************************************************
* File Name          : sentral.c
* Author             : RL
* Version            : 
* Date               : 03-21-16
* Description        : Functions for initializing and communicating with sentral.
********************************************************************************/

#include "common.h"
static DI_ERROR_CODE_T reset_sentral();
extern const unsigned char rawData[MAX_FIRMWARE_SIZE];

/*******************************************************************************
* Function Name  : data_upload
* Description    : Uploads four bytes of data fromt the firmware image to sentral's RAM
* Input          : void
* Return         : status
*******************************************************************************/ 

DI_ERROR_CODE_T data_upload(void) {
  
  DI_ERROR_CODE_T status = DE_NONE;
  uint32_t i = 0, num;
  uint8_t data[4];
  
  /* check if the firmware image stored in flash memory is valid. Magic number = 0x2a65 */
  if (rawData[0] != 0x2a && rawData[1] != 0x65)
    return DE_INVALID_FILE_HEADER;
  
  num = (rawData[13] << 8) + rawData[12] + 16;
  
  for (i = RAWDATA_OFFSET; i < num; i += 4) {
  
    for (int k = 3; k > -1; k--) {
      data[k] = rawData[i+k-3];
    }
    
    status |= SentralI2C_MultiByteWrite(UPLOAD_DATA_START_REG, data, 4);
  }

  return status;
}

/*******************************************************************************
* Function Name  : firmware_upload
* Description    : Uploads a sentral firmware image from flash memory
* Input          : void
* Return         : status of firmware upload
*******************************************************************************/ 

DI_ERROR_CODE_T firmware_upload(void) {
  
  uint8_t crc32[4] = {0};
  DI_ERROR_CODE_T status = DE_NONE;
  status |= SentralI2C_WriteReg(RESET_REQ_REG, 0x01);
  HAL_Delay(100);
  status |= SentralI2C_WriteReg(HOST_CONTROL, ENABLE_UPLOAD);
  status |= data_upload();
  
  if (status != DE_NONE)
    return status;

  status |= SentralI2C_ReadMultiReg(SENTRAL_CRC_BEGIN, crc32, 4); 
  /* check if the CRC computed by sentral matches the CRC in the fw image */
  if ((crc32[0] == rawData[4]) && (crc32[1] == rawData[5]) &&
     (crc32[2] == rawData[6]) && (crc32[3] == rawData[7]) ) {
    return DE_NONE;
  } else {
    return DE_INVALID_CRC;
  }
   
}

/*******************************************************************************
* Function Name  : reset_sentral
* Description    : Sends a reset command to sentral and checks the status of sentral
*                  this function calls Setup_Sentral after sentral successfully boots from the EEPROM
* Return         : status
*******************************************************************************/ 

DI_ERROR_CODE_T reset_sentral() {
  
  DI_ERROR_CODE_T status = DE_NONE;
  uint8_t returnedByte;
  
  status |= SentralI2C_WriteReg(RESET_REQ_REG, 0x01);
  /* give sentral enough time after a reset to scan for eeproms on the master bus */
  HAL_Delay(20); 
  
  if (SentralI2C_ReadMultiReg(SENTRAL_STATUS_REG, &returnedByte, 1) == DE_NONE) {
  
    if (!(returnedByte & 0x01))
      return DE_NONE;
    else
      /* the EEPROM on the M&M should be erased so we will treat this as an error */
      return DE_EEPROM_PRESENT;  
  }

  else {
  /* error occured reading from the status register */
  printf("I2C ERROR in function %s\n", __func__);
  return DE_I2C_ERROR;
  }  
  /* should not get here */
  return DE_UNKOWN_ERROR;
}

/*******************************************************************************
* Function Name  : setup_sentral
* Description    : Enabales events, sets sensor rates and commands CPU
*                  to run.
* Return         : status
*******************************************************************************/ 

int32_t setup_sentral() {
  
  DI_ERROR_CODE_T status = reset_sentral();
  status |= firmware_upload();  
  
  if (status != DE_NONE) {
    printf("Error occured during sentral bootup, status code = %d \r\n", status); 
    goto LEDTURNON;
  }
  
  status |= SentralI2C_WriteReg(HOST_CONTROL_REG, 0x01);  // request CPU to run 
  HAL_Delay(100);
  status |= warm_start();   
  /* enable quaternion and error events */
  status |= SentralI2C_WriteReg(ENABLE_EVENTS_REG, QUATERNION_EVENT | ERROR_EVENT);  
  status |= SentralI2C_WriteReg(MAG_RATE_REG, TARGET_MAG_RATE);  // set mag rate
  status |= SentralI2C_WriteReg(ACCEL_RATE_REG, TARGET_ACCEL_RATE);  // set accel rate
  status |= SentralI2C_WriteReg(GYRO_RATE_REG, TARGET_GYRO_RATE);  // set gyro rate 
  
LEDTURNON:  
  switch (status) {
    
  /* everything's OK */  
  case DE_NONE:
    LED_on(BLUE_LED);
    break; 
  /* communication problem or no firmware image in flash */  
  case DE_I2C_ERROR:
  case DE_INVALID_FILE_HEADER:
    LED_on(RED_LED);
    break;
  /* problem occured during warm start */
  case DE_INVALID_WARMSTART_DATA:
  case DE_WARMSTART_TIMEOUT:
    LED_on(GREEN_LED);
    break;
  /* EEPROM present, turn tri color LED yellow. We will treat this as an error */  
  case DE_EEPROM_PRESENT:
    LED_on(GREEN_LED);
    LED_on(RED_LED);    
    break;
  /* multiple errors */  
  default:
    LED_on(BLUE_LED);
    LED_on(RED_LED);
    LED_on(GREEN_LED);
    break;
  } 
  return status;
}
