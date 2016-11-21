/**
******************************************************************************
* File Name          : warmstart.c
* Description        : Functions for saving and loading warm start parameters in to the sensor fusion hub.
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
********************************************************************************/

/*******************************************************************************
* File Name          : warmstart.c
* Author             : RL
* Version            : 
* Date               : 04-21-16
* Description        : Functions for uploading and saving warm start parameters
* to and from the sensor fusion hub.
********************************************************************************/

#include "common.h"
#include "warmstart.h"
//#define DEBUG_WARM_START
extern uint32_t crc32(uint32_t crc_val, uint32_t data_val);

uint8_t gbiasParams[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 33, 34, 35};
uint8_t magParams[] = {1, 2, 18, 19, 20, 21, 22, 23};
bool gbiasSaved = FALSE, magSaved = FALSE, coldStart = FALSE;
uint8_t warmStartParams[sizeof(gbiasParams) + sizeof(magParams)];
float warmStartValues[sizeof(warmStartParams)];
#define em7180_NUM_WARM_START_PARAMS (sizeof(gbiasParams) + sizeof(magParams))

/*******************************************************************************
* Function Name  : em7180_parameter_read
* Description    : read a  single parameter value from sentral via param IO
* Return         : status
*******************************************************************************/

static int32_t em7180_parameter_read(float *values, const uint8_t *paramNo, const uint8_t numParams) {
  int32_t status = DE_NONE, warmStartAttemptCounter;
  // Start param io session
  uint8_t paramIoReq = em7180_PRAM_CLT_PARAM_TRANSFER;
  status |= SentralI2C_MultiByteWrite(em7180_ALG_CTL, &paramIoReq, 1);
  uint8_t i;
  
  for (i = 0; i < numParams; i++) {
    // Request parameter
    uint8_t paramNumber = paramNo[i];
    status |= SentralI2C_MultiByteWrite(em7180_PARAM_REQUEST, &paramNumber, 1);    
    uint8_t value = 0xFF;
    warmStartAttemptCounter = 0;
    
    do {
      // Check for parameter acknowledge
      status |= SentralI2C_ReadMultiReg(em7180_PARAM_ACK, &value, 1);
      HAL_Delay(1);
      
      if (++warmStartAttemptCounter >= MAX_WARMSTART_ACK_ATTEMPTS)
        return DE_WARMSTART_TIMEOUT;
      
    } while (value != paramNo[i]);
    // Read parameter value
    uint8_t buf[4];
    status |= SentralI2C_ReadMultiReg(em7180_PARAM_READ_LSB, buf, 4);
    float* paramValue = (float*)buf;
    values[i] = paramValue[0];
  } 
  // End param io session
  uint8_t zero = 0x00;
  status |= SentralI2C_MultiByteWrite(em7180_PARAM_REQUEST, &zero, 1);
  status |= SentralI2C_MultiByteWrite(em7180_ALG_CTL, &zero, 1);
  return status;
}

/*******************************************************************************
* Function Name  : em7180_parameter_write
* Description    : writes a single parameter value to sentral via param IO
* Return         : status
*******************************************************************************/ 

static int32_t em7180_parameter_write(const float *values, const uint8_t *paramNo, const uint8_t numParams) {
  int32_t status = DE_NONE, warmStartAttemptCounter;
  union dataConverter temp;  
  // Start param io session
  uint8_t paramIoReq = em7180_PRAM_CLT_PARAM_TRANSFER;
  status |= SentralI2C_MultiByteWrite(em7180_ALG_CTL, &paramIoReq, 1);
  uint8_t i;
  
  for (i = 0; i < numParams; i++) {
    // Upload next parameter value
    temp.f = values[i];
    uint8_t* wBuf = (uint8_t*)&temp.i;
    status |= SentralI2C_MultiByteWrite(em7180_PARAM_WRITE_LSB, wBuf, 4);
    // Request parameter load and specify parameter number
    wBuf[0] = paramNo[i] + 0x80;
    status |= SentralI2C_MultiByteWrite(em7180_PARAM_REQUEST, wBuf, 1);    
    uint8_t value = 0xFF;
    warmStartAttemptCounter = 0;
    
    do {
      // Check for parameter acknowledge
      status |= SentralI2C_ReadMultiReg(em7180_PARAM_ACK, &value, 1);
      HAL_Delay(1);
      
      if (++warmStartAttemptCounter >= MAX_WARMSTART_ACK_ATTEMPTS)
        return DE_WARMSTART_TIMEOUT;
      
    } while ((value != paramNo[i] + 0x80));
  }
  // End param io session
  uint8_t zero = 0x00;
  status |= SentralI2C_MultiByteWrite(em7180_PARAM_REQUEST, &zero, 1);
  status |= SentralI2C_MultiByteWrite(em7180_ALG_CTL, &zero, 1);  
  return status;
}

/*******************************************************************************
* Function Name  : save_data_flash
* Description    : strores the warm start data in flash
* Return         : status
*******************************************************************************/ 

int32_t save_data_flash(float *values, const uint8_t *paramNo, const uint8_t numParams) {
  
  int32_t status = DE_NONE;
  uint32_t i;
  union dataConverter temp;  
  em7180_parameter_read(values, paramNo, numParams);  

    
  for (i = 0; i < numParams; i++) {    
    uint32_t address = WARM_START_BASE_ADDRESS_IN_FLASH + (((paramNo[i] - 1) * 4) + 2);    
#ifdef DEBUG_WARM_START
    printf("%d: %6.3f address %x \r\n", paramNo[i], values[i], address);
#endif
    temp.f = values[i];
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                    | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, temp.i);
  }
  
  return status;  
}

/*******************************************************************************
* Function Name  : warm_start
* Description    : Uploads warm start parameters on bootup from the MCU's flash 
* Return         : status
*******************************************************************************/ 

int32_t warm_start() {
  
  int32_t status = DE_NONE;
  static uint32_t i;
  union dataConverter temp;   
  HAL_FLASH_Unlock();
  memcpy(warmStartParams, gbiasParams, sizeof(gbiasParams));
  memcpy(&warmStartParams[sizeof(gbiasParams)], magParams, sizeof(magParams));
  temp.i=*(__IO uint32_t*)(WARM_START_BASE_ADDRESS_IN_FLASH);  
  
  if (temp.i != WARM_START_DATA_VALID) {
#ifdef DEBUG_WARM_START
    printf("Warm Start data invalid \n");
#endif
    coldStart = TRUE;
    return DE_INVALID_WARMSTART_DATA;
  }
  else {
#ifdef DEBUG_WARM_START
    printf("Warm Start data valid \n");
#endif
  }  
  
  for (i = 0; i < sizeof(warmStartParams); i++) {
    
    uint32_t address = WARM_START_BASE_ADDRESS_IN_FLASH + (((warmStartParams[i] - 1) * 4) + 2);
    temp.i =*(__IO uint32_t*)(address);
    warmStartValues[i] = temp.f;
#ifdef DEBUG_WARM_START
    printf("param %d: value %6.3f address %x\n", warmStartParams[i], warmStartValues[i],address);
#endif
  }  

  status |= em7180_parameter_write(warmStartValues, warmStartParams, em7180_NUM_WARM_START_PARAMS);  
  
  if ( status == DE_NONE)
    coldStart = FALSE; 
  else {
    coldStart = TRUE;
  }    
    
  HAL_FLASH_Lock();
  return status;
}

/*******************************************************************************
* Function Name  : save_gbias_params
* Description    : reads and saves gyro related warm start data
* Return         : status
*******************************************************************************/ 

void save_gbias_params() {
  
  gbiasSaved = TRUE;
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                    | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
  

  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WARM_START_BASE_ADDRESS_IN_FLASH, 0x0000);
#ifdef DEBUG_WARM_START
  printf("Saving gbias parameters \r\n");
#endif
  float temp[sizeof(gbiasParams)];
  int32_t status = save_data_flash(temp, gbiasParams, sizeof(gbiasParams));
  
  if(magSaved == TRUE && status == DE_NONE && coldStart == TRUE) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WARM_START_BASE_ADDRESS_IN_FLASH, WARM_START_DATA_VALID);
    LED_on(BLUE_LED);
  }
  else if (status == DE_NONE && coldStart == FALSE) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WARM_START_BASE_ADDRESS_IN_FLASH, WARM_START_DATA_VALID);
  }
  
  HAL_FLASH_Lock();
}

/*******************************************************************************
* Function Name  : save_mag_params
* Description    : reads and saves mag related warm start data
* Return         : status
*******************************************************************************/ 

void save_mag_params() {
  
  magSaved = TRUE;
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
                    | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WARM_START_BASE_ADDRESS_IN_FLASH, 0x0000);
#ifdef DEBUG_WARM_START
  printf("Saving mag parameters \r\n");
#endif  
  float temp[sizeof(magParams)];
  int32_t status = save_data_flash(temp, magParams, sizeof(magParams));
  
  if(gbiasSaved == TRUE && status == DE_NONE && coldStart == TRUE) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WARM_START_BASE_ADDRESS_IN_FLASH, WARM_START_DATA_VALID);
    LED_on(BLUE_LED);
  }
  else if (status == DE_NONE && coldStart == FALSE) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WARM_START_BASE_ADDRESS_IN_FLASH, WARM_START_DATA_VALID);
  }  
  
  HAL_FLASH_Lock();
}

