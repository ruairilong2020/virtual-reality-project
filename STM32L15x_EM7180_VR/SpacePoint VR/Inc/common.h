#pragma once 
#include "warmstart.h"
#include "LED.h"
#include "sentral.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

extern short SentralI2C_ReadMultiReg(unsigned char regaddr, unsigned char *regcontent, unsigned char RegCount);
extern short SentralI2C_MultiByteWrite(unsigned char regaddr, unsigned char *regcontent, unsigned char size);
extern short SentralI2C_WriteReg(unsigned char regaddr, unsigned char regcontent);

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define WARM_START_BASE_ADDRESS_IN_FLASH           ((uint32_t)0x08080000) 
#define WARMSTART_GBIAS_BIT 1
#define WARMSTART_MAG_BIT 0
#define WARMSTART_FLAGS_REGISTER 0x4f
#define MAX_I2C_BUFFER_SIZE 100
#define DEFAULT_I2C_SPEED_KHZ 400
#define MAX_FIRMWARE_SIZE 25000
#define ENU_MODE 0x20
#define ENABLE_UPLOAD 0x02
#define UPLOAD_DATA_START_REG		0x96
#define SENTRAL_ADDRESS 0x50
#define ENABLE_EVENTS_REG 0x33
#define HOST_CONTROL 	0x34
#define EVENT_STATUS_REGISTER 0x35
#define MAG_RATE_REG 0x55
#define ACCEL_RATE_REG 0x56
#define GYRO_RATE_REG 0x57
#define HOST_CONTROL_REG 0x34
#define SENTRAL_STATUS_REG 0x37
#define PASSTHROUGH_CONFIG_REG 0xa0
#define RESET_REQ_REG 0x9b
#define SENTRAL_CRC_BEGIN   0x97
#define REVISION_ID_REG 0x91
#define HOST_STATUS_REG 0x92
#define ALGORITHM_CONTROL 0x54
#define CAL_STATUS_REG 0x4f
#define EEPROM_NOT_PRESENT 0xff
#define RAWDATA_OFFSET 16

#define RESET_EVENT     ``0x01
#define ERROR_EVENT       0x02
#define QUATERNION_EVENT  0x04
#define MAG_EVENT         0x08
#define ACCEL_EVENT       0x10
#define GYRO_EVENT        0x20

#define MAX_WARMSTART_ACK_ATTEMPTS 1000
#define BOOT_TIMEOUT_COUNTS 30
#define WARM_START_SAVE_TIME 30
#define TARGET_MAG_RATE 100
#define TARGET_ACCEL_RATE 110/10
#define TARGET_GYRO_RATE  50



typedef enum DI_ERROR_CODE
{
      DE_NONE,
      DE_INITIALIZATION_REQUIRED,                              
      DE_INVALID_PARAMETERS,                                   
      DE_SENSING_REQUIRED,                                     
      DE_SENSOR_NOT_PRESENT,               
      DE_RATE_TOO_HIGH,                                        
      DE_I2C_PENDING,   
      DE_NOT_CALIBRATED,      
      DE_I2C_ERROR,                                            
      DE_I2C_PASSTHRU_ERROR,                                   
      DE_FILE_OPEN_ERROR,                                      
      DE_FILE_READ_ERROR,                                      
      DE_INVALID_FILE_HEADER,                                  
      DE_INCORRECT_ROM_VERSION,                                
      DE_INVALID_FILE_LENGTH,                                  
      DE_INVALID_CRC,                                          
      DE_UNEXPECTED_RESET,                                     
      DE_CHIP_ERROR,                                                
      DE_MISSING_SENSOR,                                            
      DE_UNKNOWN_SENSOR,                                            
      DE_PARAM_OUT_OF_RANGE,                                        
      DE_PARAM_ACK_TIMEOUT,                                         
      DE_SENSOR_SAMPLE_TIMEOUT,                                     
      DE_PASSTHROUGH_TIMEOUT,                                       
      DE_INCORRECT_SENSOR,                                          
      DE_BOOT_TIMEOUT,
      DE_EEPROM_PRESENT,
      DE_EPROM_NOT_PRESENT,  
      DE_INVALID_WARMSTART_DATA,
      DE_WARMSTART_TIMEOUT,                                         
      DE_I2C_TRANSACTION_TOO_LARGE,                                       
      DE_UNKOWN_ERROR,                                              
} DI_ERROR_CODE_T;

#define RED_LED 	2
#define GREEN_LED       3
#define BLUE_LED	4

#ifndef bool 
typedef uint8_t bool;
#endif
#ifndef TRUE 
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

 



