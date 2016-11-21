#ifndef __WARM_START_H
#define __WARM_START_H
#include "stm32l1xx_hal.h"
#include <stdint.h>

#define em7180_HOST_CTL				0x34
#define em7180_PARAM_ACK			0x3A
#define em7180_PARAM_READ_LSB		        0x3B
#define em7180_ALG_CTL				0x54
#define em7180_PARAM_WRITE_LSB		        0x60
#define em7180_PARAM_REQUEST		        0x64
#define WARM_START_DATA_VALID 			0x2a65
#define em7180_PRAM_CLT_PARAM_TRANSFER         0x80

int32_t warm_start(void);
void save_gbias_params();
void save_mag_params();
  

union dataConverter {
  int i;
  float f;    
};


#endif
