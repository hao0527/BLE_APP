
#ifndef __RF_SPI_H__
#define __RF_SPI_H__



#include<stdint.h>

void RF_SPI_init(void);

uint8_t PN102_SPIWrite(uint8_t reg,uint8_t value);


uint8_t PN102_SPIRead(uint8_t reg);


#endif

