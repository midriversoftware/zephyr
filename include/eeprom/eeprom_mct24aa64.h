#ifndef EEPROM_MCT24AA64_H_
#define EEPROM_MCT24AA64_H_

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define EEPROM_SIZE_KBITS       64
#define EEPROM_SIZE   ((EEPROM_SIZE_KBITS*1024)/8)       
#define EEPROM_MAX_ADDRESS  (EEPROM_SIZE-1)

int eepromInit();
int eepromRead( uint16_t address, uint8_t *dest, int bytecnt);
int eepromWrite(uint16_t address, uint8_t *src,  int bytecnt);



#endif