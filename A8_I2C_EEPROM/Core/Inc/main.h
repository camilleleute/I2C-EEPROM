#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

#define BYTE 8
#define BYTE_MASK 0xFF
// magic timing number from THE TOOL.
#define I2C_TIMING 0x0E14
// Base I2C/EEPROM address
#define ADDRESS (0x51 << 1)
void EEPROM_init(void);
uint8_t EEPROM_read(uint16_t address);
void EEPROM_write(uint16_t address, uint8_t data);
void SysTick_Init(void);
void LED_init(void);
void delay_us(const uint32_t time_us);

void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
