
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

 #include "stm32f4xx.h"

uint8_t uart1_data;
uint8_t freqflag=0;


void RCC_Configuration(void);
void GPIO_Configuration(void);
void LED_Initialization(void);
void LED3_Toggle(void);
void USART1_Configuration(void);
void USART1_puts(char* s);
void SPI1_init(void);
uint8_t SPIx_transmit(SPI_TypeDef* SPIx, uint16_t data);
void ms5611_reset(void);
void ms5611_conver_adc_send(uint16_t Dx);
uint32_t ms5611_conver_adc_read(void);
void ms5611_read_prom(uint16_t prom);
void ms5611_calibrate(uint16_t samplenum);
void ms5611_calculate(void);
float baro_to_altitude(float P, float T);
void ms5611_init(void);
void Timer5_Initialization(void);

static inline void Delay_1us(uint32_t);
static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 45; nCnt != 0; nCnt--);
}

#endif /* __MAIN_H */


