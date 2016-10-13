
#include "main.h"


#define MS5611_ADDRESS         (0x77)
#define MS5611_ADC_READ        (0x00)
#define MS5611_ADC_D1          (0x00)
#define MS5611_ADC_D2          (0x10)
#define MS5611_RESET           (0x1E)
//osr
#define MS5611_CONV_D1_OSR256  (0x40)
#define MS5611_CONV_D1_OSR512  (0x42)
#define MS5611_CONV_D1_OSR1024 (0x44)
#define MS5611_CONV_D1_OSR2048 (0x46)
#define MS5611_CONV_D1_OSR4096 (0x48)
#define MS5611_CONV_D2_OSR256  (0x50)
#define MS5611_CONV_D2_OSR512  (0x52)
#define MS5611_CONV_D2_OSR1024 (0x54)
#define MS5611_CONV_D2_OSR2048 (0x56)
#define MS5611_CONV_D2_OSR4096 (0x58)
#define MS5611_READ_PROM_SETUP (0xA0)
#define MS5611_READ_PROM_C1    (0xA2)
#define MS5611_READ_PROM_C2    (0xA4)
#define MS5611_READ_PROM_C3    (0xA6)
#define MS5611_READ_PROM_C4    (0xA8)
#define MS5611_READ_PROM_C5    (0xAA)
#define MS5611_READ_PROM_C6    (0xAC)
#define MS5611_READ_PROM_CRC   (0xAE)
//osr delay_time
#define MS5611_ULTRA_HIGH_RES  (0x08)
#define MS5611_HIGH_RES        (0x06)   
#define MS5611_STANDARD        (0x04) 
#define MS5611_LOW_POWER       (0x02)
#define MS5611_ULTRA_LOW_POWER (0x00)

#define DIVIDED_POW_2_7      (78.125)
#define POW_2_8              (256)
#define DIVIDED_POW_2_8       (39.0625)
#define POW_2_15              (32768)
#define DIVIDED_POW_2_15        (0.3051757)
#define POW_2_16               (65536)
#define DIVIDED_POW_2_21         (47.6837158203)
#define POW_2_21               (2097152)
#define DIVIDED_POW_2_23         (0.00119)
#define DIVIDED_ONE_THOUSAND      (0.0001)



void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* USART1 clock enable */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
    // GPIO_InitTypeDef GPIO_InitStructure;

    // /*-------------------------- GPIO Configuration ----------------------------*/
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_Init(GPIOA, &GPIO_InitStructure);

    // /* Connect USART pins to AF */
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
}
 
/**************************************************************************************/
 
void LED_Initialization(void){

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG , ENABLE); //LED3/4 GPIO Port

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;  // LED is connected to PG13/PG14
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

}

void Timer5_Initialization(void)
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM5 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel =  TIM5_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    /* -- Timer Configuration --------------------------------------------------- */
    TIM_DeInit(TIM5);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Period = 100 - 1 ;  // Period 50000 -> 2Hz toggle
    TIM_TimeBaseStruct.TIM_Prescaler = 900 - 1;  // Prescaled by 900 -> = 100000 Hz
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Div by one -> 90 MHz
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_ARRPreloadConfig(TIM5, DISABLE);
    TIM_Cmd(TIM5, ENABLE);
}

void LED3_Toggle(void){


  GPIOG->ODR ^= GPIO_Pin_13;

}

void SPI1_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  SPI_InitTypeDef  SPI_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

  SPI_I2S_DeInit(SPI1);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//CPOL==0 | CPOL ==1
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//CPHA==1 | CPHA==2
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  SPI_Cmd(SPI1, ENABLE);
  
  
} 

uint8_t SPIx_transmit(SPI_TypeDef* SPIx, uint16_t data)
{
  uint8_t receivedata;

  SPI_I2S_SendData(SPIx,data);
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);

  receivedata = SPI_I2S_ReceiveData(SPIx);

  return receivedata;
}

uint16_t C[7]={0};
int32_t Press=0, Temp=0;
float Press_offset=0.0f, Temp_offset=0.0f;
float Press_sum=0.0f;
float Temp_sum=0.0f;
float h_init;
uint32_t D1=0,D2=0;

void ms5611_reset(void)
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  SPIx_transmit(SPI1,MS5611_RESET);
  Delay_1us(2800);
  GPIO_SetBits(GPIOA,GPIO_Pin_4);
}


void ms5611_conver_adc_send(uint16_t Dx)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);//uncompensated pressure (D1) & uncompensated temperature (D2)
  SPIx_transmit(SPI1, Dx);//*Dx for what? setup spi function prepaid for D1 & D2
  GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

uint32_t ms5611_conver_adc_read(void)
{
  uint8_t MSB=0,LSB=0,ret=0;
  GPIO_ResetBits(GPIOA,GPIO_Pin_4);
  SPIx_transmit(SPI1, MS5611_ADC_READ);//*spi first setup
  MSB=SPIx_transmit(SPI1, 0x00);//*why here 3 times 0x00
  ret=SPIx_transmit(SPI1, 0x00);//*0x00 for ADC read and MSB ret LSB so three times
  LSB=SPIx_transmit(SPI1, 0x00);//*prepaid clock (24 slot) After ADC read commands the device will return 24 bit result  
  GPIO_SetBits(GPIOA,GPIO_Pin_4);

  return  (uint32_t)(MSB <<16| ret <<8| LSB); // 24 bit result 
}


void ms5611_read_prom(uint16_t prom)
{

  //C1  Pressure sensitivity | SENS T1
  //C2  Pressure offset | OFF T1
  //C3  Temperature coefficient of pressure sensitivity | TCS
  //C4  Temperature coefficient of pressure offset | TCO
  //C5  Reference temperature | T REF
  //C6  Temperature coefficient of the temperature | TEMPSENS
  //CRC CYCLIC REDUNDANCY CHECK (CRC)

  uint8_t i=0;
  uint8_t MSB[7]={0},LSB[7]={0};

    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    SPIx_transmit(SPI1, MS5611_READ_PROM_C1);//*0xa0 to 0xae for read prom 
    Delay_1us(20);
    MSB[0]=SPIx_transmit(SPI1, 0xff);//0xff   
    Delay_1us(20);
    LSB[0]=SPIx_transmit(SPI1, 0xff); //*after the PROM read 16bit result
    GPIO_SetBits(GPIOA,GPIO_Pin_4);

    C[1] = (float)(MSB[0] <<8| LSB[0]);


  for(i=0;i<6;i++)//C1 to C6
  {
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    SPIx_transmit(SPI1, prom + 2 * i);//*0xa0 to 0xae for read prom 
    Delay_1us(20);
    MSB[i]=SPIx_transmit(SPI1, 0xff);//0xff   
    Delay_1us(20);
    LSB[i]=SPIx_transmit(SPI1, 0xff); //*after the PROM read 16bit result
    GPIO_SetBits(GPIOA,GPIO_Pin_4);

    C[i+1] = (float)(MSB[i] <<8| LSB[i]);//*16bit result (start from C1)
  }

}



// void ms5611_calibrate(uint16_t samplenum)
// {
//   int16_t i;

//   float dT=0;
//     float OFF=0,SENS=0;


//     for(i=1;i<=samplenum;i++)
//   {


//     D1 = (float)ms5611_conver_adc_read();//D1 (Digital pressure value)
//     ms5611_conver_adc_send(MS5611_CONV_D2_OSR4096);//D2
//       Delay_1us(9000);
//       D2 = (float)ms5611_conver_adc_read();// D2 (Digital temperature value)
//       ms5611_conver_adc_send(MS5611_CONV_D1_OSR4096);//D1
//     Delay_1us(9000);        

//       dT =(float)( D2 - C[5] * POW_2_8);
//       OFF = (float)(C[2] * POW_2_16 + dT * C[4] / POW_2_7);
//       SENS = (float)(C[1] * POW_2_15 + dT * C[3] / POW_2_8);
//       Temp = (float)((2000 + (dT * C[6]) / POW_2_23))/ 100.f;
//       Press = (float)((((D1 * SENS) / POW_2_21 - OFF) / POW_2_15))/100.f;
//       Temp_sum += Temp;
//       Press_sum += Press;
//     }
//     Press_offset = Press_sum / samplenum;
//     Temp_offset = Temp_sum / samplenum;  
// }

// void ms5611_calculate(void)
// {
//   float dT;
//     float OFF,SENS;

//   dT =( D2 - C[5] * POW_2_8);
//     OFF = (C[2] * POW_2_16 + dT * C[4] / POW_2_7);
//     SENS = (C[1] * POW_2_15 + dT * C[3] / POW_2_8);
//     Temp = ((2000 + (dT * C[6]) / POW_2_23))/ 100.f;
//     Press = ((((D1 * SENS) / POW_2_21 - OFF) / POW_2_15))/100.f;  
// }

float baro_to_altitude(float P, float T)
{
  float h;  
  h = (T + 273.15) * (pow(P / 1013.25f , 0.1903) - 1.0f) * 153.84;
  return h;
}


void ms5611_init(void)
{
  ms5611_reset();
  Delay_1us(10000);

  ms5611_read_prom(MS5611_READ_PROM_C1);

    ms5611_conver_adc_send(MS5611_CONV_D1_OSR4096); //0x48  
    Delay_1us(9000);
    D1 = ms5611_conver_adc_read();
  
    ms5611_conver_adc_send(MS5611_CONV_D2_OSR4096); //0x58
    Delay_1us(9000);//9000 where is this in ms5611 pdf
    D2 = ms5611_conver_adc_read();

  //   ms5611_calculate(); //because we got information
  //   h_init=baro_to_altitude(Press,Temp);

  //   ms5611_conver_adc_send(MS5611_CONV_D1_OSR4096); //That why we must send D1 0x48 at last step because main firt row get D1 data
  // Delay_1us(9000);    
}


void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX

    /* USARTx configuration ------------------------------------------------------*/
    /* USARTx configured as follow:
     *  - BaudRate = 57600 baud
     *  - Word Length = 8 Bits
     *  - One Stop Bit
     *  - No parity
     *  - Hardware flow control disabled (RTS and CTS signals)
     *  - Receive and transmit enabled
     */

    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* NVIC Initialization */
    NVIC_InitTypeDef NVIC_InitStruct = {
      .NVIC_IRQChannel = USART1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 0,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&NVIC_InitStruct);

}

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}

/**************************************************************************************/
int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    LED_Initialization();
    SPI1_init();
    Timer5_Initialization();
    // ms5611_init();
    ms5611_init();

    USART1_puts("Hello World!\r\n");
    USART1_puts("Just for STM32F429I Discovery verify USART1 with USB TTL Cable\r\n");
    double buff[100];
    int j=0;
    while(1)
    {
      if(freqflag==1)
        {
           freqflag=0;


        uint8_t i=0;
        uint8_t MSB[7]={0},LSB[7]={0};

        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        SPIx_transmit(SPI1, MS5611_READ_PROM_C1);//*0xa0 to 0xae for read prom 
        Delay_1us(20);
        MSB[0]=SPIx_transmit(SPI1, 0xff);//0xff   
        Delay_1us(20);
        LSB[0]=SPIx_transmit(SPI1, 0xff); //*after the PROM read 16bit result
        GPIO_SetBits(GPIOA,GPIO_Pin_4);

        C[1] = (MSB[0] <<8| LSB[0]);

        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        SPIx_transmit(SPI1, MS5611_READ_PROM_C2);//*0xa0 to 0xae for read prom 
        Delay_1us(20);
        MSB[1]=SPIx_transmit(SPI1, 0xff);//0xff   
        Delay_1us(20);
        LSB[1]=SPIx_transmit(SPI1, 0xff); //*after the PROM read 16bit result
        GPIO_SetBits(GPIOA,GPIO_Pin_4);

        C[2] = (MSB[1] <<8| LSB[1]);

        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        SPIx_transmit(SPI1, MS5611_READ_PROM_C3);//*0xa0 to 0xae for read prom 
        Delay_1us(20);
        MSB[2]=SPIx_transmit(SPI1, 0xff);//0xff   
        Delay_1us(20);
        LSB[2]=SPIx_transmit(SPI1, 0xff); //*after the PROM read 16bit result
        GPIO_SetBits(GPIOA,GPIO_Pin_4);

        C[3] = (MSB[2] <<8| LSB[2]);

        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        SPIx_transmit(SPI1, MS5611_READ_PROM_C4);//*0xa0 to 0xae for read prom 
        Delay_1us(20);
        MSB[3]=SPIx_transmit(SPI1, 0xff);//0xff   
        Delay_1us(20);
        LSB[3]=SPIx_transmit(SPI1, 0xff); //*after the PROM read 16bit result
        GPIO_SetBits(GPIOA,GPIO_Pin_4);

        C[4] = (MSB[3] <<8| LSB[3]);

        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        SPIx_transmit(SPI1, MS5611_READ_PROM_C5);//*0xa0 to 0xae for read prom 
        Delay_1us(20);
        MSB[4]=SPIx_transmit(SPI1, 0xff);//0xff   
        Delay_1us(20);
        LSB[4]=SPIx_transmit(SPI1, 0xff); //*after the PROM read 16bit result
        GPIO_SetBits(GPIOA,GPIO_Pin_4);

        C[5] = (MSB[4] <<8| LSB[4]);

        GPIO_ResetBits(GPIOA, GPIO_Pin_4);
        SPIx_transmit(SPI1, MS5611_READ_PROM_C6);//*0xa0 to 0xae for read prom 
        Delay_1us(20);
        MSB[5]=SPIx_transmit(SPI1, 0xff);//0xff   
        Delay_1us(20);
        LSB[5]=SPIx_transmit(SPI1, 0xff); //*after the PROM read 16bit result
        GPIO_SetBits(GPIOA,GPIO_Pin_4);

        C[6] = (MSB[5] <<8| LSB[5]);


        ms5611_conver_adc_send(MS5611_CONV_D1_OSR4096);
        Delay_1us(9000);
        D1 = ms5611_conver_adc_read();

        ms5611_conver_adc_send(MS5611_CONV_D2_OSR4096);
        Delay_1us(9000);
        D2 = ms5611_conver_adc_read();


        // sprintf((char *)buff, "%d,%d,%d,%d,%d,%d \r\n" ,C[1],C[2],C[3],C[4],C[5],C[6]);
        // USART1_puts((char *)buff);
        // for(j=0; j<100; j++)
        // {
        //   buff[j]==0;
        // }
        //48320,49222,29268,26594,32441,27735

        int32_t dT;
        int64_t OFF,SENS;
        int32_t Temp=0;
        float Press=0; 
        float temp_float=0;
        float testting_temp=0.0f;
        

        testting_temp=C[5]*POW_2_8;//C5*256
        dT = D2 - testting_temp;

        testting_temp=dT*C[4];

        testting_temp=testting_temp*DIVIDED_ONE_THOUSAND*DIVIDED_POW_2_7;
        // testting_temp=testting_temp*;

        // testting_temp=dT*DIVIDED_POW_2_23; 
        // testting_temp=testting_temp*DIVIDED_ONE_THOUSAND; //


        // testting_temp=(((C[5]>>4)*(C[6]>>4))>>7);
        // Temp= 2000 + ((((D2>>6)*C[6])>>18)-(((C[5]>>4)*(C[6]>>4))>>7));

        testting_temp=((C[5]*C[6])>>15);
        Temp = 2000+((((uint64_t)D2*(uint64_t)C[6]))>>23)-((C[5]*C[6])>>15);

        

        // Press = (float)(((double)D1*(double)C[1])/2097152);//first term ok
        // Press = ((float)(((double)((D1>>12)*(D2>>12))*(double)C[3])/1048576)); //second term ok
        // Press = (float)(((double)((D1>>12)*(C[5]>>2))*(double)(C[3]>>2))/1048576); //third term ok
        // Press = ((float)(((double)(C[4]*C[5]))/16384)); //four term ok
        // Press = 2*C[2]; //five term ok
        // Press =((float)(((double)((D2>>10)*C[4]))/4096)); //six term ok
        //ALl term

        // Press = ((float)(((double)D1*(double)C[1])/2097152))+((float)(((double)((D1>>12)*(D2>>12))*(double)C[3])/1048576))-((float)(((double)((D1>>12)*(C[5]>>2))*(double)(C[3]>>2))/1048576))+((float)(((double)(C[4]*C[5]))/16384))-(2*C[2])-((float)(((double)((D2>>10)*C[4]))/4096));
        LED3_Toggle();
        temp_float = 2000+(float)((((double)D2*(double)C[6]))/8388608.0)-((C[5]*C[6])>>15);//ok
        Press =(float)(((double)D1*(double)C[1])/2097152)+((float)(((double)((D1>>12)*(D2>>12))*(double)C[3])/1048576));
        Press =Press-((float)(((double)((D1>>12)*(C[5]>>2))*(double)(C[3]>>2))/1048576));
        Press =Press +((float)(((double)(C[4]*C[5]))/16384));
        Press =Press -(2*C[2]) -((float)(((double)((D2>>10)*C[4]))/4096));
        LED3_Toggle();

        OFF = (C[2]*POW_2_16) + testting_temp;

        
        sprintf((char *)buff, "%d,%6.10f,%6.10f \r\n" ,Temp,temp_float,Press);
        USART1_puts((char *)buff);
        for(j=0; j<100; j++)
        {
          buff[j]==0;
        }

        // testting_temp=dT/DIVIDED_POW_2_7;
        // testting_temp=testting_temp*DIVIDED_ONE_THOUSAND;
        // SENS = (C[1]*POW_2_15 + testting_temp*C[3]);

        // testting_temp=dT*DIVIDED_POW_2_23; //0.028223276;//C[6]=49222 divided to pow2^23   0.00586771965
        // testting_temp=testting_temp*DIVIDED_ONE_THOUSAND; //
        // testting_temp=testting_temp*C[6];
        // Temp= 2000 + testting_temp;
        // // Temp=Temp*0.01;

        // testting_temp=D1*DIVIDED_POW_2_21;
        // testting_temp=testting_temp*DIVIDED_ONE_THOUSAND;
        // testting_temp=testting_temp*DIVIDED_ONE_THOUSAND;
        // // Press =  ((testting_temp*SENS) - OFF);  
        // // Press=Press*DIVIDED_POW_2_15;
        // // Press=Press*DIVIDED_ONE_THOUSAND;
        // Press = ((((D1 * SENS) / POW_2_21 - OFF) / POW_2_15));


        sprintf((char *)buff, "%d,%d \r\n" ,D1,D2);
        USART1_puts((char *)buff);
        for(j=0; j<100; j++)
        {
          buff[j]==0;
        }

        // sprintf((char *)buff, "%d,%6.9f,%d \r\n" ,D2,testting_temp,OFF);
        // USART1_puts((char *)buff);
        // for(j=0; j<100; j++)
        // {
        //   buff[j]==0;
        // }

        

        // ms5611_conver_adc_send(MS5611_CONV_D2_OSR4096);




        // Delay_1us(10000);

        
        // USART1_puts("Hello World!\r\n");


    }
  }

}


void USART1_IRQHandler(void)
{
  
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    uart1_data = USART_ReceiveData(USART1);

    USART_SendData(USART1, uart1_data);

  }

}

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        freqflag=1;
    }
}