#pragma once

// #if defined(ARDUINO_ARCH_STM32)   // Arduino_Core_STM32

// #if defined(ARDUINO_GENERIC_F405RGTX) || defined(ARDUINO_GENERIC_F407VETX) || defined(ARDUINO_BLACK_F407VE) || defined(ARDUINO_BLACK_F407VG)
#if defined(ARDUINO_GENERIC_F405RGTX)

// #define LED_ON(__PIN__)   digitalWrite(__PIN__, LOW);
// #define LED_OFF(__PIN__)   digitalWrite(__PIN__, HIGH);

/*
**TIM3** 16 bit, general purpose, 4 ch  
PB_0_ALT1  - Servo out 1 - TIM3_CH3  
PB_1_ALT1  - Servo out 2 - TIM3_CH4  
**TIM2** 32 bit, general purpose, 4 ch  
PA3  - Servo out 3 - TIM2_CH4  
PA2  - Servo out 4 - TIM2_CH3  
**TIM5** 32 bit, general purpose, 4 ch  
PA1_ALT1  - Servo out 5 - TIM5_CH2  
PA0_ALT1  - Servo out 6 - TIM5_CH1  
**TIM12** 16 bit, general purpose, 2 ch  
PB14_ALT1 - S1_IN - TIM12_CH1  
PB15_ALT1 - S2_IN - TIM12_CH2  
**TIM8** 16 bit, advanced, 4 ch  
PC6_ALT1  - S3_IN - TIM8_CH1  
PC7_ALT1  - S4_IN - TIM8_CH2  
PC8_ALT1  - S5_IN - TIM8_CH3  
PC9_ALT1  - S6_IN - TIM8_CH4  
*/

#define OUTPUT_GRP0_TIMx TIM3
const PinName output0_Pin = PB_0_ALT1;   const uint32_t output0_Chan = 3;
const PinName output1_Pin = PB_1_ALT1;   const uint32_t output1_Chan = 4;

#define OUTPUT_GRP1_TIMx TIM2
const PinName output2_Pin = PA_3;   const uint32_t output2_Chan = 4;
const PinName output3_Pin = PA_2;   const uint32_t output3_Chan = 3;

#define OUTPUT_GRP2_TIMx TIM5
const PinName output4_Pin = PA_1_ALT1;   const uint32_t output4_Chan = 2;
const PinName output5_Pin = PA_0_ALT1;   const uint32_t output5_Chan = 1;

#define OUTPUT_GRP3_TIMx TIM12
const PinName output6_Pin = PB_14_ALT2;  const uint32_t output6_Chan = 1;
const PinName output7_Pin = PB_15_ALT2;  const uint32_t output7_Chan = 2;

#define OUTPUT_GRP4_TIMx TIM8
const PinName output8_Pin = PC_6_ALT1;   const uint32_t output8_Chan = 1;
const PinName output9_Pin = PC_7_ALT1;   const uint32_t output9_Chan = 2;
const PinName output10_Pin = PC_8_ALT1;  const uint32_t output10_Chan = 3;
const PinName output11_Pin = PC_9_ALT1;  const uint32_t output11_Chan = 4;

#define SBUS_SERIALx Serial3

#define EXEC_TIMx TIM1

#define MPU_SPIx SPI1
const PinName MPU_PIN_miso = PA_6;    // YEL
const PinName MPU_PIN_mosi = PA_7;    // GRN
const PinName MPU_PIN_sclk = PA_5;    // ORG
const PinName MPU_PIN_ssel = NC;      // SSEL CS contolled by SW
const PinName MPU_slaveSelect = PA_4; // BLU

const uint32_t MPU_SPI_LS_CLOCK =  1000000;
const uint32_t MPU_SPI_HS_CLOCK =  6250000;

#define W25_SPIx SPI3
const PinName W25_PIN_mosi = PC_12;
const PinName W25_PIN_miso = PC_11;
const PinName W25_PIN_sclk = PC_10;
const PinName W25_PIN_ssel = NC;
const PinName W25_slaveSelect = PB_3;

const uint32_t W25_SPI_CLOCK =  1000000;

const int warningLED = PB4;
const int statusLED = PB5;

#elif defined(ARDUINO_NUCLEO_F411RE)

/*
**TIM3** 16 bit, general purpose, 4 ch  
PC6  - Servo out 1 - TIM3_CH1  
PC7  - Servo out 2 - TIM3_CH2  
PC8  - Servo out 3 - TIM3_CH3  
PC9  - Servo out 4 - TIM3_CH4  
 
**TIM5** 32 bit, general purpose, 4 ch  
PA0  - Servo out 5 - TIM5_CH1  
PA1  - Servo out 6 - TIM5_CH2  
*/

#define OUTPUT_GRP0_TIMx TIM3
const PinName output0_Pin = PC_6;   const uint32_t output0_Chan = 1;
const PinName output1_Pin = PC_7;   const uint32_t output1_Chan = 2;
const PinName output2_Pin = PC_8;   const uint32_t output2_Chan = 3;
const PinName output3_Pin = PC_9;   const uint32_t output3_Chan = 4;

#define OUTPUT_GRP1_TIMx TIM5
const PinName output4_Pin = PA_0;   const uint32_t output4_Chan = 1;
const PinName output5_Pin = PA_1;   const uint32_t output5_Chan = 2;

#define SBUS_SERIALx Serial1

#define EXEC_TIMx TIM1

#define MPU_SPIx SPI1
const PinName MPU_PIN_miso = PA_6;    // YEL
const PinName MPU_PIN_mosi = PA_7;    // GRN
const PinName MPU_PIN_sclk = PA_5;    // ORG
const PinName MPU_PIN_ssel = NC;      // SSEL CS contolled by SW
const PinName MPU_slaveSelect = PA_4; // BLU

const uint32_t MPU_SPI_LS_CLOCK =  1000000;
const uint32_t MPU_SPI_HS_CLOCK =  6250000;

#define W25_SPIx SPI2
const PinName W25_PIN_mosi = PB_15;
const PinName W25_PIN_miso = PB_14;
const PinName W25_PIN_sclk = PB_13;
const PinName W25_PIN_ssel = NC;
const PinName W25_slaveSelect = PB_12;

const uint32_t W25_SPI_CLOCK =  1000000;

const int warningLED = PB4;
const int statusLED = PB5;

#else

    #error unsupported STM32 variant

#endif

// #else
//   #error unsupported architecture
// #endif


/* Revo pins

BOOT0
NRST
OSC_IN/PD0
OSC_OUT/PD1
PA0 - TIM5_CH1
PA1- TIM5_CH2
PA2 - TIM2 CH3
PA3 - TIM9 CH2
PA4 - SPI1 NSS
PA5 - SPI1 SCK
PA6 - SPI1 MISO
PA7 - SPI1 MOSI
PA8 - TIM1 CH1
PA9 - USART1 TX
PA10 - USART1 RX
PA11 - USB DM
PA12 - USB DP
PA13 - JTMS
PA14 - JTCK
PA15 - SPI3 NSS
PB0 - TIM3 CH3
PB1 - TIM3 CH4
PB2 - BOOT1
PB3 - Flash CS
PB4
PB5
PB6
PB7 - Mag DRDY
PB8 - I2C1 SCL
PB9 - I2C1 SDA
PB10 - I2C2 SCL
PB11 - I2C2 SDA
PB12 
PB13
PB14 - TIM12 CH1
PB15 - TIM12 CH2
PC0 - S.Bus Invert
PC1 - ADC123_11 Curr
PC2 - ADC123_12 Volt
PC3 - Frequency
PC4 - Gyro INT
PC5 - VBUS SEN
PC6 - TIM8 CH1
PC7 - TIM8 CH2
PC8 - TIM8 CH3
PC9 - TIM8 CH4
PC10 - SPI3 SCK
PC11 - SPI3 MISO
PC12 - SPI3 MOSI
PC13 - RTC
PC14 - OSC32_IN
PC15 - OSC32_OUT
PD2 - Modem IRQ
VBAT
VDD1
VDD2
VDD3
VDD4
VDDA
VCP1
VCP2
VSS3
VSS4
VSSA
*/