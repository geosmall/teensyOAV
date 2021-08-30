#pragma once

// #if defined(ARDUINO_ARCH_STM32)   // Arduino_Core_STM32

// #if defined(ARDUINO_GENERIC_F405RGTX) || defined(ARDUINO_GENERIC_F407VETX) || defined(ARDUINO_BLACK_F407VE) || defined(ARDUINO_BLACK_F407VG)

// #define LED_ON(__PIN__)   digitalWrite(__PIN__, LOW);
// #define LED_OFF(__PIN__)   digitalWrite(__PIN__, HIGH);


#define EXEC_TIMx TIM1

#define MPU_SPIx SPI1
const PinName MPU_PIN_miso = PA_6;    // YEL
const PinName MPU_PIN_mosi = PA_7;    // GRN
const PinName MPU_PIN_sclk = PA_5;    // ORG
const PinName MPU_PIN_ssel = NC;      // SSEL CS contolled by SW
const PinName MPU_slaveSelect = PA_4; // BLU

#define W25_SPIx SPI2
const PinName W25_PIN_mosi = PB_15;
const PinName W25_PIN_miso = PB_14;
const PinName W25_PIN_sclk = PB_13;
const PinName W25_PIN_ssel = NC;

const int warningLED = PB4;
const int statusLED = PB5;

// #else
//     #error unsupported STM32 variant
// #endif

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