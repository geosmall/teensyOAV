# teensyOAV for STM32

## Libraries

shell  
https://interrupt.memfault.com/blog/firmware-shell#formatted-output  
https://github.com/memfault/interrupt/tree/master/example/firmware-shell  

prinf  
https://github.com/mpaland/printf  

littleFS on SPI flash   
https://mcuoneclipse.com/2019/01/06/driver-and-shell-for-winbond-w25q128-16mbyte-serial-flash-device/  


## Outputs

PB0 - Servo out 1  
&nbsp;&nbsp;&nbsp;&nbsp;{PB_0,       TIM1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 2, 1)}, // TIM1_CH2N  
&nbsp;&nbsp;&nbsp;&nbsp;{PB_0_ALT1,  TIM3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 3, 0)}, // TIM3_CH3  
&nbsp;&nbsp;&nbsp;&nbsp;{PB_0_ALT2,  TIM8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 2, 1)}, // TIM8_CH2N  
PB1 - Servo out 2  
&nbsp;&nbsp;&nbsp;&nbsp;{PB_1,       TIM1,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM1, 3, 1)}, // TIM1_CH3N  
&nbsp;&nbsp;&nbsp;&nbsp;{PB_1_ALT1,  TIM3,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM3, 4, 0)}, // TIM3_CH4  
&nbsp;&nbsp;&nbsp;&nbsp;{PB_1_ALT2,  TIM8,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM8, 3, 1)}, // TIM8_CH3N  
PA3 - Servo out 3  
&nbsp;&nbsp;&nbsp;&nbsp;{PA_3,       TIM2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 4, 0)}, // TIM2_CH4  
&nbsp;&nbsp;&nbsp;&nbsp;{PA_3_ALT1,  TIM5,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM5, 4, 0)}, // TIM5_CH4  
&nbsp;&nbsp;&nbsp;&nbsp;{PA_3_ALT2,  TIM9,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM9, 2, 0)}, // TIM9_CH2  
PA2 - Servo out 4  
&nbsp;&nbsp;&nbsp;&nbsp;{PA_2,       TIM2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 3, 0)}, // TIM2_CH3  
&nbsp;&nbsp;&nbsp;&nbsp;{PA_2_ALT1,  TIM5,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM5, 3, 0)}, // TIM5_CH3  
&nbsp;&nbsp;&nbsp;&nbsp;{PA_2_ALT2,  TIM9,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF3_TIM9, 1, 0)}, // TIM9_CH1  
PA1 - Servo out 5
&nbsp;&nbsp;&nbsp;&nbsp;{PA_1,       TIM2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 2, 0)}, // TIM2_CH2  
&nbsp;&nbsp;&nbsp;&nbsp;{PA_1_ALT1,  TIM5,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM5, 2, 0)}, // TIM5_CH2  
PA0 - Servo out 6
&nbsp;&nbsp;&nbsp;&nbsp;{PA_0,       TIM2,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_TIM2, 1, 0)}, // TIM2_CH1  
&nbsp;&nbsp;&nbsp;&nbsp;{PA_0_ALT1,  TIM5,  STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF2_TIM5, 1, 0)}, // TIM5_CH1  

