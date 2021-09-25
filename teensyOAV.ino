#include "iocfg.h"
#include "typedefs.h"
#include "globals.h"
#include "port.h"
#include "errors.h"

#include "src/sbus/sbus.h"
#include "src/MPU6000/MPU6000.h"
#include "src/fs/W25Q128/W25Q128.h"
#include "src/fs/fs.h"
#include "src/shell/shell.h"
#include "src/myPWMServo/myPWMServo.h"

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Main sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

spi_stm32_t MPU6000_spi;
MPU6000 mpu(MPU6000_spi);

SbusRx sbusRx(&SBUS_SERIALx);
SbusTx sbusTx(&Serial1);

#if defined(ARDUINO_GENERIC_F405RGTX)
// Output PWM objects (see port.h for board level definitions)
myPWMServo output0(output0_Pin, OUTPUT_GRP0_TIMx, output0_Chan);
myPWMServo output1(output1_Pin, OUTPUT_GRP0_TIMx, output1_Chan);
myPWMServo output2(output2_Pin, OUTPUT_GRP1_TIMx, output2_Chan);
myPWMServo output3(output3_Pin, OUTPUT_GRP1_TIMx, output3_Chan);
myPWMServo output4(output4_Pin, OUTPUT_GRP2_TIMx, output4_Chan);
myPWMServo output5(output5_Pin, OUTPUT_GRP2_TIMx, output5_Chan);
myPWMServo output6(output6_Pin, OUTPUT_GRP3_TIMx, output6_Chan);
myPWMServo output7(output7_Pin, OUTPUT_GRP3_TIMx, output7_Chan);
myPWMServo output8(output8_Pin, OUTPUT_GRP4_TIMx, output8_Chan);
myPWMServo output9(output9_Pin, OUTPUT_GRP4_TIMx, output9_Chan);
#elif defined(ARDUINO_NUCLEO_F411RE)
// Output PWM objects (see port.h for board level definitions)
myPWMServo output0(output0_Pin, OUTPUT_GRP0_TIMx, output0_Chan);
myPWMServo output1(output1_Pin, OUTPUT_GRP0_TIMx, output1_Chan);
myPWMServo output2(output2_Pin, OUTPUT_GRP0_TIMx, output2_Chan);
myPWMServo output3(output3_Pin, OUTPUT_GRP0_TIMx, output3_Chan);
myPWMServo output4(output4_Pin, OUTPUT_GRP1_TIMx, output4_Chan);
myPWMServo output5(output5_Pin, OUTPUT_GRP1_TIMx, output5_Chan);
#endif

// HEARTBEAT_LED
#define HEARTBEAT_LED 3

// LED Heartbeat Variables
uint8_t ledState = LOW;

// Executive Timing Variables
TIM_TypeDef *execTimerInstance = EXEC_TIMx;
HardwareTimer *execTimer = new HardwareTimer(execTimerInstance);

#define FRAME_COUNT 1000
#define COUNT_500HZ 2
#define COUNT_250HZ 4
#define COUNT_100HZ 10
#define COUNT_50HZ  20
#define COUNT_20HZ  50
#define COUNT_10HZ  100
#define COUNT_5HZ   200
#define COUNT_1HZ   1000

#define RC_OVERDUE  750  // Number of 1 mSec ticks before RC will be overdue = 750 mSec

uint16_t frameCounter = 0;
uint8_t frame_500Hz   = false;
uint8_t frame_250Hz   = false;
uint8_t frame_100Hz   = false;
uint8_t frame_50Hz    = false;
uint8_t frame_20Hz    = false;
uint8_t frame_10Hz    = false;
uint8_t frame_5Hz     = false;
uint8_t frame_1Hz     = false;

uint16_t armTimer          = 0;
uint16_t disarmTimer       = 0;
uint16_t gyroTimeout       = 0;
uint16_t rcTimeout         = RC_OVERDUE;
uint16_t statusSeconds     = 0;
uint16_t transitionTimeout = 0;
uint16_t updateStatusTimer = 0;

// Button Pins
#define BUTTON1 20
#define BUTTON2 21
#define BUTTON3 22
#define BUTTON4 23

uint8_t pinb = 0xFF;

//***********************************************************
//* Code and Data variables
//***********************************************************

// Flight variables
int16_t transition = 0;
int16_t transitionCounter = 0;

// Flags
volatile uint8_t flightFlags  = 0;
volatile uint8_t generalError = 0;
volatile uint8_t alarmFlags   = 0;

// Global Buffers
#define PBUFFER_SIZE 25
char pBuffer[PBUFFER_SIZE];      // Print buffer (25 bytes)

// used by printf_ and shell
extern "C" int _putchar(char c) {
  Serial.write(c);
  return 1;
}

char console_getc(void) {
  uint8_t cr;
  while (Serial.available() != true);
  cr = Serial.read();
  return (char)cr;
}

//************************************************************
//* Setup
//************************************************************

sShellImpl shell_impl = { .send_char = _putchar };

void setup() {

  Serial.begin(115200);
  Serial.print("Initializing OAV, SystemCoreClock(Hz) = ");
  Serial.println(SystemCoreClock);

  /* Initialize board */
  int res = initialize();
  if (res != ERR_OK) {
    while (1);
  }

  /* Start command line shell */
  shell_init(&shell_impl);

  /* Set TIM1 timer interval, attach Exec function to TIM interrupt and start timer */
  execTimer->setOverflow(1000, HERTZ_FORMAT);  // run every 1 mSec
  execTimer->attachInterrupt(execCount);
  execTimer->resume();

}

//************************************************************
//* Exec function
//************************************************************

void execCount(void) {
  frameCounter++;
  if (frameCounter > FRAME_COUNT)
    frameCounter = 1;

  if ((frameCounter % COUNT_500HZ) == 0)
    frame_500Hz = true;

  if ((frameCounter % COUNT_250HZ) == 0)
    frame_250Hz = true;

  if ((frameCounter % COUNT_100HZ) == 0)
  {
    frame_100Hz = true;
    rxGetChannels();
  }

  if ((frameCounter % COUNT_50HZ) == 0)
    frame_50Hz = true;

  if ((frameCounter % COUNT_20HZ) == 0)
    frame_20Hz = true;

  if ((frameCounter % COUNT_10HZ) == 0)
    frame_10Hz = true;

  if ((frameCounter % COUNT_5HZ) == 0)
    frame_5Hz = true;

  if ((frameCounter % COUNT_1HZ) == 0)
    frame_1Hz = true;

  armTimer++;
  disarmTimer++;
  gyroTimeout++;
  rcTimeout++;
  statusSeconds++;
  transitionTimeout++;
  updateStatusTimer++;

  pinb = digitalRead(BUTTON1) << 7 |
         digitalRead(BUTTON2) << 6 |
         digitalRead(BUTTON3) << 5 |
         digitalRead(BUTTON4) << 4;
}

//************************************************************
//* Main loop
//************************************************************

char c;
void loop() {
  c = console_getc();
  shell_receive_char(c);
}

//************************************************************
//* STM32 Error Handler
//************************************************************

void _Error_Handler(const char *msg, int val)
{
  /* User can add his own implementation to report the HAL error return state */
  // core_debug("Error: %s (%i)\n", msg, val);
  while (1) {
  }
}

//************************************************************
//* STM32F405RG 8MHz Xtal clock config
//************************************************************

#if defined(ARDUINO_GENERIC_F405RGTX)

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }

  /* Ensure CCM RAM clock is enabled */
  __HAL_RCC_CCMDATARAMEN_CLK_ENABLE();

}

#endif
