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

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Main sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

spi_stm32_t MPU6000_spi;
MPU6000 mpu(MPU6000_spi);

SbusRx sbusRx(&SBUS_SERIALx);

//myPWMServo output0;
//myPWMServo output1;
//myPWMServo output2;
//myPWMServo output3;
//myPWMServo output4;
//myPWMServo output5;
//myPWMServo output6;
//myPWMServo output7;

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
  Serial.println("Initializing...");
  
  /* Initialize board */
  int res = initialize();
  if (res != ERR_OK) {
    while(1);
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
