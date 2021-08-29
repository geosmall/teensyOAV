#include "iocfg.h"
#include "typedefs.h"
#include "globals.h"

#include "src/MPU6000/MPU6000.h"
#include "src/fs/W25Q128/W25Q128.h"
#include "src/fs/fs.h"
#include "src/shell/shell.h"

spi_stm32_t MPU6000_spi;
const uint32_t SPI_LS_CLOCK =  1000000;
const uint32_t SPI_HS_CLOCK =  6250000;
MPU6000 mpu(MPU6000_spi);

//SbusRx sbusRx(&Serial1);

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
//IntervalTimer execTimer;

#define FRAME_COUNT 1000
#define COUNT_500HZ 2
#define COUNT_250HZ 4
#define COUNT_100HZ 10
#define COUNT_50HZ  20
#define COUNT_20HZ  50
#define COUNT_10HZ  100
#define COUNT_5HZ   200
#define COUNT_1HZ   1000

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
uint16_t rcTimeout         = 0;
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

sShellImpl shell_impl = { .send_char = _putchar };


void setup() {

  Serial.begin(115200);
  Serial.println("Initializing...");
  
  initialize();

  shell_boot(&shell_impl);

}


char c;
void loop() {
  c = console_getc();
  shell_receive_char(c);
}
