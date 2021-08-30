// ************************************************************
// * Code
// ************************************************************

#include "src/fs/W25Q128/W25Q128.h"
#include "src/fs/fs.h"
#include "src/fs/littleFS/lfs.h"
#include "src/MPU6000/MPU6000.h"
#include "src/printf/printf.h"
#include "port.h"
#include "errors.h"


uint8_t initialize(void)
{
  uint8_t i;
  uint8_t ret;

  //***********************************************************
  // Start up - load EEPROM
  //***********************************************************

  // Initialize SPI flash file system
  if (W25_Init() != ERR_OK) {
    return ERR_NO_SPI_FLASH;
  }

  // Attempt SPI flash file system mount, drive might not be formatted so
  // attempt formatting on 1st fail. Error out if 2nd attempt fails.
  if (FS_Mount() != ERR_OK) {
    printf_("Could not mount media, attemping to format\r\n");
    if (FS_Format() != ERR_OK) {
      printf_("ERROR: Format failed\r\n");
      return ERR_SPI_FLASH_FS_MOUNT;
    }
    printf_("Attempting to mount freshly formatted media\r\n");
    if (FS_Mount() != ERR_OK) {
      printf_("ERROR: Mount after format failed\r\n");
      return ERR_SPI_FLASH_FS_MOUNT;
    }
  }
  FS_PrintStatus();
  // FS_Unmount();
  // FS_Format();
  // FS_Mount();

  // FS_SaveConfig();
  // FS_PrintHexFile("./config.txt");
  // FS_Dir("./");

  // Load EEPROM settings
  updated = initialEEPROMConfigLoad(); // Config now contains valid values

  FS_PrintHexFile("./config.txt");
  FS_Dir("./");

  //***********************************************************
  // I/O setup - 4 buttons, 2 LEDs, setupServos()
  //***********************************************************

  // Heartbeat LED
  pinMode(HEARTBEAT_LED, OUTPUT);

  // Status LED
  pinMode(STATUS_LED, OUTPUT);

  //***********************************************************
  // RX Configuration - sbus/dsmx/cppm begin
  //***********************************************************

  //***********************************************************
  // LCD initialization - u82g
  //***********************************************************

  //***********************************************************
  // ESC calibration - Cal ONLY if buttons 1 and 4 pressed
  //***********************************************************

  //***********************************************************
  // Reset EEPROM default settings ONLY if middle two buttons pressed
  //***********************************************************
    
  //***********************************************************
  // i2c/MPU6050 init
  //*********************************************************** 

  // set slaveSelect pin output high (set BSRR bit) before pinMode to avoid transient low
  LL_GPIO_SetOutputPin(set_GPIO_Port_Clock(STM_PORT(MPU_slaveSelect)), STM_LL_GPIO_PIN(MPU_slaveSelect));
  pinMode(pinNametoDigitalPin(MPU_slaveSelect), OUTPUT);

  /* Config MPU SPI port per settings from port.h */
  MPU6000_spi.handle.Instance = MPU_SPIx;
  MPU6000_spi.pin_miso = MPU_PIN_miso;
  MPU6000_spi.pin_mosi = MPU_PIN_mosi;
  MPU6000_spi.pin_sclk = MPU_PIN_sclk;
  MPU6000_spi.pin_ssel = MPU_PIN_ssel;

  mpu.initialize(MPU_slaveSelect, SPI_LS_CLOCK, SPI_HS_CLOCK, SPI_STM32_MODE_3, SPI_STM32_MSBFIRST);
  mpu.setSpeedSPI(LOW);

  if (mpu.testConnection() == false) {
    Serial.println("MPU6000 initialization unsuccessful");
    Serial.println("Check MPU6000 wiring or try cycling power");
    return ERR_NO_IMU;
  }

  mpu.setClockSource(MPU6000_CLOCK_PLL_XGYRO);
  mpu.setInterruptDrive(MPU6000_INTCFG_INT_OPEN_BIT);
  mpu.setDLPFMode(6 - config.mpu60x0LPF);
  mpu.setRate(1);  // 500 Hz
  mpu.setFullScaleGyroRange(MPU6000_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6000_ACCEL_FS_4);


  mpu.setSpeedSPI(HIGH);

  //***********************************************************
  // Remaining init tasks
  //***********************************************************

  // Display "Hold steady" message on LCD
  
  // Set ADC resolution and averaging

  // Initial gyro calibration - calibrateGyrosSlow(), reboot on fail
  
  // Check power-up battery voltage
  
  // updateLimits() from mixer.ino - Updates travel and trigger limits

  // Disarm on start-up if Armed setting is ARMABLE

  // Reset IMU with resetIMU() - why?

  // Beep that init is complete

#ifdef ERROR_LOG
// If restart, log it as such
add_log(REBOOT);
#endif

  return ERR_OK;

}
