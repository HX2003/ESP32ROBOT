/**************************************************************************/
/*!
    @file     Adafruit_ADS1015.c
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)
    Driver for the ADS1015/ADS1115 ADC
    This is a library for the Adafruit MPL115A2 breakout
    ----> https://www.adafruit.com/products/???
    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
    @section  HISTORY
    v1.0 - First release
*/
/**************************************************************************/
#include "ads1115.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <driver/i2c.h>
#include <esp_log.h>
void Adafruit_ADS1115(uint8_t i2cAddress){
	   m_i2cAddress = i2cAddress;
	   m_conversionDelay = ADS1115_CONVERSIONDELAY;
	   m_bitShift = 0;
	   m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
	}
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1
uint16_t readRegister(uint8_t i2cAddress, uint8_t reg){
	//Wire.beginTransmission(i2cAddress);esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2cAddress << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	// i2cwrite(ADS1015_REG_POINTER_CONVERT);
	i2c_master_write_byte(cmd, ADS1015_REG_POINTER_CONVERT, ACK_CHECK_EN);
	//Wire.endTransmission();
    i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 20/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	//Wire.requestFrom(i2cAddress, (uint8_t)2);
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2cAddress << 1) | I2C_MASTER_READ,ACK_CHECK_EN);
	uint8_t tmpByte1;
	uint8_t tmpByte2;
	i2c_master_read_byte(cmd, &tmpByte1, ACK_VAL);
	i2c_master_read_byte(cmd, &tmpByte2, ACK_VAL);
	  i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 20/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
    return ((tmpByte1 << 8) | tmpByte2);
}
void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value){
	 // Wire.beginTransmission(i2cAddress);
	 // i2cwrite((uint8_t)reg);
	 // i2cwrite((uint8_t)(value>>8));
	 // i2cwrite((uint8_t)(value & 0xFF));
	 // Wire.endTransmission();
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2cAddress << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (uint8_t)(value>>8), ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (uint8_t)(value & 0xFF), ACK_CHECK_EN);
    i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 20/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}
uint16_t readADC_SingleEnded(uint8_t channel) {
  if (channel > 3)
  {
    return 0;
  }

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
					ADS1015_REG_CONFIG_DR_2400SPS   | // 490samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  ets_delay_us(m_conversionDelay*1000);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  return readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
}
double conversionToVoltage(uint16_t val, double mul){
	double multiplyer=0.1875;
	return val*multiplyer*mul;
}
