/*
 *  Sub1.ino - MP Example to communicate message data
 *  Copyright 2019 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

#include "MP.h"

/* BMI270 */
#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

#include "bmi270.h"

#define BMI270_CS SS
#define BMI270_INT1 22

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH (9.80665f)
#include <MP.h>

/* Callback function prototypes for the BMI270 Sensor API */
int8_t bmi2_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi2_spi_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t len);
int8_t bmi2_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi2_i2c_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t len);
void bmi2_delay_us(uint32_t period);

/* Other functions */
void panic_led_trap(void);
int8_t configure_sensor(struct bmi2_dev *dev);
void bmi2_intr1_callback(void);
void print_rslt(int8_t rslt);

/* Static variables */
static struct bmi2_dev bmi2;
static volatile bool bmi2_intr_recvd = false;
static volatile uint32_t last_time_us = 0;
/* BMI270 end */




/* BMI270 */
/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width) {
  float half_scale = ((float)(1 << bit_width) / 2.0f);

  return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width) {
  float half_scale = ((float)(1 << bit_width) / 2.0f);

  return (dps / ((half_scale))) * (val);
}
/* BMI270 end */

void setup() {
  int ret = 0;

  ret = MP.begin();
  if (ret < 0) {
    errorLoop(2);
  }
  /* BMI270 */
  int8_t rslt;
  Wire.begin();
  bmi2.dev_id = BMI2_I2C_SEC_ADDR;
  //  bmi2.dev_id = BMI2_I2C_PRIM_ADDR;  // JP1 shot
  bmi2.read = bmi2_i2c_read;
  bmi2.write = bmi2_i2c_write;
  bmi2.delay_us = bmi2_delay_us;
  bmi2.intf = BMI2_I2C_INTF;
  bmi2.read_write_len = 30;     // Limitation of the Wire library
  bmi2.config_file_ptr = NULL;  // Use the default BMI270 config file
  /* End of I2C configuration */

  rslt = bmi270_init(&bmi2);
  print_rslt(rslt);

  attachInterrupt(BMI270_INT1, bmi2_intr1_callback, RISING);

  rslt = configure_sensor(&bmi2);
  print_rslt(rslt);
  /* BMI270 end */
}

void loop() {
  int ret;
  int8_t msgid = 0;
  uint32_t msgdata;
  uint32_t resultant_acceleration = 0;

  /* BMI270 */
  if (bmi2_intr_recvd) {
    // bmi2_intr_recvd = false;
    //digitalWrite(LED_BUILTIN, LOW); // Flash the LED to show activity

    struct bmi2_sens_data sensor_data;
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi2);
    print_rslt(rslt);

    Serial.print(last_time_us);  // Comment out this line if using the Serial plotter
    Serial.print(",");           // Comment out this line if using the Serial plotter
    int8_t acc_x = lsb_to_mps2(sensor_data.acc.x, 4, bmi2.resolution);
    Serial.print(acc_x);
    Serial.print(",");
    int8_t acc_y = lsb_to_mps2(sensor_data.acc.y, 4, bmi2.resolution);
    Serial.print(acc_y);
    Serial.print(",");
    int8_t acc_z = lsb_to_mps2(sensor_data.acc.z, 4, bmi2.resolution);
    Serial.print(acc_z);
    Serial.print(",");

    /*    Serial.print(lsb_to_mps2(sensor_data.acc.x, 4, bmi2.resolution));
    Serial.print(",");
    Serial.print(lsb_to_mps2(sensor_data.acc.y, 4, bmi2.resolution));
    Serial.print(",");
    Serial.print(lsb_to_mps2(sensor_data.acc.z, 4, bmi2.resolution));
    Serial.print(",");
*/
    //    digitalWrite(LED_BUILTIN, HIGH);

    resultant_acceleration = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    Serial.println(resultant_acceleration);
    if (resultant_acceleration > 25) {
      Serial.println("OK!!");
      //        digitalWrite(LED_BUILTIN1, HIGH);
      //delay(800);
      //        digitalWrite(LED_BUILTIN1, LOW);
    }
    /*    Serial.print(lsb_to_dps(sensor_data.gyr.x, 2000, bmi2.resolution));
    Serial.print(",");
    Serial.print(lsb_to_dps(sensor_data.gyr.y, 2000, bmi2.resolution));
    Serial.print(",");
    Serial.print(lsb_to_dps(sensor_data.gyr.z, 2000, bmi2.resolution));
    Serial.println();
    */
  }
  /* BMI270 end */

  /* Echo back */

  ret = MP.Recv(&msgid, &msgdata);
  if (ret < 0) {
    errorLoop(3);
  }

  //  ret = MP.Send(msgid, msgdata);
  //  MPLog("Test:0x%08lx\n", resultant_acceleration);
  ret = MP.Send(msgid, resultant_acceleration);
  if (ret < 0) {
    errorLoop(4);
  }
}

void errorLoop(int num) {
  int i;

  while (1) {
    for (i = 0; i < num; i++) {
      ledOn(LED0);
      delay(300);
      ledOff(LED0);
      delay(300);
    }
    delay(1000);
  }
}

/* BMI270 */
int8_t configure_sensor(struct bmi2_dev *dev) {
  int8_t rslt;
  uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

  struct bmi2_int_pin_config int_pin_cfg;
  int_pin_cfg.pin_type = BMI2_INT1;
  int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
  int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
  int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
  int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
  int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

  struct bmi2_sens_config sens_cfg[2];
  sens_cfg[0].type = BMI2_ACCEL;
  sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
  sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE,
  sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

  sens_cfg[1].type = BMI2_GYRO;
  sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
  sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
  sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

  rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  rslt = bmi2_sensor_enable(sens_list, 2, dev);
  if (rslt != BMI2_OK)
    return rslt;

  return rslt;
}

int8_t bmi2_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
  if ((data == NULL) || (len == 0))
    return -1;
  digitalWrite(dev_id, LOW);
  SPI.transfer(reg_addr);
  for (uint16_t i = 0; i < len; i++) {
    data[i] = SPI.transfer(0xff);
  }
  digitalWrite(dev_id, HIGH);
  return 0;
}

int8_t bmi2_spi_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t len) {
  if ((data == NULL) || (len == 0))
    return -1;
  digitalWrite(dev_id, LOW);
  SPI.transfer(reg_addr);
  for (uint16_t i = 0; i < len; i++)
    SPI.transfer(data[i]);
  digitalWrite(dev_id, HIGH);
  return 0;
}

int8_t bmi2_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
  if ((data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }
  uint8_t bytes_received;

  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  if (Wire.endTransmission() == 0) {
    bytes_received = Wire.requestFrom(dev_id, len);
    // Optionally, throw an error if bytes_received != len
    for (uint16_t i = 0; i < bytes_received; i++) {
      data[i] = Wire.read();
    }
  } else {
    return -1;
  }

  return 0;
}

int8_t bmi2_i2c_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t len) {

  if ((data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }

  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  for (uint16_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  if (Wire.endTransmission() != 0) {
    return -1;
  }

  return 0;
}

void bmi2_delay_us(uint32_t period) {
  delayMicroseconds(period);
}

void bmi2_intr1_callback(void) {
  bmi2_intr_recvd = true;
  last_time_us = micros();
}

void panic_led_trap(void) {
  while (1) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
}

void print_rslt(int8_t rslt) {
  switch (rslt) {
    case BMI2_OK:
      return; /* Do nothing */
      break;
    case BMI2_E_NULL_PTR:
      Serial.println("Error [" + String(rslt) + "] : Null pointer");
      panic_led_trap();
      break;
    case BMI2_E_COM_FAIL:
      Serial.println("Error [" + String(rslt) + "] : Communication failure");
      panic_led_trap();
      break;
    case BMI2_E_DEV_NOT_FOUND:
      Serial.println("Error [" + String(rslt) + "] : Device not found");
      panic_led_trap();
      break;
    case BMI2_E_OUT_OF_RANGE:
      Serial.println("Error [" + String(rslt) + "] : Out of range");
      panic_led_trap();
      break;
    case BMI2_E_ACC_INVALID_CFG:
      Serial.println("Error [" + String(rslt) + "] : Invalid accel configuration");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_INVALID_CFG:
      Serial.println("Error [" + String(rslt) + "] : Invalid gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_ACC_GYR_INVALID_CFG:
      Serial.println("Error [" + String(rslt) + "] : Invalid accel/gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_SENSOR:
      Serial.println("Error [" + String(rslt) + "] : Invalid sensor");
      panic_led_trap();
      break;
    case BMI2_E_CONFIG_LOAD:
      Serial.println("Error [" + String(rslt) + "] : Configuration loading error");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_PAGE:
      Serial.println("Error [" + String(rslt) + "] : Invalid page ");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FEAT_BIT:
      Serial.println("Error [" + String(rslt) + "] : Invalid feature bit");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INT_PIN:
      Serial.println("Error [" + String(rslt) + "] : Invalid interrupt pin");
      panic_led_trap();
      break;
    case BMI2_E_SET_APS_FAIL:
      Serial.println("Error [" + String(rslt) + "] : Setting advanced power mode failed");
      panic_led_trap();
      break;
    case BMI2_E_AUX_INVALID_CFG:
      Serial.println("Error [" + String(rslt) + "] : Invalid auxilliary configuration");
      panic_led_trap();
      break;
    case BMI2_E_AUX_BUSY:
      Serial.println("Error [" + String(rslt) + "] : Auxilliary busy");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_FAIL:
      Serial.println("Error [" + String(rslt) + "] : Self test failed");
      panic_led_trap();
      break;
    case BMI2_E_REMAP_ERROR:
      Serial.println("Error [" + String(rslt) + "] : Remapping error");
      panic_led_trap();
      break;
    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
      Serial.println("Error [" + String(rslt) + "] : Gyro user gain update failed");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_NOT_DONE:
      Serial.println("Error [" + String(rslt) + "] : Self test not done");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INPUT:
      Serial.println("Error [" + String(rslt) + "] : Invalid input");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_STATUS:
      Serial.println("Error [" + String(rslt) + "] : Invalid status");
      panic_led_trap();
      break;
    case BMI2_E_CRT_ERROR:
      Serial.println("Error [" + String(rslt) + "] : CRT error");
      panic_led_trap();
      break;
    case BMI2_E_ST_ALREADY_RUNNING:
      Serial.println("Error [" + String(rslt) + "] : Self test already running");
      panic_led_trap();
      break;
    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
      Serial.println("Error [" + String(rslt) + "] : CRT ready for DL fail abort");
      panic_led_trap();
      break;
    case BMI2_E_DL_ERROR:
      Serial.println("Error [" + String(rslt) + "] : DL error");
      panic_led_trap();
      break;
    case BMI2_E_PRECON_ERROR:
      Serial.println("Error [" + String(rslt) + "] : PRECON error");
      panic_led_trap();
      break;
    case BMI2_E_ABORT_ERROR:
      Serial.println("Error [" + String(rslt) + "] : Abort error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_ERROR:
      Serial.println("Error [" + String(rslt) + "] : Gyro self test error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
      Serial.println("Error [" + String(rslt) + "] : Gyro self test timeout");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_ONGOING:
      Serial.println("Error [" + String(rslt) + "] : Write cycle ongoing");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_TIMEOUT:
      Serial.println("Error [" + String(rslt) + "] : Write cycle timeout");
      panic_led_trap();
      break;
    case BMI2_E_ST_NOT_RUNING:
      Serial.println("Error [" + String(rslt) + "] : Self test not running");
      panic_led_trap();
      break;
    case BMI2_E_DATA_RDY_INT_FAILED:
      Serial.println("Error [" + String(rslt) + "] : Data ready interrupt failed");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FOC_POSITION:
      Serial.println("Error [" + String(rslt) + "] : Invalid FOC position");
      panic_led_trap();
      break;
    default:
      Serial.println("Error [" + String(rslt) + "] : Unknown error code");
      panic_led_trap();
      break;
  }
}
/* BMI270 end */