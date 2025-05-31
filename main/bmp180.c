/**
 * @file bmp180.c
 *
 * ESP-IDF driver for BMP180 digital pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2015 Frank Bargstedt
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2024 Zorxx Software
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <malloc.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "bmp180.h"

#define TAG "BMP180"

#define I2C_TRANSFER_TIMEOUT  50 /* (milliseconds) give up on i2c transaction after this timeout */
#define I2C_SPEED             400000 /* hz */
#define BMP180_DELAY_BUFFER   500 /* microseconds */

/* Register Definitions */
#define BMP180_VERSION_REG        0xD0
#define BMP180_CONTROL_REG        0xF4
#define BMP180_RESET_REG          0xE0
#define BMP180_OUT_MSB_REG        0xF6
#define BMP180_OUT_LSB_REG        0xF7
#define BMP180_OUT_XLSB_REG       0xF8
#define BMP180_CALIBRATION_REG    0xAA

/* Values for BMP180_CONTROL_REG */
#define BMP180_MEASURE_TEMP       0x2E
#define BMP180_MEASURE_PRESS      0x34

/* CHIP ID stored in BMP180_VERSION_REG */
#define BMP180_CHIP_ID            0x55

/* Reset value for BMP180_RESET_REG */
#define BMP180_RESET_VALUE        0xB6

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

#pragma pack(push, 2) /* 16-bit packing */
typedef struct s_bmp180_calibration_data
{
   union
   {
      uint16_t raw[11];
      struct
      {
         int16_t AC1;
         int16_t AC2;
         int16_t AC3;
         uint16_t AC4;
         uint16_t AC5;
         uint16_t AC6;
         int16_t B1;
         int16_t B2;
         int16_t MB;
         int16_t MC;
         int16_t MD;
      };
   };
} t_bmp180_calibration_data;
#pragma pack(pop)

typedef struct
{
   i2c_master_bus_handle_t bus;
   bool bus_created;
   i2c_master_dev_handle_t device;
   uint32_t measurement_delay;
   bmp180_mode_t mode;
   t_bmp180_calibration_data cal;
} bmp180_context_t;

static bool i2c_write_reg(i2c_master_dev_handle_t device, uint8_t reg, uint8_t *data, uint8_t length)
{
   uint8_t *buffer = malloc(length + 1);
   if (buffer == NULL) {
      return false;
   }
   
   buffer[0] = reg;
   memcpy(&buffer[1], data, length);
   esp_err_t result = i2c_master_transmit(device, buffer, length + 1, -1);
   free(buffer);
   
   return (result == ESP_OK);
}

static bool i2c_read_reg(i2c_master_dev_handle_t device, uint8_t reg, uint8_t *data, uint8_t length)
{
   return (i2c_master_transmit_receive(device, &reg, 1, data, length, -1) == ESP_OK);
}

static int bmp180_compensate(t_bmp180_calibration_data *cal, uint8_t oss,
    int32_t uncompensatedTemperature, int32_t uncompensatedPressure,
   int32_t *temperature, int32_t *pressure)
{
   int32_t X1, X2, B5;
   int32_t X3, B3, B6;
   int32_t T, P;
   uint32_t B4, B7;

   X1 = ((uncompensatedTemperature - (int32_t)cal->AC6) * (int32_t)cal->AC5) >> 15;
   X2 = (((int32_t)cal->MC) << 11) / (X1 + (int32_t)cal->MD);
   B5 = X1 + X2;
   T = (B5 + 8) >> 4;
   
   if(temperature != NULL)
      *temperature = T;

   if(pressure != NULL)
   {
      B6 = B5 - 4000;
      X1 = ((int32_t)cal->B2 * ((B6 * B6) >> 12)) >> 11;
      X2 = ((int32_t)cal->AC2 * B6) >> 11;
      X3 = X1 + X2;
      B3 = ((((int32_t)cal->AC1 * 4 + X3) << oss) + 2) >> 2;
      X1 = ((int32_t)cal->AC3 * B6) >> 13;
      X2 = ((int32_t)cal->B1 * ((B6 * B6) >> 12)) >> 16;
      X3 = ((X1 + X2) + 2) >> 2;
      B4 = ((uint32_t)cal->AC4 * (uint32_t)(X3 + 32768)) >> 15;
      B7 = ((uint32_t)uncompensatedPressure - B3) * (uint32_t)(50000UL >> oss);
      
      if(B7 < 0x80000000UL)
         P = (B7 * 2) / B4;
      else
         P = (B7 / B4) * 2;
         
      X1 = (P >> 8) * (P >> 8);
      X1 = (X1 * 3038) >> 16;
      X2 = (-7357 * P) >> 16;
      P += (X1 + X2 + (int32_t)3791) >> 4;
      *pressure = P;
   }
   return 0; 
}

static bool bmp180_get_uncompensated_temperature(bmp180_context_t *ctx, int32_t *ut)
{
   uint8_t d[2] = { BMP180_MEASURE_TEMP };
   if(!i2c_write_reg(ctx->device, BMP180_CONTROL_REG, d, sizeof(uint8_t)))
      return false;

   ets_delay_us(4500 + BMP180_DELAY_BUFFER);
   if(!i2c_read_reg(ctx->device, BMP180_OUT_MSB_REG, d, sizeof(d)))
      return false;
   uint32_t r = ((uint32_t)d[0] << 8) | d[1];
   *ut = r;
   ESP_LOGD(TAG, "Temperature: %" PRIi32, *ut);
   return true;
}

static bool bmp180_get_uncompensated_pressure(bmp180_context_t *ctx, uint32_t *up)
{
   uint8_t oss = ctx->mode;
   uint8_t d[3] = { BMP180_MEASURE_PRESS | (oss << 6), 0 };
   if(!i2c_write_reg(ctx->device, BMP180_CONTROL_REG, d, sizeof(uint8_t)))
      return false;
   
   ets_delay_us(ctx->measurement_delay + BMP180_DELAY_BUFFER);
   if(!i2c_read_reg(ctx->device, BMP180_OUT_MSB_REG, d, sizeof(d)))
      return false;

   uint32_t r = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
   r >>= 8 - oss; 
   *up = r;
   ESP_LOGD(TAG, "Pressure: %" PRIu32, *up);
   return true;
}

static bool bmp180_read_calibration(bmp180_context_t *ctx)
{
   for(int i = 0; i < ARRAY_SIZE(ctx->cal.raw); ++i)
   {
      uint8_t d[] = { 0, 0 };
      uint8_t reg = BMP180_CALIBRATION_REG + (i * 2);
      if(!i2c_read_reg(ctx->device, reg, d, sizeof(d)))
         return false;
      ctx->cal.raw[i] = ((uint16_t) d[0]) << 8 | (d[1]);
      if(ctx->cal.raw[i] == 0)
      {
         ESP_LOGD(TAG, "Invalid read %u", i);
         return false; 
      }
      ESP_LOGD(TAG, "Calibration value %u = %d", i, ctx->cal.raw[i]);
   }
   return true;
}

/* --------------------------------------------------------------------------------------------------------
 * Exported Functions
 */

bmp180_t bmp180_init(i2c_lowlevel_config *config, uint8_t i2c_address, bmp180_mode_t mode)
{
   bmp180_context_t *ctx;
   uint8_t id = 0;
   bool success = false;

   ctx = (bmp180_context_t *) malloc(sizeof(*ctx));
   if(NULL == ctx)
      return NULL;

   memset(ctx, 0, sizeof(*ctx));

   i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = (i2c_address == 0) ? BMP180_DEVICE_ADDRESS : i2c_address,
      .scl_speed_hz = I2C_SPEED,
   };

   if(NULL == config->bus)
   {
      i2c_master_bus_config_t bus_cfg = {
         .clk_source = I2C_CLK_SRC_DEFAULT,
         .i2c_port = config->port,
         .sda_io_num = config->pin_sda,
         .scl_io_num = config->pin_scl,
         .glitch_ignore_cnt = 7,
         .flags.enable_internal_pullup = true,      
      };
      if(i2c_new_master_bus(&bus_cfg, &ctx->bus) != ESP_OK)
      {
         ESP_LOGE(TAG, "Failed to initialize I2C bus");
         free(ctx);
         return NULL;
      }
      ctx->bus_created = true;
   }
   else
   {
      ctx->bus = *config->bus;
      ctx->bus_created = false;
   }

   if(i2c_master_bus_add_device(ctx->bus, &dev_cfg, &ctx->device) != ESP_OK)
   {
      ESP_LOGE(TAG, "I2C device initialization failed");
      if(ctx->bus_created)
         i2c_del_master_bus(ctx->bus);
      free(ctx);
      return NULL;
   }

   ctx->mode = mode;
   switch(mode)
   {
      case BMP180_MODE_ULTRA_LOW_POWER:       ctx->measurement_delay = 4500; break;
      case BMP180_MODE_STANDARD:              ctx->measurement_delay = 7500; break;
      case BMP180_MODE_HIGH_RESOLUTION:       ctx->measurement_delay = 13500; break;
      case BMP180_MODE_ULTRA_HIGH_RESOLUTION: ctx->measurement_delay = 25500; break;
      default:
         ESP_LOGE(TAG, "Invalid mode %d", mode);
         if(ctx->bus_created)
            i2c_del_master_bus(ctx->bus);
         free(ctx);
         return NULL; 
   }

   if(!i2c_read_reg(ctx->device, BMP180_VERSION_REG, &id, sizeof(id))
   || id != BMP180_CHIP_ID)
   {
      ESP_LOGE(TAG, "Invalid device ID (0x%02x, expected 0x%02x)", id, BMP180_CHIP_ID);
   }
   else if(!bmp180_read_calibration(ctx))
   {
      ESP_LOGE(TAG, "Failed to read calibration");
   }
   else
   {
      ESP_LOGD(TAG, "Initialization successful");
      success = true;
   }

   if(!success)
   {
      if(ctx->bus_created)
         i2c_del_master_bus(ctx->bus);
      free(ctx);
      ctx = NULL;
   }
   return ctx;  
}

bool bmp180_free(bmp180_t bmp)
{
   bmp180_context_t *ctx = (bmp180_context_t *) bmp;
   if(NULL == ctx)
      return false;
   
   if(ctx->bus_created)
      i2c_del_master_bus(ctx->bus);
   free(ctx);
   return true;
}

bool bmp180_measure(bmp180_t bmp, float *temperature, uint32_t *pressure)
{
   bmp180_context_t *ctx = (bmp180_context_t *) bmp;
   int32_t UT = 0;
   uint32_t UP = 0;
   int32_t T, P;

   if(NULL == ctx)
      return false;

   /* Temperature is always needed; required for pressure only. */
   if(!bmp180_get_uncompensated_temperature(ctx, &UT))
      return false;

   if(NULL != pressure)
   {
      if(!bmp180_get_uncompensated_pressure(ctx, &UP))
         return false;
   }

   if(bmp180_compensate(&ctx->cal, ctx->mode, UT, UP, &T,
      (NULL == pressure) ? NULL : &P) != 0)
   {
      return false;
   }
   
   if(NULL != temperature)
      *temperature = (float)T/10.0;
   if(NULL != pressure)
      *pressure = P;
   return true;
}