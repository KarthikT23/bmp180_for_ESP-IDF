/*! \copyright 2024 Zorxx Software. All rights reserved.
 *  \license This file is released under the MIT License. See the LICENSE file for details.
 *  \brief bmp180 library esp-idf example application
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bmp180.h"

#define TAG "BMP180_EXAMPLE"

/* The following definitions may change, based on the ESP device,
   BMP180 device configuration, and wiring between them. */
#define ESP_I2C_PORT I2C_NUM_0
#define ESP_I2C_SDA  GPIO_NUM_21
#define ESP_I2C_SCL  GPIO_NUM_22
#define DEVICE_I2C_ADDRESS 0 /* let the library figure it out */

void app_main(void)
{
   ESP_ERROR_CHECK(nvs_flash_init());
   ESP_ERROR_CHECK(esp_event_loop_create_default());
   
   i2c_lowlevel_config config = {0};
   config.port = ESP_I2C_PORT;
   config.pin_sda = ESP_I2C_SDA;
   config.pin_scl = ESP_I2C_SCL;
   config.bus = NULL; /* Let the library create the bus */
   
   bmp180_t ctx = bmp180_init(&config, DEVICE_I2C_ADDRESS, BMP180_MODE_HIGH_RESOLUTION);
   if(NULL == ctx)
   {
      ESP_LOGE(TAG, "BMP180 initialization failed");
   }
   else
   {
      ESP_LOGI(TAG, "BMP180 initialized successfully");
      
      float temperature;
      uint32_t pressure; 
      
      for(int i = 0; i < 100; ++i)
      {
         if(!bmp180_measure(ctx, &temperature, &pressure))
         {
            ESP_LOGE(TAG, "BMP180 measurement failed");
         }
         else
         {
            float pressure_hPa = (float)pressure / 100.0;
            float pressure_mmHg = (float)pressure * 0.00750062;
            float pressure_inHg = (float)pressure * 0.0002953;
            
            ESP_LOGI(TAG, "Measurement %d:", i + 1);
            ESP_LOGI(TAG, "  Temperature: %.2f °C", temperature);
            ESP_LOGI(TAG, "  Pressure: %" PRIu32 " Pa (%.2f hPa, %.2f mmHg, %.2f inHg)", 
                     pressure, pressure_hPa, pressure_mmHg, pressure_inHg);
         }
         vTaskDelay(pdMS_TO_TICKS(1000)); /* Wait 1 second between measurements */
      }
      
      if(!bmp180_free(ctx))
      {
         ESP_LOGE(TAG, "Failed to free BMP180 context");
      }
      else
      {
         ESP_LOGI(TAG, "BMP180 context freed successfully");
      }
   }
   
   ESP_LOGI(TAG, "BMP180 test application finished");
   
   /* Keep the task alive */
   for(;;)
   {
      vTaskDelay(portMAX_DELAY);
   }
}