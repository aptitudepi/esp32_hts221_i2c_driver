#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "hts221_i2c.h"

static const char *TAG = "hts221-i2c";

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    uint8_t data[1];
    hts221_register_read(HTS221_CTRL_REG1, data, 1);
    data[0] |= 0b00000011;
    hts221_register_write_byte(HTS221_CTRL_REG1, data[0]);

    int16_t HTS221_H1_T0_OUT = read_lh(HTS221_H1_T0_OUT_L, HTS221_H1_T0_OUT_H);
    int16_t HTS221_H0_T0_OUT = read_lh(HTS221_H0_T0_OUT_L, HTS221_H0_T0_OUT_H);
    uint8_t HTS221_H0_rH     = read(HTS221_H0_rH_x2);
    uint8_t HTS221_H1_rH     = read(HTS221_H1_rH_x2);
    float rH_slope =  ((HTS221_H1_rH- HTS221_H0_rH) / 2.0) / (HTS221_H1_T0_OUT - HTS221_H0_T0_OUT);
    printf("rH_slope = %f\n", rH_slope);
    float rH_intercept = (HTS221_H0_rH / 2.0) - (rH_slope * HTS221_H0_T0_OUT);
    printf ("rH_intercept = %f\n", rH_intercept);
    while (1) {
        hts221_register_read(HTS221_STATUS_REG, data, 1);
        if (data[0] & 0b00000010) {
            int16_t HTS221_HUMIDITY_OUT = read_lh(HTS221_HUMIDITY_OUT_L, HTS221_HUMIDITY_OUT_H);
            float HUMIDITY = (rH_slope * HTS221_HUMIDITY_OUT) + rH_intercept;
            printf("HUMIDITY %f\n", HUMIDITY);
        }
        
    }
    
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}