#include "hts221_i2c.h"
#include "driver/i2c.h"

/**
 * @brief Read a sequence of bytes from a HTS221 Humidity/Temperature sensor registers
 */
esp_err_t hts221_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, HTS221_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t hts221_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, HTS221_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}
int16_t read_lh(int LOW, int HIGH) {
    uint8_t data[2];
    ESP_ERROR_CHECK(hts221_register_read(LOW, data, 1));
    uint16_t LSB = data[0];
    ESP_ERROR_CHECK(hts221_register_read(HIGH, data, 1));
    uint16_t MSB = data[0];
    return ((MSB << 8) & 0x0ff00) | (LSB & 0x000ff);
}
uint8_t read(int REGISTER) {
    uint8_t data[2];
    ESP_ERROR_CHECK(hts221_register_read(REGISTER, data, 1));
    return data[0];
}