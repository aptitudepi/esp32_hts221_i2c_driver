#ifndef HTS221_I2C_H
#define HTS221_I2C_H


#define I2C_MASTER_SCL_IO           1      /*!< GPIO Pin number used for I2C master clock */
#define I2C_MASTER_SDA_IO           0      /*!< GPIO Pin number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define HTS221_SENSOR_ADDR          0x5F    /*!< Slave (Chip) address of the HTS221 Humidity and Temperature sensor */
#define HTS221_WHO_AM_I             0x0F    /*!< Register Address of the "who am I" register for the HTS221 */
#define HTS221_AV_CONF              0x10    /*!< Register Address for Humidity/Temperature Resolution/Error Value Configuration */
#define HTS221_CTRL_REG1            0x20    /*!< Register addresses for power managment/data update frquency Configuratiom */
#define HTS221_CTRL_REG2            0x21    /*!< Register Address for Reboot Memory Content / Internal Heating Element Configuration */
#define HTS221_CTRL_REG3            0x22    /*!< Register Address for DRDY (Data Ready Output Signal) that activates when receiving 
                                               new data and deactivates after both HUMIDITY_OUT_H and TEMP_OUT_H registers are read. */
#define HTS221_STATUS_REG           0x27    /*!< Register Address for the HTS221's staus register. 
                                               The register is activated when there is new humidity (H_DA) or temperature (T_DA) data available.
                                               Reading HUMIDITY_OUT_H (0x29) clears the first bit (H_DA), while reading TEMP_OUT_H (0x2B) clears the second bit (T_DA). */
#define HTS221_RESET_BIT            7       

/*! Output Registers: 0x28 => 0x2B */
//! Relative Humidity Data expressed as signed 16-bit integers (s16) stored in 2 8-bit registers (0x28, 0x29)
#define HTS221_HUMIDITY_OUT_L 0x28  
#define HTS221_HUMIDITY_OUT_H 0x29
//! Relative Temperature Data expressed as signed 16-bit integers (s16) stored in 2 8-bit registers (0x2A, 0x2B)
#define HTS221_TEMP_OUT_L 0x2A
#define HTS221_TEMP_OUT_H 0x2B

/*! Calibration Coefficient Registers: 0x30 => 0x3F */
#define HTS221_H0_rH_x2 0x30
#define HTS221_H1_rH_x2 0x31
#define HTS221_T0_degC_x8 0x32
#define HTS221_T1_degC_x8 0x33
//! 0x35 is a reserved register/bit
#define HTS221_T1_T0_MSB 0x35
#define HTS221_H0_T0_OUT_L 0x36
#define HTS221_H0_T0_OUT_H 0x37
//! 0x38 is a reserved register/bit
//! 0x39 is a reserved register/bit
#define HTS221_H1_T0_OUT_L 0x3A
#define HTS221_H1_T0_OUT_H 0x3B
#define HTS221_T0_OUT_L 0x3C
#define HTS221_T0_OUT_H 0x3D
#define HTS221_T1_OUT_L 0x3E
#define HTS221_T1_OUT_H 0x3F

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
/**
 * @brief Read a sequence of bytes from a HTS221 Humidity/Temperature sensor registers
 */
esp_err_t hts221_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t hts221_register_write_byte(uint8_t reg_addr, uint8_t data);
int16_t read_lh(int LOW, int HIGH);
uint8_t read(int REGISTER);


#endif /* HTS221_I2C_H */