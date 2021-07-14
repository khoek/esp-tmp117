#include <driver/i2c.h>
#include <esp_log.h>

#include "private.h"

esp_err_t tmp117_init(i2c_port_t port, uint8_t addr, tmp117_handle_t* out_dev) {
    assert(!(addr & 0x80));

    tmp117_handle_t dev = malloc(sizeof(tmp117_t));
    dev->addr = addr;
    dev->port = port;

    // As per spec, there is a power-on reset (POR) which occurs on startup.
    // Nneed to wait until EEPROM_BUSY goes low---this takes approx 1.5ms.
    while (tmp117_reg_read(dev, TMP117_REG_CONFIGURATION) & TMP117_CONFIGURATION_EEPROM_BUSY) {
        vTaskDelay(1);
    }

    tmp117_reset(dev);

    uint16_t ver = tmp117_reg_read(dev, TMP117_REG_DEVICE_ID);
    switch (ver) {
        case TMP117_DEVICE_ID_DRIVER_SUPPORTED: {
            // Current version supported by the driver.
            break;
        }
        default: {
            ESP_LOGE(TAG, "unknown device id (0x%04X), are pin numbers correct?", ver);
            free(dev);
            return ESP_FAIL;
        }
    }

    *out_dev = dev;
    return ESP_OK;
}

void tmp117_destroy(tmp117_handle_t dev) {
    free(dev);
}

#define ADDR_GENERAL_CALL_RESET 0x00
#define CMD_GENERAL_CALL_RESET 0x06

void tmp117_reset(tmp117_handle_t dev) {
    ESP_LOGD(TAG, "resetting");

    // Note that there is also a software reset method via the configuration
    // register, instead of via a general call.

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDR_GENERAL_CALL_RESET | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CMD_GENERAL_CALL_RESET, true);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(dev->port, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    // As per spec, need to wait until EEPROM_BUSY goes low after a reset.
    // This takes approx 1.5ms.
    while (tmp117_reg_read(dev, TMP117_REG_CONFIGURATION) & TMP117_CONFIGURATION_EEPROM_BUSY) {
        vTaskDelay(1);
    }
}

uint16_t tmp117_reg_read(tmp117_handle_t dev, tmp117_reg_t reg) {
    uint8_t data[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (((uint8_t) dev->addr) << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ((uint8_t) reg), true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (((uint8_t) dev->addr) << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(dev->port, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    uint16_t val = (((uint16_t) data[0]) << 8) | (((uint16_t) data[1]) << 0);
    ESP_LOGD(TAG, "reg_read(0x%02X)=0x%04X", reg, val);
    return val;
}

void tmp117_reg_write(tmp117_handle_t dev, tmp117_reg_t reg, uint16_t val) {
    uint8_t data[2];
    data[0] = (val & 0xFF00) >> 8;
    data[1] = (val & 0x00FF) >> 0;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (((uint8_t) dev->addr) << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ((uint8_t) reg), true);
    i2c_master_write(cmd, data, 2, true);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(dev->port, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    ESP_LOGD(TAG, "reg_write(0x%02X)=0x%04X", reg, val);
}
