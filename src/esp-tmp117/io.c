#include <driver/i2c.h>
#include <esp_log.h>
#include <libi2c.h>

#include "device/tmp117.h"

static const char* TAG = "tmp117";

static esp_err_t regw_read(tmp117_handle_t dev, tmp117_reg_t reg, uint16_t* out_val) {
    uint8_t data[2];

    esp_err_t ret = i2c_7bit_reg8b_read(dev, reg, data, 2);
    *out_val = (((uint16_t) data[0]) << 8) | (((uint16_t) data[1]) << 0);
    return ret;
}

static esp_err_t regw_write(tmp117_handle_t dev, tmp117_reg_t reg, uint16_t val) {
    uint8_t data[2];
    data[0] = (val & 0xFF00) >> 8;
    data[1] = (val & 0x00FF) >> 0;

    return i2c_7bit_reg8b_write(dev, reg, data, 2);
}

esp_err_t tmp117_init(i2c_port_t port, uint8_t addr, tmp117_handle_t* out_dev) {
    tmp117_handle_t dev;
    i2c_7bit_init(port, addr, &dev);

    // As per spec, there is a power-on reset (POR) which occurs on startup.
    // Need to wait until EEPROM_BUSY goes low---this takes approx 1.5ms.
    uint16_t reg_cfg;
    do {
        vTaskDelay(1);

        if (regw_read(dev, TMP117_REG_CONFIGURATION, &reg_cfg) != ESP_OK) {
            ESP_LOGE(TAG, "I2C read failed, are I2C pin numbers/address correct?");
            goto tmp117_init_fail;
        }
    } while (reg_cfg & TMP117_CONFIGURATION_EEPROM_BUSY);

    uint16_t ver = tmp117_reg_read(dev, TMP117_REG_DEVICE_ID);
    switch (ver) {
        case TMP117_DEVICE_ID_DRIVER_SUPPORTED: {
            // Current version supported by the driver.
            break;
        }
        default: {
            ESP_LOGE(TAG, "unknown device id (0x%02X), have you specified the address of another device?", ver);
            goto tmp117_init_fail;
        }
    }

    tmp117_reset(dev);

    *out_dev = dev;
    return ESP_OK;

tmp117_init_fail:
    i2c_7bit_destroy(dev);
    return ESP_FAIL;
}

void tmp117_destroy(tmp117_handle_t dev) {
    i2c_7bit_destroy(dev);
}

void tmp117_reset(tmp117_handle_t dev) {
    ESP_LOGD(TAG, "resetting");

    // Note that there hardware also supports a reset via a general call.

    tmp117_reg_write(dev, TMP117_REG_CONFIGURATION, TMP117_CONFIGURATION_SOFT_RESET);
    // As per spec, reset will begin after at most 2ms.
    vTaskDelay(1 + (2 / portTICK_PERIOD_MS));

    // As per spec, need to wait until EEPROM_BUSY goes low after a reset.
    // This takes approx 1.5ms.
    while (tmp117_reg_read(dev, TMP117_REG_CONFIGURATION) & TMP117_CONFIGURATION_EEPROM_BUSY) {
        vTaskDelay(1);
    }
}

uint16_t tmp117_reg_read(tmp117_handle_t dev, tmp117_reg_t reg) {
    uint16_t val;
    ESP_ERROR_CHECK(regw_read(dev, reg, &val));

    ESP_LOGD(TAG, "reg_read(0x%02X)=0x%04X", reg, val);
    return val;
}

void tmp117_reg_write(tmp117_handle_t dev, tmp117_reg_t reg, uint16_t val) {
    ESP_ERROR_CHECK(regw_write(dev, reg, val));

    ESP_LOGD(TAG, "reg_write(0x%02X)=0x%04X", reg, val);
}
