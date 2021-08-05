#include <driver/i2c.h>
#include <esp_log.h>
#include <libesp.h>
#include <libesp/marshall.h>
#include <libi2c.h>

#include "device/tmp117.h"

static const char* TAG = "tmp117";

esp_err_t tmp117_init(i2c_port_t port, uint8_t addr, tmp117_handle_t* out_dev) {
    esp_err_t ret;

    tmp117_handle_t dev;
    i2c_7bit_init(port, addr, &dev);

    // As per spec, there is a power-on reset (POR) which occurs on startup.
    // Need to wait until EEPROM_BUSY goes low---this takes approx 1.5ms.
    uint16_t reg_cfg;
    do {
        vTaskDelay(1);

        ret = tmp117_reg_read(dev, TMP117_REG_CONFIGURATION, &reg_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "I2C read failed, are I2C pin numbers/address correct?");
            goto tmp117_init_fail;
        }
    } while (reg_cfg & TMP117_CONFIGURATION_EEPROM_BUSY);

    uint16_t ver;
    ret = tmp117_reg_read(dev, TMP117_REG_DEVICE_ID, &ver);
    if (ret != ESP_OK) {
        goto tmp117_init_fail;
    }

    switch (ver) {
        case TMP117_DEVICE_ID_DRIVER_SUPPORTED: {
            // Current version supported by the driver.
            break;
        }
        default: {
            ret = ESP_FAIL;

            ESP_LOGE(TAG,
                     "unknown device id (0x%02X), have you specified the "
                     "address of another device?",
                     ver);
            goto tmp117_init_fail;
        }
    }

    ret = tmp117_reset(dev);
    if (ret != ESP_OK) {
        goto tmp117_init_fail;
    }

    *out_dev = dev;
    return ESP_OK;

tmp117_init_fail:
    tmp117_destroy(dev);
    return ret;
}

void tmp117_destroy(tmp117_handle_t dev) {
    ESP_ERROR_DISCARD(tmp117_reg_write(dev, TMP117_REG_CONFIGURATION,
                                       TMP117_CONFIGURATION_SOFT_RESET));
    i2c_7bit_destroy(dev);
}

esp_err_t tmp117_reset(tmp117_handle_t dev) {
    esp_err_t ret;

    ESP_LOGD(TAG, "resetting");

    // Note that the hardware also supports a reset via a general call.

    ret = tmp117_reg_write(dev, TMP117_REG_CONFIGURATION,
                           TMP117_CONFIGURATION_SOFT_RESET);
    if (ret != ESP_OK) {
        return ret;
    }

    // As per spec, reset will begin after at most 2ms.
    vTaskDelay(1 + (2 / portTICK_PERIOD_MS));

    // As per spec, need to wait until EEPROM_BUSY goes low after a reset.
    // This takes approx 1.5ms.
    uint16_t cfg;
    while (1) {
        ret = tmp117_reg_read(dev, TMP117_REG_CONFIGURATION, &cfg);
        if (ret != ESP_OK) {
            return ret;
        }

        if (!(cfg & TMP117_CONFIGURATION_EEPROM_BUSY)) {
            break;
        }

        vTaskDelay(1);
    }

    return ESP_OK;
}

esp_err_t tmp117_reg_read(tmp117_handle_t dev, tmp117_reg_t reg,
                          uint16_t* val) {
    uint8_t data[2];
    esp_err_t ret = i2c_7bit_reg8b_read(dev, reg, data, 2);

    marshall_2u8_to_1u16_be(val, data);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "reg_read(0x%02X)=0x%04X", reg, *val);
    } else {
        ESP_LOGE(TAG, "reg_read(0x%02X)=? <ERR>:0x%X", reg, ret);
    }

    return ret;
}

esp_err_t tmp117_reg_write(tmp117_handle_t dev, tmp117_reg_t reg,
                           uint16_t val) {
    uint8_t data[2];
    marshall_1u16_to_2u8_be_args(&data[0], &data[1], val);

    esp_err_t ret = i2c_7bit_reg8b_write(dev, reg, data, 2);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "reg_write(0x%02X)=0x%04X", reg, val);
    } else {
        ESP_LOGE(TAG, "reg_write(0x%02X)=0x%04X <ERR>:0x%X", reg, val, ret);
    }

    return ret;
}
