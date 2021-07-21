#ifndef __LIB_TMP117_H
#define __LIB_TMP117_H

#include <driver/i2c.h>
#include <libi2c.h>

// The device id of the TMP117 supported by this driver.
#define TMP117_DEVICE_ID_DRIVER_SUPPORTED 0x0117

typedef enum tmp117_reg {
    TMP117_REG_TEMP_RESULT = 0x00,
    TMP117_REG_CONFIGURATION = 0x01,
    TMP117_REG_THIGH_LIMIT = 0x02,
    TMP117_REG_TLOW_LIMIT = 0x03,
    TMP117_REG_EEPROM_UL = 0x04,
    TMP117_REG_EEPROM1 = 0x05,
    TMP117_REG_EEPROM2 = 0x06,
    TMP117_REG_TEMP_OFFSET = 0x07,
    TMP117_REG_EEPROM3 = 0x08,

    TMP117_REG_DEVICE_ID = 0x0F,
} tmp117_reg_t;

#define TMP117_CONFIGURATION_DATA_READY (1ULL << 13)
#define TMP117_CONFIGURATION_EEPROM_BUSY (1ULL << 12)

#define TMP117_CONFIGURATION_MOD_CC (0b00ULL << 10)
#define TMP117_CONFIGURATION_MOD_SD (0b01ULL << 10)
#define TMP117_CONFIGURATION_MOD_OS (0b11ULL << 10)

#define TMP117_CONFIGURATION_AVG_NO (0b00ULL << 5)
#define TMP117_CONFIGURATION_AVG_8 (0b01ULL << 5)
#define TMP117_CONFIGURATION_AVG_32 (0b10ULL << 5)
#define TMP117_CONFIGURATION_AVG_64 (0b11ULL << 5)

#define TMP117_CONFIGURATION_SOFT_RESET (1ULL << 1)

#define TMP117_EEPROM_UL_EUN (1ULL << 15)

typedef i2c_7bit_handle_t tmp117_handle_t;

// Register the TMP117 on the given I2C bus.
esp_err_t tmp117_init(i2c_port_t port, uint8_t addr, tmp117_handle_t* out_dev);

// Release the given handle.
void tmp117_destroy(tmp117_handle_t dev);

// Reset the TMP117.
void tmp117_reset(tmp117_handle_t dev);

// Read a register over I2C.
uint16_t tmp117_reg_read(tmp117_handle_t dev, tmp117_reg_t addr);

// Write a register over I2C.
void tmp117_reg_write(tmp117_handle_t dev, tmp117_reg_t addr, uint16_t val);

#endif
