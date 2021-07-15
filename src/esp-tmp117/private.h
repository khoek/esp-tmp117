#ifndef __LIB__TMP117_PRIVATE_H
#define __LIB__TMP117_PRIVATE_H

#include <driver/i2c.h>

#include "tmp117.h"

static const char* TAG = "tmp117";

struct tmp117 {
    i2c_port_t port;
    uint8_t addr;
};

#endif
