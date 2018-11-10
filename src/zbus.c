/*
 * Copyright (c) 2018 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zbus.h"

#include <device.h>
#include <soc.h>
#include <gpio.h>

int zbus_init(struct zbus_device *bus)
{
    // configure alert pin
    struct device *port = device_get_binding(bus->config->port_label);
    if (port == NULL) {
        return -ENODEV;
    }

    gpio_pin_configure(port, bus->config->pin, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

    // configure SERCOM I²C
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 << bus->config->sercom;

// TODO mbenda: implement this
    return 0;
}
