/*
 * Copyright (c) 2018 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr.h>

enum zbus_state {
    ZBUS_OFF,
    ZBUS_CNF,
    ZBUS_RDY,
};

typedef u8_t zbus_addr;

struct zbus_data {
    // the state of the bus
    enum zbus_state state;

    // address (0 when not in ZBUS_RDY state)
    zbus_addr addr;

    // buffers

    // error counters
    // stats
};

struct zbus_config {
    // UDID
    // sercom
    // alert port/pin
};
