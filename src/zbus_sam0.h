/*
 * Copyright (c) 2018 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr.h>

#include "zbus.h"

/**
 * Configuration.
 */
#define CONFIG_ZBUS_SERCOM 5
#define CONFIG_ZBUS_ALERT_PORT "PORTA"
#define CONFIG_ZBUS_ALERT_PIN 21
#define CONFIG_I2C_SAM0_WORKAROUND 1

/** The maximum size of a Zbus data packet. */
#define ZBUS_MAX_DATA 128

/**
 * Enumeration of all possible Zbus states.
 */
enum zbus_state {
    ZBUS_OFF,   // The initial state before zbus_init is called.
    ZBUS_CONF,  // The configuration phase.
    ZBUS_READY, // The ready state when data packets can be sent and received.
};

/**
 * The configuration of a Zbus device.
 */
struct zbus_cfg {
    // UDID
    u8_t udid[8];
    // TODO(mbenda): other device attributes (flags, vendor, type)
};

/**
 * Initializes the bus.
 *
 * @param bus
 * @return
 */
int zbus_init_old(struct zbus_cfg *cfg);

/**
 * Receives a data packet from the master.
 *
 * @param buf data buffer to store received bytes
 * @param size size of the buffer
 * @return number of received bytes on success, -errno on failure
 */
int zbus_recv_old(void *buf, int size); // blocks

/**
 * Sends the specified data to the master.
 *
 * @param buf data buffer to send
 * @param size size of the buffer
 * @return 0 on success, -errno on failure
 */
int zbus_send_old(const void *buf, int size); // blocks

/**
 * Resets the state of the bus and starts its configuration phase.
 *
 * @return 0 on success, -errno on failure
 */
int zbus_reset_old(void);

/**
 * Checks if the bus is ready.
 *
 * @return true if ready, false if not (off or configuring)
 */
bool zbus_is_ready_old(void);
