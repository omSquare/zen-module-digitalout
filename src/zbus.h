/*
 * Copyright (c) 2018 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr.h>

/** The maximum size of a Zbus data packet. */
#define ZBUS_MAX_DATA 128

/**
 * Enumeration of all possible Zbus states.
 */
enum zbus_state {
    /* The initial state before zbus_init is called. */
            ZBUS_OFF,

    /* The configuration phase when the bus waits for address assignment. */
            ZBUS_CONF,

    /* The ready state when data packets can be sent and received. */
            ZBUS_READY,
};

struct zbus_device {
    const struct zbus_config *config;
    struct zbus_data *data;
};

/**
 * The state data of a Zbus device.
 */
struct zbus_data {
    // the state of the bus
    enum zbus_state state;

    // address (0 when not in ZBUS_RDY state)
    u8_t addr;

    // packet receiving
    u8_t recv_buf[ZBUS_MAX_DATA];
    int recv_len;
    int recv_pos;

    // packet sending
    const u8_t *send_buf;
    int send_len;
    int send_pos;

    // TODO(mbenda): stats, error counters...
};

/**
 * The configuration of a Zbus device.
 */
struct zbus_config {
    // UDID
    u8_t udid[8];
    // TODO(mbenda): other device attributes (flags, vendor, type)

    // SERCOM I²C index
    int sercom;

    // alert port/pin
    const char *port_label;
    u32_t pin;
};

/**
 * Initializes the bus.
 *
 * @param bus
 * @return
 */
int zbus_init(struct zbus_device *bus);

/**
 * Receives a data packet from the master.
 *
 * @param zbus the Zbus device
 * @param buf data buffer to store received bytes
 * @param size size of the buffer
 * @return number of received bytes on success, -errno on failure
 */
int zbus_recv(struct zbus_device *bus, void *buf, int size); // blocks

/**
 * Sends the specified data to the master.
 *
 * @param bus the Zbus device
 * @param buf data buffer to send
 * @param size size of the buffer
 * @return 0 on success, -errno on failure
 */
int zbus_send(struct zbus_device *bus, const void *buf, int size); // blocks

/**
 * Resets the state of the bus and starts its configuration phase.
 *
 * @param bus the Zbus device
 * @return 0 on success, -errno on failure
 */
int zbus_reset(struct zbus_device *bus);

/**
 * Checks if the bus is ready.
 *
 * @param bus the Zbus device
 * @return true if ready, false if not (off or configuring)
 */
bool zbus_is_ready(struct zbus_device *bus);
