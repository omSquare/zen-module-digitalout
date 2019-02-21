/*
 * Copyright (c) 2019 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <device.h>

// // // //
//  API  //
// // // //

typedef uint8_t zbus_addr;

struct zbus_udid {
    u8_t id[8];
};

struct zbus_config {
    // UDID
    struct zbus_udid udid;

    // TODO(mbenda): flags, device class etc.
};

struct zbus_driver_api {
    int (*configure)(struct device *dev, const struct zbus_config *cfg);
    int (*connect)(struct device *dev);
    int (*send)(struct device *dev, const void *buf, int size);
    int (*recv)(struct device *dev, void *buf, int size);
};

/**
 * Initializes the Zbus driver. Disconnects it if it is connected.
 *
 * @param dev the Zbus device
 * @param cfg the Zbus configuration
 * @return 0 on success, -errno on failure
 */
static inline
int zbus_configure(struct device *dev, const struct zbus_config *cfg)
{
    return ((struct zbus_driver_api *) dev->driver_api)->configure(dev, cfg);
}

/**
 * Ensures that the slave is connected to a master and properly configured. Does
 * nothing if already connected and configured.
 *
 * @param dev the Zbus device
 * @return
 */
static inline
int zbus_connect(struct device *dev)
{
    return ((struct zbus_driver_api *) dev->driver_api)->connect(dev);
}

/**
 * Sends the specified data to the master.
 *
 * @param dev the Zbus device
 * @param buf data buffer to send
 * @param size size of the buffer
 * @return 0 on success, -errno on failure
 */
static inline
int zbus_send(struct device *dev, const void *buf, int size)
{
    return ((struct zbus_driver_api *) dev->driver_api)->send(dev, buf, size);
}

/**
 * Receives a data packet from the master.
 *
 * @param dev the Zbus device
 * @param buf data buffer to store received bytes
 * @param size size of the buffer
 * @return number of received bytes on success, -errno on failure
 */
static inline
int zbus_recv(struct device *dev, void *buf, int size)
{
    return ((struct zbus_driver_api *) dev->driver_api)->recv(dev, buf, size);
}
