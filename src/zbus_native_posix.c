/*
 * Copyright (c) 2019 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zbus.h"

struct zbus_np_data {

};

struct zbus_np_config {
    char *host;
    uint16_t port;
};

static int zbus_np_init(struct device *dev)
{
    return 0;
}

static int zbus_np_configure(struct device *dev, const struct zbus_config *cfg)
{
    return 0;
}

static int zbus_np_connect(struct device *dev)
{
    return 0;
}

static int zbus_np_send(struct device *dev, const void *buf, int size)
{
    return 0;
}

static int zbus_np_recv(struct device *dev, void *buf, int size)
{
    return 0;
}

static const struct zbus_driver_api zbus_np_driver_api = {
        .configure = zbus_np_configure,
        .connect = zbus_np_connect,
        .send = zbus_np_send,
        .recv = zbus_np_recv,
};

static const struct zbus_np_config zbus_np_cfg0 = {
        .host = "localhost",
        .port = 7802,
};

static struct zbus_np_data zbus_np_data0;

DEVICE_AND_API_INIT(uart_native_posix0, "zbus0", zbus_np_init,
        &zbus_np_data0, &zbus_np_cfg0, APPLICATION,
        CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &zbus_np_driver_api);
