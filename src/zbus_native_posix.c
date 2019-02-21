/*
 * Copyright (c) 2019 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>

#include <memory.h>
#include <netdb.h>
#include <unistd.h>

#include <logging/log.h>
#include <soc.h>

#include "zbus.h"

LOG_MODULE_REGISTER(zbus);

#define ZBUS_MAGIC   0x7082
#define ZBUS_VERSION 0x0000

#define CMD_PACKET 0x00
#define CMD_CONF   0x01
#define CMD_QUIT   0xFF

struct zbus_np_data {
    struct zbus_udid udid;

    zbus_addr addr;
    int sock_fd;
};

struct zbus_np_config {
    char *host;
    uint16_t port;
};

static int zbus_np_init(struct device *dev)
{
    struct zbus_np_data *data = dev->driver_data;
    memset(data, 0, sizeof(struct zbus_np_data));
    data->sock_fd = -1;

    return 0;
}

static int zbus_np_configure(struct device *dev, const struct zbus_config *cfg)
{
    struct zbus_np_data *data = dev->driver_data;
    data->udid = cfg->udid;

    return 0;
}

static int zbus_np_connect(struct device *dev)
{
    struct zbus_np_data *data = dev->driver_data;
    if (data->sock_fd >= 0) {
        // already connected
        return 0;
    }

    // connect to the Zbus simulator
    const struct zbus_np_config *cfg = dev->config->config_info;
    LOG_DBG("connecting to %s:%d", cfg->host, cfg->port);

    struct hostent *host = gethostbyname(cfg->host);
    if (!host) {
        return -ENOENT;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg->port);
    addr.sin_addr = *((struct in_addr *) host->h_addr_list[0]);

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) {
        return -errno;
    }

    if (connect(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        return -errno;
    }

    // perform handshake
    uint16_t header[2] = {htons(ZBUS_MAGIC), htons(ZBUS_VERSION)};
    if (send(fd, header, sizeof(header), 0) < sizeof(header)) {
        return -errno;
    }

    if (recv(fd, header, sizeof(header), MSG_WAITALL) < sizeof(header)) {
        return -errno;
    }

    // check magic and version
    if (ntohs(header[0]) != ZBUS_MAGIC) {
        return -EPROTO;
    }

    if ((ntohs(header[1]) & 0xFF00) != (ZBUS_VERSION & 0xFF00)) {
        return -EPROTONOSUPPORT;
    }

    // write UDID and read address
    if (send(fd, data, sizeof(data->udid), 0) == -1) {
        return -errno;
    }

    uint8_t cmd[2];
    if (recv(fd, cmd, sizeof(cmd), MSG_WAITALL) < sizeof(cmd)) {
        return -errno;
    }

    switch (cmd[0]) {
        case CMD_QUIT:
            return -ENOTCONN;

        case CMD_CONF:
            // finally connected
            data->addr = cmd[1];
            data->sock_fd = fd;
            return 0;

        default:
            return -EPROTO;
    }
}

static void zbus_np_disconnect(struct zbus_np_data *data)
{
    if (data->sock_fd >= 0) {
        close(data->sock_fd);
    }

    memset(data, 0, sizeof(struct zbus_np_data));
    data->sock_fd = -1;
}

static int zbus_np_send(struct device *dev, const void *buf, int size)
{
    if (size < 0 || size > 255) {
        return -EINVAL;
    }

    struct zbus_np_data *data = dev->driver_data;
    if (data->sock_fd < 0) {
        // not connected
        return -ENOTCONN;
    }

    uint8_t header[2] = {CMD_PACKET, (uint8_t) size};
    if (send(data->sock_fd, header, sizeof(header), 0) < sizeof(header)) {
        zbus_np_disconnect(data);
        return -errno;
    }

    if (send(data->sock_fd, buf, (size_t) size, 0) < size) {
        zbus_np_disconnect(data);
        return -errno;
    }

    return 0;
}

static int zbus_np_recv(struct device *dev, void *buf, int size)
{
    if (size < 0 || size > 255) {
        return -EINVAL;
    }

    struct zbus_np_data *data = dev->driver_data;
    if (data->sock_fd < 0) {
        // not connected
        return -ENOTCONN;
    }

    uint8_t header[2];
    if (recv(data->sock_fd, header, sizeof(header), MSG_WAITALL) != sizeof(header)) {
        zbus_np_disconnect(data);
        return -errno;
    }

    switch (header[0]) {
        case CMD_QUIT:
            zbus_np_disconnect(data);
            return -ENOTCONN;

        case CMD_PACKET:
            // continue to receive the packet payload
            break;

        default:
            zbus_np_disconnect(data);
            return -EPROTO;
    }

    size_t len = (size_t) (header[1] > size ? size : header[1]);
    if (recv(data->sock_fd, buf, len, MSG_WAITALL) != len) {
        zbus_np_disconnect(data);
        return -errno;
    }

    return len;
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

static void zbus_np_cleanup(void)
{
    zbus_np_disconnect(&zbus_np_data0);
}

NATIVE_TASK(zbus_np_cleanup, ON_EXIT, 99);
