/*
 * Copyright (c) 2018 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zbus.h"

#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <posix/pthread.h>

static struct {
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

    // peripherals
    struct device *alert_port;
    u32_t alert_pin;

    // TODO(mbenda): stats, error counters...
} zbus;

static PTHREAD_MUTEX_DEFINE(zbus_lock);
static PTHREAD_COND_DEFINE(zbus_recv_cond);
static PTHREAD_COND_DEFINE(zbus_send_cond);

static void i2c_reset(void);

int zbus_init(struct zbus_config *cfg)
{
    // configure alert pin
    struct device *port = device_get_binding(cfg->port_label);
    if (port == NULL) {
        return -ENODEV;
    }

    gpio_pin_configure(port, cfg->pin, GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

    // configure SERCOM I²C
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 << cfg->sercom;

// TODO mbenda: implement this

    zbus.state = ZBUS_CONF;
    return 0;
}

int zbus_recv(void *buf, int size)
{
// TODO mbenda: implement this
    return 0;
}

int zbus_send(const void *buf, int size)
{
// TODO mbenda: implement this
    return 0;
}

int zbus_reset(void)
{
    pthread_mutex_lock(&zbus_lock);

    // move to the "conf" state
    zbus.state = ZBUS_CONF;

    // stop any ongoing transfers
    pthread_cond_broadcast(&zbus_recv_cond);
    pthread_cond_broadcast(&zbus_send_cond);
    // TODO(mbenda): buffers

    // reset I²C and alert pin
    i2c_reset();
    gpio_pin_write(zbus.alert_port, zbus.alert_pin, 1);

    // TODO configure I²C

    pthread_mutex_unlock(&zbus_lock);
    return 0;
}

bool zbus_is_ready(void)
{
    return zbus.state == ZBUS_READY;
}

void i2c_reset(void)
{
//    i2c_sync();
//    I2C.INTENCLR.reg = SERCOM_I2CS_INTENCLR_MASK;
//    I2C.INTFLAG.reg = SERCOM_I2CS_INTFLAG_MASK;
//    I2C.CTRLA.reg &= ~SERCOM_I2CS_CTRLA_ENABLE;
//
//    i2c_state = (struct i2c_state) {false, false, 0};
//
//    i2c_sync();
//    I2C.CTRLA.reg |= SERCOM_I2CS_CTRLA_SWRST;
}
