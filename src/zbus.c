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
#include <component/sercom.h>

// "configuration" address
#define CONF_ADDR 0x76

// "data poll" address
#define POLL_ADDR 0x77

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
    SercomI2cs *i2c;
    int i2c_irq;
    bool i2c_read;
    u8_t i2c_addr;

    // TODO(mbenda): stats, error counters...
} zbus;

static PTHREAD_MUTEX_DEFINE(zbus_lock);
static PTHREAD_COND_DEFINE(zbus_recv_cond);
static PTHREAD_COND_DEFINE(zbus_send_cond);

static void i2c_reset(void);
static void i2c_enable_conf(void);

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

    // reset I²C and alert pin
    i2c_reset();
    gpio_pin_write(zbus.alert_port, zbus.alert_pin, 1);

    // stop any ongoing transfers
    pthread_cond_broadcast(&zbus_recv_cond);
    pthread_cond_broadcast(&zbus_send_cond);
    // TODO(mbenda): buffers

    // configure I²C
    i2c_enable_conf();

    pthread_mutex_unlock(&zbus_lock);
    return 0;
}

bool zbus_is_ready(void)
{
    return zbus.state == ZBUS_READY;
}

// TODO(mbenda): are all sync necessary?

static inline void i2c_sync(void)
{
    while (zbus.i2c->STATUS.reg & SERCOM_I2CS_STATUS_SYNCBUSY) {
        // wait for synchronization...
    }
}

static void i2c_disable(void)
{
    zbus.i2c->INTENCLR.reg = SERCOM_I2CS_INTENCLR_MASK;
    zbus.i2c->INTFLAG.reg = SERCOM_I2CS_INTFLAG_MASK;
    NVIC_ClearPendingIRQ(zbus.i2c_irq);

    i2c_sync();
    zbus.i2c->CTRLA.reg &= ~SERCOM_I2CS_CTRLA_ENABLE;
}

void i2c_reset(void)
{
    i2c_disable();

    i2c_sync();
    zbus.i2c->CTRLA.reg |= SERCOM_I2CS_CTRLA_SWRST;
}

void i2c_enable_conf(void)
{
    i2c_sync();
    zbus.i2c->CTRLA.reg = SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE;
    // TODO CTRLA LOWTOUT SDAHOLD

    i2c_sync();
    zbus.i2c->CTRLB.reg = SERCOM_I2CS_CTRLB_AMODE(0x00);

    i2c_sync();
    zbus.i2c->ADDR.reg =
            SERCOM_I2CS_ADDR_ADDR(CONF_ADDR) | SERCOM_I2CS_ADDR_GENCEN;

    i2c_sync();
    zbus.i2c->CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;
}

void i2c_enable_ready(void)
{
    // disable I²C first
    i2c_disable();

    // reconfigure address matching
    i2c_sync();
    zbus.i2c->CTRLB.reg = SERCOM_I2CS_CTRLB_AMODE(0x01);

    i2c_sync();
    zbus.i2c->ADDR.reg =
            SERCOM_I2CS_ADDR_ADDR(zbus.addr) | SERCOM_I2CS_ADDR_GENCEN
                    | SERCOM_I2CS_ADDR_ADDRMASK(POLL_ADDR);

    i2c_sync();
    zbus.i2c->INTENSET.reg = SERCOM_I2CS_INTENSET_AMATCH;
    zbus.i2c->CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;
}

void i2c_amatch(void)
{
    // address match
    if (zbus.i2c->STATUS.reg
            & (SERCOM_I2CS_STATUS_BUSERR | SERCOM_I2CS_STATUS_COLL
                    | SERCOM_I2CS_STATUS_LOWTOUT)) {
        // a bus error occurred
//        error_received();
    }

    // get the address and check direction
    u8_t addr = zbus.i2c->DATA.reg >> 1;
    if (zbus.i2c->STATUS.reg & SERCOM_I2CS_STATUS_DIR) {
        zbus.i2c_read = true;
        // check read buffer
    } else {
        zbus.i2c_read = false;
        zbus.i2c_addr = addr;
        // check write buffer
    }
}

void i2c_drdy(void)
{
    // data ready
}

void i2c_prec(void)
{
    // stop received
//    stop_received();
    zbus.i2c->INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
}

void i2c_isr(void *arg)
{
    ARG_UNUSED(arg);

    // read interrupt flags
    u8_t status = zbus.i2c->INTFLAG.reg & zbus.i2c->INTENSET.reg;

    if (status & SERCOM_I2CS_INTFLAG_AMATCH) {
        i2c_amatch();
    } else if (status & SERCOM_I2CS_INTFLAG_DRDY) {
        i2c_drdy();
    } else if (status & SERCOM_I2CS_INTFLAG_PREC) {
        i2c_prec();
    }
}
