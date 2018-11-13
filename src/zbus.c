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
#include <logging/log.h>

LOG_MODULE_REGISTER(zbus);

// "configuration" address
#define CONF_ADDR 0x76

// "data poll" address
#define POLL_ADDR 0x77

#define SERCOM(n) SERCOM_EVAL(n)
#define SERCOM_EVAL(n) SERCOM##n

#define IRQ (SERCOM0_IRQn + CONFIG_ZBUS_SERCOM)
#define I2C (&SERCOM(CONFIG_ZBUS_SERCOM)->I2CS)

static struct {
    // the state of the bus
    enum zbus_state state;

    // address (0 when not in ZBUS_RDY state)
    s8_t addr;

    // packet receiving
    u8_t rx_buf[ZBUS_MAX_DATA];
    int rx_len;
    int rx_pos;

    // packet sending
    const u8_t *tx_buf;
    int tx_len;
    int tx_pos;

    // peripherals
    struct device *alert_port;
    u32_t alert_pin;
    bool i2c_read;
    s8_t i2c_addr;

    // TODO(mbenda): stats, error counters...
} zbus;

static struct {
    void *fifo_header;
    volatile int ev;
} zbus_event;

#define EV_DONE     0x01
#define EV_ERR_DATA 0x02
#define EV_ERR_BUS  0x04

void zbus_worker(void *, void *, void *);
static void i2c_reset(void);
static void i2c_enable_conf(void);
static void i2c_isr(void *arg);

K_FIFO_DEFINE(zbus_fifo);

// TX buffer for POLL replies
u8_t zbus_poll_buf[8];

// TX buffer for CONF replies
u8_t zbus_conf_buf[2];

static PTHREAD_MUTEX_DEFINE(zbus_lock);
static PTHREAD_COND_DEFINE(zbus_recv_cond);
static PTHREAD_COND_DEFINE(zbus_send_cond);

K_THREAD_DEFINE(zbus_thread, 1024, zbus_worker, NULL, NULL, NULL, -1, 0,
        K_NO_WAIT);

int zbus_init(struct zbus_config *cfg)
{
    // configure alert pin
    struct device *port = device_get_binding(CONFIG_ZBUS_ALERT_PORT);
    if (port == NULL) {
        return -ENODEV;
    }

    gpio_pin_configure(port, CONFIG_ZBUS_ALERT_PIN,
            GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

    // configure SERCOM I²C
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 << CONFIG_ZBUS_SERCOM;

    IRQ_DIRECT_CONNECT(IRQ, 0, i2c_isr, 0);
    irq_enable(IRQ);

// TODO mbenda: implement this

    // initialize conf buffer

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

    zbus.addr = -1;

    // configure I²C
    i2c_enable_conf();

    pthread_mutex_unlock(&zbus_lock);
    return 0;
}

bool zbus_is_ready(void)
{
    return zbus.state == ZBUS_READY;
}

void zbus_worker(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("worker started");

    while (true) {
        // read next event
        k_fifo_get(&zbus_fifo, K_FOREVER);
        int ev = zbus_event.ev;
        zbus_event.ev = 0;

        LOG_DBG("processing event %x", ev);
    }
}

// ----------------------------------------------------------------------
// SERCOM I²C bus
// ----------------------------------------------------------------------

// TODO(mbenda): are all sync necessary?

static inline void i2c_sync(void)
{
    while (I2C->STATUS.reg & SERCOM_I2CS_STATUS_SYNCBUSY) {
        // wait for synchronization...
    }
}

static inline void i2c_set_ackact(bool ack)
{
#ifdef CONFIG_I2C_SAM0_WORKAROUND
    int key = irq_lock();
    I2C->STATUS.reg = 0;

    if (ack) {
        I2C->CTRLB.reg = 0;
    } else {
        I2C->CTRLB.reg = SERCOM_I2CS_CTRLB_ACKACT;
    }
    irq_unlock(key);
#else /* !CONFIG_I2C_SAM0_WORKAROUND */
    if (ack == true) {
        I2C->CTRLB.reg &= ~SERCOM_I2CS_CTRLB_ACKACT;
    } else {
        I2C->CTRLB.reg |= SERCOM_I2CS_CTRLB_ACKACT;
    }
#endif
}

static inline void i2c_amatch_cmd3()
{
#ifdef CONFIG_I2C_SAM0_WORKAROUND
    if (I2C->INTFLAG.bit.PREC) {
        I2C->INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
    }
    I2C->INTFLAG.reg = SERCOM_I2CS_INTFLAG_AMATCH;
#else /* !CONFIG_I2C_SAM0_WORKAROUND */
    I2C->CTRLB.reg = SERCOM_I2CS_CTRLB_CMD(0x3);
#endif
}

static inline void i2c_set_tx(const void *buf, int size)
{
    zbus.tx_buf = buf;
    zbus.tx_pos = 0;
    zbus.tx_len = size;
}

static inline int i2c_notify(int ev)
{
    if (zbus_event.ev) {
        // event not consumed yet
        return -1;
    }

    zbus_event.ev = ev;

    k_fifo_put(&zbus_fifo, &zbus_event);
    return 0;
}

static void i2c_disable(void)
{
    I2C->INTENCLR.reg = SERCOM_I2CS_INTENCLR_MASK;
    I2C->INTFLAG.reg = SERCOM_I2CS_INTFLAG_MASK;
    NVIC_ClearPendingIRQ(IRQ);

    i2c_sync();
    I2C->CTRLA.reg &= ~SERCOM_I2CS_CTRLA_ENABLE;
}

void i2c_reset(void)
{
    i2c_disable();

    i2c_sync();
    I2C->CTRLA.reg |= SERCOM_I2CS_CTRLA_SWRST;
}

void i2c_enable_conf(void)
{
    i2c_sync();
    I2C->CTRLA.reg = SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE;
    // TODO CTRLA LOWTOUT SDAHOLD

    i2c_sync();
    I2C->CTRLB.reg = SERCOM_I2CS_CTRLB_AMODE(0x00) | SERCOM_I2CS_CTRLB_SMEN;

    i2c_sync();
    I2C->ADDR.reg =
            SERCOM_I2CS_ADDR_ADDR(CONF_ADDR) | SERCOM_I2CS_ADDR_GENCEN;

    i2c_sync();
    I2C->CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;
}

void i2c_enable_ready(void)
{
    // disable I²C first
    i2c_disable();

    // reconfigure address matching
    i2c_sync();
    I2C->CTRLB.reg = SERCOM_I2CS_CTRLB_AMODE(0x01) | SERCOM_I2CS_CTRLB_SMEN;

    i2c_sync();
    I2C->ADDR.reg =
            SERCOM_I2CS_ADDR_ADDR(zbus.addr) | SERCOM_I2CS_ADDR_GENCEN
                    | SERCOM_I2CS_ADDR_ADDRMASK(POLL_ADDR);

    i2c_sync();
    I2C->INTENSET.reg = SERCOM_I2CS_INTENSET_AMATCH;
    I2C->CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;
}

void i2c_amatch(void)
{
    // address match
    int ev = 0;

    if (I2C->STATUS.reg & (SERCOM_I2CS_STATUS_BUSERR
            | SERCOM_I2CS_STATUS_COLL | SERCOM_I2CS_STATUS_LOWTOUT)) {
        // a bus error occurred
        ev |= EV_ERR_BUS;
    }

    // get the address and check direction
    u8_t addr = I2C->DATA.reg >> 1;
    bool err = false;

    if (I2C->STATUS.reg & SERCOM_I2CS_STATUS_DIR) {
        // master reads data
        zbus.i2c_read = true;

        if (addr == zbus.addr) {
            // data read
            // TODO(mbenda): implement this
        } else if (addr == CONF_ADDR) {
            // configuration read
            i2c_set_tx(zbus_conf_buf, sizeof(zbus_conf_buf));
        } else if (addr == POLL_ADDR) {
            // poll read
            // TODO(mbenda): implement this
            i2c_set_tx(zbus_poll_buf, sizeof(zbus_poll_buf));
        } else {
            // NAK the address
            err = true;
        }
    } else {
        // master writes data
        zbus.i2c_read = false;
        zbus.i2c_addr = addr;

        if (zbus.rx_pos > 0) {
            // buffer is not ready
            err = true;
        }
    }

    // ACK/NAK the address
    if (err) {
        i2c_set_ackact(false);
        ev |= EV_ERR_DATA;
    } else {
        i2c_set_ackact(true);
        I2C->INTENSET.reg = SERCOM_I2CS_INTENSET_DRDY
                | SERCOM_I2CS_INTENSET_PREC;
    }

    i2c_amatch_cmd3();
    i2c_set_ackact(true);

    if (ev) {
        i2c_notify(ev);
    }
}

void i2c_drdy(void)
{
    // data ready
    bool err = false;

    if (zbus.i2c_read) {
        // master is reading, send another byte
        if (zbus.tx_pos > 0 && (I2C->STATUS.reg & SERCOM_I2CS_STATUS_RXNACK)) {
            // master NAK
            err = true;
        } else if (zbus.tx_pos >= zbus.tx_len) {
            // buffer underflow
            err = true;
        } else {
            I2C->DATA.reg = zbus.tx_buf[zbus.tx_pos++];
        }
    } else {
        // master is writing, receive another byte
        if (zbus.rx_pos >= zbus.rx_len) {
            // buffer overflow
            err = true;
        } else {
            zbus.rx_buf[zbus.rx_pos++] = I2C->DATA.reg;
        }
    }

    if (err) {
        // terminate the transfer
        i2c_set_ackact(false);
        I2C->CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x2);
        I2C->INTENCLR.reg = SERCOM_I2CS_INTENSET_DRDY
                | SERCOM_I2CS_INTENSET_PREC;

        i2c_notify(EV_ERR_DATA);
    }
}

void i2c_prec(void)
{
    // stop received
    I2C->INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
    I2C->INTENCLR.reg = SERCOM_I2CS_INTENSET_DRDY
            | SERCOM_I2CS_INTENSET_PREC;

    i2c_notify(EV_DONE);
}

void i2c_isr(void *arg)
{
    ARG_UNUSED(arg);

    // read interrupt flags
    u8_t status = I2C->INTFLAG.reg & I2C->INTENSET.reg;

    if (status & SERCOM_I2CS_INTFLAG_AMATCH) {
        i2c_amatch();
    } else if (status & SERCOM_I2CS_INTFLAG_DRDY) {
        i2c_drdy();
    } else if (status & SERCOM_I2CS_INTFLAG_PREC) {
        i2c_prec();
    }
}
