/*
 * Copyright (c) 2018 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zbus_sam0.h"

#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <posix/pthread.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(zbus, LOG_LEVEL_DBG);

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
    u8_t* rx_data;
    u8_t rx_len;

    // packet sending
    const u8_t *tx_data;
    u8_t tx_len;

    // peripherals
    struct device *alert_port;
    bool i2c_read;
    s8_t i2c_addr;

    // buffer
    u8_t buf[ZBUS_MAX_DATA];
    int buf_len;
    int buf_pos;
    s8_t buf_addr;

    // TODO(mbenda): stats, error counters...
} zbus;

static struct {
    void *fifo_header;
    volatile int ev;
} zbus_event;

#define EV_DONE_RX  0x01
#define EV_DONE_TX  0x02
#define EV_ERR_DATA 0x04
#define EV_ERR_BUS  0x08

static void zbus_worker(void *, void *, void *);
static void i2c_reset(void);
static void i2c_enable_conf(void);
static void i2c_isr(void *arg);

K_FIFO_DEFINE(zbus_fifo);

// TX buffer for POLL replies
u8_t zbus_poll_buf[2];

// TX buffer for CONF replies
u8_t zbus_conf_buf[8];

// RX buffer for bus requests
u8_t zbus_rx_buf[9];

static PTHREAD_MUTEX_DEFINE(zbus_lock);
static PTHREAD_COND_DEFINE(zbus_recv_cond);
static PTHREAD_COND_DEFINE(zbus_send_cond);

K_THREAD_DEFINE(zbus_thread, 1024, zbus_worker, NULL, NULL, NULL, -1, 0,
        K_NO_WAIT);

// ----------------------------------------------------------------------
// Zbus API
// ----------------------------------------------------------------------

int zbus_init_old(struct zbus_cfg *cfg)
{
    // TODO(mbenda): initialize conf buffer

    // configure alert pin
    zbus.alert_port = device_get_binding(CONFIG_ZBUS_ALERT_PORT);
    if (zbus.alert_port == NULL) {
        return -ENODEV;
    }

    gpio_pin_configure(zbus.alert_port, CONFIG_ZBUS_ALERT_PIN,
            GPIO_DIR_OUT | GPIO_PUD_PULL_UP);

    // configure SERCOM I²C
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 << CONFIG_ZBUS_SERCOM;

    IRQ_DIRECT_CONNECT(IRQ, 0, i2c_isr, 0);

    zbus_reset_old();

    return 0;
}

int zbus_recv_old(void *buf, int size)
{
    pthread_mutex_lock(&zbus_lock);

    while (zbus.tx_data != NULL) {
        // wait for other transfers to complete
        pthread_cond_wait(&zbus_recv_cond, &zbus_lock);
    }

    // set rx data atomically
    int key = irq_lock();

    zbus.rx_data = buf;
    zbus.rx_len = (u8_t) (size > ZBUS_MAX_DATA ? ZBUS_MAX_DATA : size);

    irq_unlock(key);

    // wait for transfer to complete
    pthread_cond_wait(&zbus_send_cond, &zbus_lock);

    // TODO(mbenda): return value
    pthread_mutex_unlock(&zbus_lock);
    return 0;
}

int zbus_send_old(const void *buf, int size)
{
    pthread_mutex_lock(&zbus_lock);

    while (zbus.tx_data != NULL) {
        // wait for other transfers to complete
        pthread_cond_wait(&zbus_send_cond, &zbus_lock);
    }

    // set tx data atomically
    int key = irq_lock();

    zbus.tx_data = buf;
    zbus.tx_len = (u8_t) (size > ZBUS_MAX_DATA ? ZBUS_MAX_DATA : size);

    irq_unlock(key);

    // wait for transfer to complete
    pthread_cond_wait(&zbus_send_cond, &zbus_lock);

    // TODO(mbenda): return value
    pthread_mutex_unlock(&zbus_lock);
    return 0;
}

int zbus_reset_old(void)
{
    pthread_mutex_lock(&zbus_lock);

    // move to the "conf" state
    zbus.state = ZBUS_CONF;

    // reset I²C and alert pin
    i2c_reset();
    gpio_pin_write(zbus.alert_port, CONFIG_ZBUS_ALERT_PIN, 1);

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

bool zbus_is_ready_old(void)
{
    return zbus.state == ZBUS_READY;
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

static inline void i2c_init_buf(void *buf, int size)
{
    memcpy(zbus.buf, buf, (size_t) size);
    zbus.buf_pos = 0;
    zbus.buf_len = size;
}

static inline int i2c_notify(int ev)
{
    if (zbus_event.ev) {
        // event not consumed yet
        LOG_ERR("PANIC!");
//        k_panic(); // FIXME
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

//    NVIC_ClearPendingIRQ(IRQ);
    irq_disable(IRQ);

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
    I2C->INTENSET.reg = SERCOM_I2CS_INTENSET_AMATCH;

    irq_enable(IRQ);
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

    irq_enable(IRQ);
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
    bool nak = false;

    // TODO(mbenda): check buf readiness?

    if (I2C->STATUS.reg & SERCOM_I2CS_STATUS_DIR) {
        // master reads data
        zbus.i2c_read = true;

        if (addr == zbus.addr) {
            // data read
            if (zbus.tx_data == NULL) {
                // no data to send
                ev |= EV_ERR_DATA;
                nak = true;
            } else {
                i2c_init_buf((void *) zbus.tx_data, zbus.tx_len);
            }
        } else if (addr == CONF_ADDR) {
            // configuration read
            i2c_init_buf(zbus_conf_buf, sizeof(zbus_conf_buf));
        } else if (addr == POLL_ADDR) {
            // poll read
            if (zbus.rx_data == NULL) {
                // nothing to send
                nak = true;
            } else {
                zbus.buf[0] = (u8_t) zbus.addr;
                zbus.buf[1] = (u8_t) zbus.tx_len;
                zbus.buf_pos = 0;
                zbus.buf_len = 2;
            }
        } else {
            // NAK the address
            nak = true;
        }
    } else {
        // master writes data
        zbus.i2c_read = false;
        zbus.i2c_addr = addr;

        if (addr == zbus.addr) {
            // data write
            // TODO(mbenda): implement this
        } else if (addr == 0) {
            // general call (command)
            i2c_init_buf(zbus_rx_buf, 1);
        } else if (addr == CONF_ADDR) {
            // configuration write
            i2c_init_buf(zbus_rx_buf, 8);
        } else {
            nak = true;
        }
    }

    // ACK/NAK the address
    if (nak) {
        i2c_set_ackact(false);
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
    bool nak = false;

    if (zbus.i2c_read) {
        // master is reading, send another byte
        if (zbus.buf_pos > 0 && (I2C->STATUS.reg & SERCOM_I2CS_STATUS_RXNACK)) {
            // master NAK
            nak = true;
        } else if (zbus.buf_pos >= zbus.buf_len) {
            // buffer underflow
            nak = true;
        } else {
            I2C->DATA.reg = zbus.buf[zbus.buf_pos++];
        }
    } else {
        // master is writing, receive another byte
        if (zbus.buf_pos >= zbus.buf_len) {
            // buffer overflow
            nak = true;
        } else {
            zbus.buf[zbus.buf_pos++] = I2C->DATA.reg;
        }
    }

    if (nak) {
        // terminate the transfer
        i2c_set_ackact(false);
        I2C->CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x2);
        I2C->INTENCLR.reg = SERCOM_I2CS_INTENSET_DRDY
                | SERCOM_I2CS_INTENSET_PREC;

        // TODO(mbenda): store error and send it with "done" event?
        i2c_notify(EV_ERR_DATA);
    }
}

void i2c_prec(void)
{
    // stop received
    I2C->INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
    I2C->INTENCLR.reg = SERCOM_I2CS_INTENSET_DRDY
            | SERCOM_I2CS_INTENSET_PREC;

    zbus.buf_addr = zbus.i2c_addr;
    i2c_notify(zbus.i2c_read ? EV_DONE_TX : EV_DONE_RX);
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

// ----------------------------------------------------------------------
// Zbus process
// ----------------------------------------------------------------------

static void zbus_worker(void *p1, void *p2, void *p3)
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

        pthread_mutex_lock(&zbus_lock);

        if (ev & EV_DONE_RX) {
            switch (zbus.buf_addr) {
            case 0:
                // reset
                // TODO(mbenda): check command
                LOG_DBG("reset requested");
                zbus_reset_old();
                break;

            case CONF_ADDR:
                // address configuration
                LOG_DBG("address configuration received");
                zbus.addr = zbus.buf_addr; // FIXME check UDID and set address
                i2c_enable_ready();
                break;

            default:
                if (zbus.buf_addr == zbus.addr) {
                    // data or ping received
                    LOG_INF("data packet received, len=%d", zbus.buf_len);
                    pthread_cond_broadcast(&zbus_recv_cond);
                } else {
                    LOG_ERR("unexpected RX addr: %x", zbus.buf_addr);
                }
                break;
            }
        }

        if (ev & EV_DONE_TX) {
            switch (zbus.buf_addr) {
            case POLL_ADDR:
                // poll answered
                LOG_DBG("data poll reply sent");
                break;

            case CONF_ADDR:
                // discovery answered
                LOG_DBG("discovery reply sent");
                break;

            default:
                if (zbus.buf_addr == zbus.addr) {
                    // data sent
                    LOG_DBG("data packet sent, len=%d", zbus.buf_len);
                    pthread_cond_broadcast(&zbus_send_cond);
                } else {
                    LOG_ERR("unexpected TX addr: %x", zbus.buf_addr);
                }
                break;
            }
        }

        pthread_mutex_unlock(&zbus_lock);
    }
}
