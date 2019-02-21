/*
 * Copyright (c) 2018 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zbus.h"
#include "zbus_sam0.h"

#include <device.h>
#include <soc.h>
#include <gpio.h>
#include <posix/pthread.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(zbus, LOG_LEVEL_DBG);

// TODO(mbenda): use DTS!!!
#define DT_ZBUS_SAM0_0_LABEL                "zbus0"
#define DT_ZBUS_SAM0_0_SERCOM_BASE_ADDRESS  0x42001c00UL
#define DT_ZBUS_SAM0_0_SERCOM_IRQ           12                  + 1 /*FIXME*/
#define DT_ZBUS_SAM0_0_SERCOM_IRQ_PRIORITY  0
#define DT_ZBUS_SAM0_0_ALERT_PORT           "PORTA"
#define DT_ZBUS_SAM0_0_ALERT_PIN            21

// "configuration" address
#define CONF_ADDR 0x76

// "data poll" address
#define POLL_ADDR 0x77

#define SERCOM(n) SERCOM_EVAL(n)
#define SERCOM_EVAL(n) SERCOM##n

#define IRQ (SERCOM0_IRQn + CONFIG_ZBUS_SERCOM)
#define I2CX (&SERCOM(CONFIG_ZBUS_SERCOM)->I2CS)


// convenience device accessors
#define DEV_CFG(dev) ((const struct zbus_sam0_config *) (dev)->config->config_info)
#define DEV_DATA(dev) ((struct zbus_sam0_data *) (dev)->driver_data)
#define I2C(dev) (&((Sercom *) DEV_CFG(dev)->sercom_base)->I2CS)
#define PIN(dev) (DEV_CFG(dev)->alert_pin)

struct zbus_sam0_config {
    // SERCOM
    uintptr_t sercom_base;
    void (*sercom_irq)(struct device *dev);

    // alert signal
    const char *alert_port;
    u32_t alert_pin;
};

struct zbus_sam0_data {
    // UDID
    struct zbus_udid udid;
    // address (0 when not connected)
    zbus_addr addr;

    // packet receiving
    u8_t* rx_data;
    u8_t rx_len;

    // packet sending
    const u8_t *tx_data;
    u8_t tx_len;

    // peripherals
    struct device *alert_port; // TODO(mbenda): direct access via cfg?

    bool i2c_read;
    s8_t i2c_addr;

    // buffer
    u8_t buf[ZBUS_MAX_PACKET_LEN];
    int buf_len;
    int buf_pos;
    zbus_addr buf_addr;

    // TODO(mbenda): stats, error counters...
};

// ----------------------------------------------------------------------
// driver API implementation
// ----------------------------------------------------------------------

static int zbus_sam0_init(struct device *dev)
{
    const struct zbus_sam0_config *cfg = DEV_CFG(dev);
    struct zbus_sam0_data *data = DEV_DATA(dev);

    // configure SERCOM
    cfg->sercom_irq(dev);

    // configure alert pin
    data->alert_port = device_get_binding(cfg->alert_port);
    if (data->alert_port == NULL) {
        return -ENODEV;
    }

    int err = gpio_pin_configure(data->alert_port, PIN(dev),
            GPIO_DIR_OUT | GPIO_PUD_PULL_UP);
    if (err != 0) {
        return err;
    }

    return 0;
}

static int zbus_sam0_configure(struct device *dev, const struct zbus_config *cfg)
{
    // TODO(mbenda): disconnect, reset state

    DEV_DATA(dev)->udid = cfg->udid;
    return 0;
}

static int zbus_sam0_connect(struct device *dev)
{
    // TODO(mbenda): implement this
    return -1;
}

static int zbus_sam0_send(struct device *dev, const void *buf, int size)
{
    // TODO(mbenda): implement this
    return -1;
}

static int zbus_sam0_recv(struct device *dev, void *buf, int size)
{
    // TODO(mbenda): implement this
    return -1;
}

static const struct zbus_driver_api zbus_sam0_driver_api = {
        .configure = zbus_sam0_configure,
        .connect = zbus_sam0_connect,
        .send = zbus_sam0_send,
        .recv = zbus_sam0_recv,
};

// ----------------------------------------------------------------------
// driver implementation
// ----------------------------------------------------------------------

static void zbus_sam0_amatch(void);
static void zbus_sam0_drdy(void);
static void zbus_sam0_prec(void);

static void zbus_sam0_isr(void *arg)
{
    struct device *dev = arg;

    // read interrupt flags
    u8_t status = I2C(dev)->INTFLAG.reg & I2C(dev)->INTENSET.reg;

    if (status & SERCOM_I2CS_INTFLAG_AMATCH) {
        zbus_sam0_amatch();
    } else if (status & SERCOM_I2CS_INTFLAG_DRDY) {
        zbus_sam0_drdy();
    } else if (status & SERCOM_I2CS_INTFLAG_PREC) {
        zbus_sam0_prec();
    }
}

static void zbus_sam0_amatch(void)
{
    // TODO(mbenda): implement this
}

static void zbus_sam0_drdy(void)
{
    // TODO(mbenda): implement this
}

static void zbus_sam0_prec(void)
{
    // TODO(mbenda): implement this
}

// ----------------------------------------------------------------------
// device instances
// ----------------------------------------------------------------------

#define ZBUS_SAM0_IRQ_HANDLER_DECL(n)                                   \
static void zbus_sam0_irq_config_##n(struct device *dev)
#define ZBUS_SAM0_IRQ_HANDLER_FUNC(n)                                   \
        .sercom_irq = zbus_sam0_irq_config_##n,
#define ZBUS_SAM0_IRQ_HANDLER(n)                                        \
static void zbus_sam0_irq_config_##n(struct device *dev)                \
{                                                                       \
    IRQ_CONNECT(DT_ZBUS_SAM0_##n##_SERCOM_IRQ,                          \
                DT_ZBUS_SAM0_##n##_SERCOM_IRQ_PRIORITY,                 \
                zbus_sam0_isr, DEVICE_GET(zbus_sam0_##n),               \
                0);                                                     \
    irq_enable(DT_ZBUS_SAM0_##n##_SERCOM_IRQ);                          \
}

#define ZBUS_SAM0_CONFIG_DEFN(n)                                        \
static const struct zbus_sam0_config zbus_sam0_config_##n = {           \
        .sercom_base = DT_ZBUS_SAM0_##n##_SERCOM_BASE_ADDRESS,          \
        ZBUS_SAM0_IRQ_HANDLER_FUNC(n)                                   \
        .alert_port = DT_ZBUS_SAM0_##n##_ALERT_PORT,                    \
        .alert_pin = DT_ZBUS_SAM0_##n##_ALERT_PIN,                      \
}

#define ZBUS_SAM0_DEVICE_INIT(n)                                        \
static struct zbus_sam0_data zbus_sam0_data_##n;                        \
ZBUS_SAM0_IRQ_HANDLER_DECL(n);                                          \
ZBUS_SAM0_CONFIG_DEFN(n);                                               \
DEVICE_AND_API_INIT(zbus_sam0_##n, DT_ZBUS_SAM0_##n##_LABEL,            \
                    zbus_sam0_init, &zbus_sam0_data_##n,                \
                    &zbus_sam0_config_##n, APPLICATION,                 \
                    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                 \
                    &zbus_sam0_driver_api);                             \
ZBUS_SAM0_IRQ_HANDLER(n)

#ifdef DT_ZBUS_SAM0_0_LABEL
ZBUS_SAM0_DEVICE_INIT(0)
#endif

#ifdef DT_ZBUS_SAM0_1_LABEL
ZBUS_SAM0_DEVICE_INIT(1)
#endif

#ifdef DT_ZBUS_SAM0_2_LABEL
ZBUS_SAM0_DEVICE_INIT(2)
#endif

#ifdef DT_ZBUS_SAM0_3_LABEL
ZBUS_SAM0_DEVICE_INIT(3)
#endif

#ifdef DT_ZBUS_SAM0_4_LABEL
ZBUS_SAM0_DEVICE_INIT(4)
#endif

#ifdef DT_ZBUS_SAM0_5_LABEL
ZBUS_SAM0_DEVICE_INIT(5)
#endif

// ----------------------------------------------------------------------
// OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD OLD
// ----------------------------------------------------------------------

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
    while (I2CX->STATUS.reg & SERCOM_I2CS_STATUS_SYNCBUSY) {
        // wait for synchronization...
    }
}

static inline void i2c_set_ackact(bool ack)
{
#ifdef CONFIG_I2C_SAM0_WORKAROUND
    int key = irq_lock();
    I2CX->STATUS.reg = 0;

    if (ack) {
        I2CX->CTRLB.reg = 0;
    } else {
        I2CX->CTRLB.reg = SERCOM_I2CS_CTRLB_ACKACT;
    }
    irq_unlock(key);
#else /* !CONFIG_I2C_SAM0_WORKAROUND */
    if (ack == true) {
        I2CX->CTRLB.reg &= ~SERCOM_I2CS_CTRLB_ACKACT;
    } else {
        I2CX->CTRLB.reg |= SERCOM_I2CS_CTRLB_ACKACT;
    }
#endif
}

static inline void i2c_amatch_cmd3()
{
#ifdef CONFIG_I2C_SAM0_WORKAROUND
    if (I2CX->INTFLAG.bit.PREC) {
        I2CX->INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
    }
    I2CX->INTFLAG.reg = SERCOM_I2CS_INTFLAG_AMATCH;
#else /* !CONFIG_I2C_SAM0_WORKAROUND */
    I2CX->CTRLB.reg = SERCOM_I2CS_CTRLB_CMD(0x3);
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
    I2CX->INTENCLR.reg = SERCOM_I2CS_INTENCLR_MASK;
    I2CX->INTFLAG.reg = SERCOM_I2CS_INTFLAG_MASK;

//    NVIC_ClearPendingIRQ(IRQ);
    irq_disable(IRQ);

    i2c_sync();
    I2CX->CTRLA.reg &= ~SERCOM_I2CS_CTRLA_ENABLE;
}

void i2c_reset(void)
{
    i2c_disable();

    i2c_sync();
    I2CX->CTRLA.reg |= SERCOM_I2CS_CTRLA_SWRST;
}

void i2c_enable_conf(void)
{
    i2c_sync();
    I2CX->CTRLA.reg = SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE;
    // TODO CTRLA LOWTOUT SDAHOLD

    i2c_sync();
    I2CX->CTRLB.reg = SERCOM_I2CS_CTRLB_AMODE(0x00) | SERCOM_I2CS_CTRLB_SMEN;

    i2c_sync();
    I2CX->ADDR.reg =
            SERCOM_I2CS_ADDR_ADDR(CONF_ADDR) | SERCOM_I2CS_ADDR_GENCEN;

    i2c_sync();
    I2CX->CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;
    I2CX->INTENSET.reg = SERCOM_I2CS_INTENSET_AMATCH;

    irq_enable(IRQ);
}

void i2c_enable_ready(void)
{
    // disable I²C first
    i2c_disable();

    // reconfigure address matching
    i2c_sync();
    I2CX->CTRLB.reg = SERCOM_I2CS_CTRLB_AMODE(0x01) | SERCOM_I2CS_CTRLB_SMEN;

    i2c_sync();
    I2CX->ADDR.reg =
            SERCOM_I2CS_ADDR_ADDR(zbus.addr) | SERCOM_I2CS_ADDR_GENCEN
                    | SERCOM_I2CS_ADDR_ADDRMASK(POLL_ADDR);

    i2c_sync();
    I2CX->INTENSET.reg = SERCOM_I2CS_INTENSET_AMATCH;
    I2CX->CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;

    irq_enable(IRQ);
}

void i2c_amatch(void)
{
    // address match
    int ev = 0;

    if (I2CX->STATUS.reg & (SERCOM_I2CS_STATUS_BUSERR
            | SERCOM_I2CS_STATUS_COLL | SERCOM_I2CS_STATUS_LOWTOUT)) {
        // a bus error occurred
        ev |= EV_ERR_BUS;
    }

    // get the address and check direction
    u8_t addr = I2CX->DATA.reg >> 1;
    bool nak = false;

    // TODO(mbenda): check buf readiness?

    if (I2CX->STATUS.reg & SERCOM_I2CS_STATUS_DIR) {
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
        I2CX->INTENSET.reg = SERCOM_I2CS_INTENSET_DRDY
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
        if (zbus.buf_pos > 0 && (I2CX->STATUS.reg & SERCOM_I2CS_STATUS_RXNACK)) {
            // master NAK
            nak = true;
        } else if (zbus.buf_pos >= zbus.buf_len) {
            // buffer underflow
            nak = true;
        } else {
            I2CX->DATA.reg = zbus.buf[zbus.buf_pos++];
        }
    } else {
        // master is writing, receive another byte
        if (zbus.buf_pos >= zbus.buf_len) {
            // buffer overflow
            nak = true;
        } else {
            zbus.buf[zbus.buf_pos++] = I2CX->DATA.reg;
        }
    }

    if (nak) {
        // terminate the transfer
        i2c_set_ackact(false);
        I2CX->CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x2);
        I2CX->INTENCLR.reg = SERCOM_I2CS_INTENSET_DRDY
                | SERCOM_I2CS_INTENSET_PREC;

        // TODO(mbenda): store error and send it with "done" event?
        i2c_notify(EV_ERR_DATA);
    }
}

void i2c_prec(void)
{
    // stop received
    I2CX->INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
    I2CX->INTENCLR.reg = SERCOM_I2CS_INTENSET_DRDY
            | SERCOM_I2CS_INTENSET_PREC;

    zbus.buf_addr = zbus.i2c_addr;
    i2c_notify(zbus.i2c_read ? EV_DONE_TX : EV_DONE_RX);
}

void i2c_isr(void *arg)
{
    ARG_UNUSED(arg);

    // read interrupt flags
    u8_t status = I2CX->INTFLAG.reg & I2CX->INTENSET.reg;

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
