#include <zephyr.h>
#include <gpio.h>
#include <pinmux.h>
#include <soc.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(main);

#define LED_PORT LED0_GPIO_CONTROLLER
#define LED LED0_GPIO_PIN

#define CONF_ADDR 0x76
#define POLL_ADDR 0x77

#define I2C (SERCOM5->I2CS)
#define IRQ SERCOM5_IRQn

#define WORKAROUND 0

/////////////////////////////////
// PB16(SDA) PB17(SCL) SERCOM5 //
/////////////////////////////////
/////////////////////////////////
// PA08(SDA) PA09(SCL) SERCOM0 //
/////////////////////////////////
/////////////////////////////////
// PA22(SDA) PA23(SCL) SERCOM5 //
/////////////////////////////////

struct i2c_state {
    bool read_dir;
    bool read_start;

    u8_t read_data;
};

static struct i2c_state i2c_state;

static void isr_i2c_slave(void *);

static inline void i2c_sync(void)
{
    while (I2C.STATUS.reg & SERCOM_I2CS_STATUS_SYNCBUSY) {
        /* Wait for synchronization... */
    }
}

void i2c_reset(void)
{
    i2c_sync();
    I2C.INTENCLR.reg = SERCOM_I2CS_INTENCLR_MASK;
    I2C.INTFLAG.reg = SERCOM_I2CS_INTFLAG_MASK;
    I2C.CTRLA.reg &= ~SERCOM_I2CS_CTRLA_ENABLE;

    i2c_state = (struct i2c_state) {false, false, 0};

    i2c_sync();
    I2C.CTRLA.reg |= SERCOM_I2CS_CTRLA_SWRST;
}

void i2c_init_slave(void)
{
    i2c_sync();
    I2C.CTRLA.reg =
            SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE;// | SERCOM_I2CS_CTRLA_SDAHOLD(0x2);
    // TODO CTRLA LOWTOUT SDAHOLD

    i2c_sync();
//    I2C.CTRLB.reg |= SERCOM_I2CS_CTRLB_AMODE(0) | SERCOM_I2CS_CTRLB_SMEN;
    I2C.CTRLB.reg = SERCOM_I2CS_CTRLB_AMODE(0);
    // TODO CTRLB AMODE SMEN

    i2c_sync();
    I2C.ADDR.reg = SERCOM_I2CS_ADDR_ADDR(CONF_ADDR) | SERCOM_I2CS_ADDR_GENCEN;
    // TODO ADDR ADDRMASK

    NVIC_ClearPendingIRQ(IRQ);
    IRQ_CONNECT(IRQ, 0, isr_i2c_slave, 0, 0);
    irq_enable(IRQ);

    I2C.INTENSET.reg = SERCOM_I2CS_INTENSET_DRDY | SERCOM_I2CS_INTENSET_AMATCH
            | SERCOM_I2CS_INTENSET_PREC;

    i2c_sync();
    I2C.CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;
}

static u8_t last_data;

static int write_requested(u8_t addr)
{
    LOG_INF("wreq %x", addr);
    return 0;
}

static int write_received(u8_t val)
{
//    LOG_INF("wrec %x", val);
    last_data = val;
    return 0;
}

static int read_requested(u8_t addr, u8_t *val)
{
    LOG_INF("rreq %x", addr);
    *val = last_data;
    *val = 0x40;
    return 0;
}

static int read_processed(u8_t *val)
{
    static u8_t data = 120;
//    LOG_INF("rpro");
    data = data + 1;
//    *val = data | 0x40;
    *val = data;
    *val = 0xAA;
    return 0;
}

static int stop_received(void)
{
    LOG_INF("stop");
    return 0;
}

static int error_received(void)
{
    // TODO(mbenda): error code
    LOG_INF("err");
    return 0;
}

static inline void i2c_set_ackact(bool ack)
{
#if WORKAROUND
    int key = irq_lock();
    I2C.STATUS.reg = 0;

    if (ack) {
        I2C.CTRLB.reg = 0;
    } else {
        I2C.CTRLB.reg = SERCOM_I2CS_CTRLB_ACKACT;
    }
    irq_unlock(key);
#else /* !WORKAROUND */
    if (ack == true) {
        I2C.CTRLB.reg &= ~SERCOM_I2CS_CTRLB_ACKACT;
    } else {
        I2C.CTRLB.reg |= SERCOM_I2CS_CTRLB_ACKACT;
    }
#endif
}

static inline void i2c_amatch_cmd3()
{
#if WORKAROUND
    if (I2C.INTFLAG.bit.PREC) {
        I2C.INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
    }
    I2C.INTFLAG.reg = SERCOM_I2CS_INTFLAG_AMATCH;
#else /* !WORKAROUND */
    I2C.CTRLB.reg = SERCOM_I2CS_CTRLB_CMD(0x3);
#endif
}

static void isr_i2c_slave_amatch(void)
{
    if (I2C.STATUS.reg & (SERCOM_I2CS_STATUS_BUSERR | SERCOM_I2CS_STATUS_COLL
            | SERCOM_I2CS_STATUS_LOWTOUT)) {
        /* A bus error occurred. */
        error_received();
    }

    u8_t addr = I2C.DATA.reg >> 1;
    int err;

    i2c_state.read_dir = I2C.STATUS.reg & SERCOM_I2CS_STATUS_DIR;
    if (i2c_state.read_dir) {
        /* Master read requested. */
        err = read_requested(addr, &i2c_state.read_data);
        if (!err) {
            i2c_state.read_start = true;
        }

    } else {
        /* Master write requested. */
        err = write_requested(addr);
    }

    /* ACK or NAK the address. */
    i2c_set_ackact(err == 0);
    i2c_amatch_cmd3();

    i2c_set_ackact(true);
}

static void isr_i2c_slave_drdy(void)
{
    int err;

    if (i2c_state.read_dir) {
        /* Master read data. */
        if (i2c_state.read_start) {
            /* Send the first byte. */
            I2C.DATA.reg = i2c_state.read_data;
            i2c_state.read_start = false;
            err = 0;
        } else if ((I2C.STATUS.reg & SERCOM_I2CS_STATUS_RXNACK)) {
            /* Master NAK. */
//            error_received();
            err = EIO; // TODO EOF
        } else {
            /* Ask for a byte to send. */
            u8_t data;
            err = read_processed(&data);
            if (!err) {
                I2C.DATA.reg = data;
            }
        }
    } else {
        /* Master write data. */
        u8_t data = I2C.DATA.reg;
        err = write_received(data);
    }

    if (err) {
        /* Terminate the transfer */
        i2c_set_ackact(false);
        I2C.CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x2);
    } else {
        /* Continue in the transfer. */
        I2C.CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x3);
    }
}

static void isr_i2c_slave(void *arg)
{
    ARG_UNUSED(arg);

    /* Read interrupt flags. */
    u8_t status = I2C.INTFLAG.reg & I2C.INTENSET.reg;

    if (status & SERCOM_I2CS_INTFLAG_AMATCH) {
        isr_i2c_slave_amatch();
    } else if (status & SERCOM_I2CS_INTFLAG_DRDY) {
        isr_i2c_slave_drdy();
    } else if (status & SERCOM_I2CS_INTFLAG_PREC) {
        /* Stop received. */
        stop_received();
        I2C.INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
    }
}

static volatile int active;

int p = 20;

void main(void)
{
    LOG_INF("I²C slave test");

    struct device *port = device_get_binding(LED_PORT);

    gpio_pin_configure(port, LED, GPIO_DIR_OUT);
    gpio_pin_write(port, LED, 1);

    struct device *muxa = device_get_binding("PINMUX_A");
    struct device *muxb = device_get_binding("PINMUX_B");

//    pinmux_pin_set(muxb, 16, PINMUX_FUNC_C);
//    pinmux_pin_set(muxb, 17, PINMUX_FUNC_C);
//    pinmux_pin_set(muxa, 8, PINMUX_FUNC_C);
//    pinmux_pin_set(muxa, 9, PINMUX_FUNC_C);
    pinmux_pin_set(muxa, 22, PINMUX_FUNC_D);
    pinmux_pin_set(muxa, 23, PINMUX_FUNC_D);
    // TODO pull up?

    /* Set up bus clock and GCLK generator. */
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;

//    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM3_GCLK_ID_CORE)
//            | GCLK_CLKCTRL_CLKEN
//            | GCLK_CLKCTRL_GEN(0);
//
//    while (GCLK->STATUS.bit.SYNCBUSY) {
//        /* Synchronize GCLK. */
//    }
//
//    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM3_GCLK_ID_SLOW)
//            | GCLK_CLKCTRL_CLKEN
//            | GCLK_CLKCTRL_GEN(1);
//
//    while (GCLK->STATUS.bit.SYNCBUSY) {
//        /* Synchronize GCLK. */
//    }

//    i2c_reset();

    i2c_init_slave();

    while (true) {
        gpio_pin_write(port, LED, active);
        k_sleep(200 * p / 20);
        gpio_pin_write(port, LED, 1);
        k_sleep(200 * p / 20);
        gpio_pin_write(port, LED, active);
        k_sleep(200 * p / 20);
        gpio_pin_write(port, LED, 1);
        k_sleep(800 * p / 20);
//        LOG_INF("baf %d!", p);
        p--;
        if (p < 0) {
            p = 20;
        }
    }
}
