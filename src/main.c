/*
 * Copyright (c) 2019 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>

#include "zbus.h"

LOG_MODULE_REGISTER(main);

int main(void)
{
    LOG_INF("starting");

    struct device *dev = device_get_binding("zbus0");

    for (;;) {
        int err = zbus_connect(dev);
        if (err != 0) {
            LOG_ERR("cound not connect: %d, retrying...", err);
            k_sleep(1000);
            continue;
        }

        u8_t buf[64];
        int n = zbus_recv(dev, buf, sizeof(buf));
        if (n < 0) {
            LOG_ERR("recv error: %d", n);
            continue;
        }

        if (n == 1 && buf[0] == 0xFF) {
            break;
        }

        LOG_INF("recv %d bytes", n);
        for (int i = 0; i < n; i++) {
            buf[i] = (buf[i] >> 4) | (buf[i] << 4);
        }

        n = zbus_send(dev, buf, n);
        if (n < 0) {
            LOG_ERR("send error: %d", n);
            continue;
        }
    }

    LOG_INF("stopping");

#ifdef CONFIG_ARCH_POSIX
    void posix_exit(int);
    posix_exit(0);
#endif
    return 0;
}
