/*
 * Copyright (c) 2026 Jakub Turek <qb4.dev@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mcp7940.c
 *
 * ESP-IDF driver for mcp7940 real-time clock
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2026 Jakub Turek <qb4.dev@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include "mcp7940.h"

#define I2C_FREQ_HZ 400000

#define RAM_SIZE 64

/* clang-format off */
#define MCP794XX_REG_SECS 0x00
#       define MCP794XX_BIT_ST   (1 << 7)
#define ST_MASK 0x7f
#define SECONDS_MASK 0x7f

#define MCP794XX_REG_MIN    0x01

#define MCP794XX_REG_HOUR   0x02
#       define MCP794XX_BIT_HOUR12 (1 << 6)
#       define MCP794XX_BIT_PM     (1 << 5)
#define HOUR12_MASK 0x1f
#define HOUR24_MASK 0x3f

#define MCP794XX_REG_WDAY   0x03
#       define MCP794XX_BIT_OSCRUN (1 << 5)
#       define MCP794XX_BIT_VBATEN (1 << 3)
#define OSCRUN_MASK 0xdf
#define VBATEN_MASK 0xf7

#define MCP794XX_REG_MDAY   0x04
#define MCP794XX_REG_MONTH  0x05
#define MCP794XX_REG_YEAR   0x06

#define MCP794XX_REG_CONTROL        0x07
#   define MCP794XX_BIT_OUT     (1 << 7)
#   define MCP794XX_BIT_SQWE    (1 << 6)
#   define MCP794XX_BIT_ALM0_EN (1 << 4)
#   define MCP794XX_BIT_ALM1_EN (1 << 5)
#define OUT_MASK  0x7f
#define SQWE_MASK 0xbf
#define ALM0_EN_MASK 0xef
#define ALM1_EN_MASK 0xdf

#define MCP794XX_REG_ALARM0_BASE    0x0a
#define MCP794XX_REG_ALARM0_CTRL    0x0d
#define MCP794XX_REG_ALARM1_BASE    0x11
#define MCP794XX_REG_ALARM1_CTRL    0x14
#   define MCP794XX_BIT_ALMX_IF  (1 << 3)
#   define MCP794XX_BIT_ALMX_C0  (1 << 4)
#   define MCP794XX_BIT_ALMX_C1  (1 << 5)
#   define MCP794XX_BIT_ALMX_C2  (1 << 6)
#   define MCP794XX_BIT_ALMX_POL (1 << 7)
#define ALMX_IF_MASK  0xf7
#define ALMX_C0_MASK  0xef
#define ALMX_C1_MASK  0xdf
#define ALMX_C2_MASK  0xbf
#define ALMX_POL_MASK 0x7f

#define MCP794XX_REG_RAM  0x20
/* clang-format on */

#define CHECK_ARG(ARG)                  \
    do {                                \
        if (!(ARG))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

static esp_err_t update_register(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    CHECK_ARG(dev);

    uint8_t old;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, &old, 1));
    uint8_t   buf = (old & mask) | val;
    esp_err_t res = i2c_dev_write_reg(dev, reg, &buf, 1);
    I2C_DEV_GIVE_MUTEX(dev);

    return res;
}

esp_err_t mcp7940_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = MCP7940_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t mcp7940_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t mcp7940_start(i2c_dev_t *dev, bool start)
{
    CHECK_ARG(dev);

    /* Enable VBATEN bit to allow battery backup */
    ESP_ERROR_CHECK_WITHOUT_ABORT(update_register(dev, MCP794XX_REG_WDAY, VBATEN_MASK, MCP794XX_BIT_VBATEN));
    return update_register(dev, MCP794XX_REG_SECS, ST_MASK, start ? MCP794XX_BIT_ST : 0);
}

esp_err_t mcp7940_is_running(i2c_dev_t *dev, bool *running)
{
    CHECK_ARG(dev && running);

    uint8_t val;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, MCP794XX_REG_SECS, &val, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    *running = val & MCP794XX_BIT_ST ? true : false;

    return ESP_OK;
}

esp_err_t mcp7940_get_time(i2c_dev_t *dev, struct tm *time)
{
    CHECK_ARG(dev && time);

    uint8_t buf[7];

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, MCP794XX_REG_SECS, buf, 7));
    I2C_DEV_GIVE_MUTEX(dev);

    time->tm_sec = bcd2dec(buf[0] & SECONDS_MASK);
    time->tm_min = bcd2dec(buf[1]);
    if (buf[2] & MCP794XX_BIT_HOUR12)
    {
        // RTC in 12-hour mode
        time->tm_hour = bcd2dec(buf[2] & HOUR12_MASK) - 1;
        if (buf[2] & MCP794XX_BIT_PM)
            time->tm_hour += 12;
    }
    else
        time->tm_hour = bcd2dec(buf[2] & HOUR24_MASK);
    time->tm_wday = bcd2dec(buf[3]) - 1;
    time->tm_mday = bcd2dec(buf[4]);
    time->tm_mon = bcd2dec(buf[5]) - 1;
    time->tm_year = bcd2dec(buf[6]) + 100;

    return ESP_OK;
}

esp_err_t mcp7940_set_time(i2c_dev_t *dev, const struct tm *time)
{
    CHECK_ARG(dev && time);

    uint8_t buf[7] = { dec2bcd(time->tm_sec),       dec2bcd(time->tm_min),  dec2bcd(time->tm_hour),
                       dec2bcd(time->tm_wday + 1),  dec2bcd(time->tm_mday), dec2bcd(time->tm_mon + 1),
                       dec2bcd(time->tm_year - 100)
                     };

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, MCP794XX_REG_SECS, buf, sizeof(buf)));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mcp7940_read_ram(i2c_dev_t *dev, uint8_t offset, uint8_t *buf, uint8_t len)
{
    CHECK_ARG(dev && buf);

    if (offset + len > RAM_SIZE)
        return ESP_ERR_NO_MEM;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, MCP794XX_REG_RAM + offset, buf, len));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mcp7940_write_ram(i2c_dev_t *dev, uint8_t offset, uint8_t *buf, uint8_t len)
{
    CHECK_ARG(dev && buf);

    if (offset + len > RAM_SIZE)
        return ESP_ERR_NO_MEM;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, MCP794XX_REG_RAM + offset, buf, len));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
