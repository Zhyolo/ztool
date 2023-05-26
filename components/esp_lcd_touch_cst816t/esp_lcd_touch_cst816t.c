/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"

static const char *TAG = "CST816T";

/* CST816T registers */
#define ESP_LCD_TOUCH_CST816T_TOUCH_NUM_REG     (0x02)
#define ESP_LCD_TOUCH_CST816T_READ_XY_REG       (0x03)
#define ESP_LCD_TOUCH_CST816T_PRODUCT_ID_REG    (0xA7)
#define ESP_LCD_TOUCH_CST816T_VER_REG           (0xA9)
#define ESP_LCD_TOUCH_CST816T_DIS_AUTOSLEEP     (0xFE)

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t esp_lcd_touch_cst816t_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_cst816t_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_cst816t_del(esp_lcd_touch_handle_t tp);

/* I2C read/write */
static esp_err_t touch_cst816t_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t touch_cst816t_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data);

/* CST816T reset */
static esp_err_t touch_cst816t_reset(esp_lcd_touch_handle_t tp);
/* Read status and config register */
static esp_err_t touch_cst816t_read_cfg(esp_lcd_touch_handle_t tp);
/* Disable auto sleep */
static esp_err_t touch_cst816t_disable_auto_sleep(esp_lcd_touch_handle_t tp);

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_new_i2c_cst816t(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    assert(io != NULL);
    assert(config != NULL);
    assert(out_touch != NULL);

    /* Prepare main structure */
    esp_lcd_touch_handle_t esp_lcd_touch_cst816t = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_cst816t, ESP_ERR_NO_MEM, err, TAG, "no mem for CST816T controller");

    /* Communication interface */
    esp_lcd_touch_cst816t->io = io;

    /* Only supported callbacks are set */
    esp_lcd_touch_cst816t->read_data = esp_lcd_touch_cst816t_read_data;
    esp_lcd_touch_cst816t->get_xy = esp_lcd_touch_cst816t_get_xy;
    esp_lcd_touch_cst816t->del = esp_lcd_touch_cst816t_del;

    /* Mutex */
    esp_lcd_touch_cst816t->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_cst816t->config, config, sizeof(esp_lcd_touch_config_t));

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_cst816t->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_cst816t->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Prepare pin for touch controller reset */
    if (esp_lcd_touch_cst816t->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_cst816t->config.rst_gpio_num)
        };
        ret = gpio_config(&rst_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Reset controller */
    ret = touch_cst816t_reset(esp_lcd_touch_cst816t);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "CST816T reset failed");

    ret = touch_cst816t_disable_auto_sleep(esp_lcd_touch_cst816t);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "CST816T disable auto sleep failed");

    /* Read status and config info */
    ret = touch_cst816t_read_cfg(esp_lcd_touch_cst816t);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "CST816T init failed");


err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller CST816T initialization failed!", ret);
        if (esp_lcd_touch_cst816t) {
            esp_lcd_touch_cst816t_del(esp_lcd_touch_cst816t);
        }
    }

    *out_touch = esp_lcd_touch_cst816t;

    return ret;
}

static esp_err_t esp_lcd_touch_cst816t_read_data(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t buf[4];
    uint8_t touch_cnt = 0;

    assert(tp != NULL);

    err = touch_cst816t_i2c_read(tp, ESP_LCD_TOUCH_CST816T_TOUCH_NUM_REG, &touch_cnt, 1);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    /* Any touch data? */
    if (0 == touch_cnt) {
    } else {
        /* Read all points */
        err = touch_cst816t_i2c_read(tp, ESP_LCD_TOUCH_CST816T_READ_XY_REG, buf, 4);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

        portENTER_CRITICAL(&tp->data.lock);

        /* Number of touched points */
        tp->data.points = touch_cnt;

        tp->data.coords[0].x = ((buf[0] & 0x0f) << 8) | buf[1];
        tp->data.coords[0].y = ((buf[2] & 0x0f) << 8) | buf[3];

        portEXIT_CRITICAL(&tp->data.lock);
    }

    return ESP_OK;
}

static bool esp_lcd_touch_cst816t_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(point_num != NULL);
    assert(max_point_num > 0);

    portENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    /* Invalidate */
    tp->data.points = 0;

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t esp_lcd_touch_cst816t_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }

    /* Reset GPIO pin settings */
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}

/*******************************************************************************
* Private API function
*******************************************************************************/

/* Reset controller */
static esp_err_t touch_cst816t_reset(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

static esp_err_t touch_cst816t_read_cfg(esp_lcd_touch_handle_t tp)
{
    uint8_t buf[2];

    assert(tp != NULL);

    ESP_RETURN_ON_ERROR(touch_cst816t_i2c_read(tp, ESP_LCD_TOUCH_CST816T_PRODUCT_ID_REG, (uint8_t *)&buf[0], 1), TAG, "CST816T read error!");
    ESP_RETURN_ON_ERROR(touch_cst816t_i2c_read(tp, ESP_LCD_TOUCH_CST816T_VER_REG, (uint8_t *)&buf[1], 1), TAG, "CST816T read error!");

    ESP_LOGI(TAG, "TouchPad_ID:0x%02x", buf[0]);
    ESP_LOGI(TAG, "TouchPad_Config_Version:0x%02x", buf[1]);

    return ESP_OK;
}

static esp_err_t touch_cst816t_disable_auto_sleep(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    ESP_RETURN_ON_ERROR(touch_cst816t_i2c_write(tp, ESP_LCD_TOUCH_CST816T_DIS_AUTOSLEEP, 0XFF), TAG, "CST816T disable auto sleep error!");
    return ESP_OK;
}

static esp_err_t touch_cst816t_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len)
{
    assert(tp != NULL);
    assert(data != NULL);

    /* Read data */
    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}

static esp_err_t touch_cst816t_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data)
{
    assert(tp != NULL);

    // *INDENT-OFF*
    /* Write data */
    return esp_lcd_panel_io_tx_param(tp->io, reg, (uint8_t[]){data}, 1);
    // *INDENT-ON*
}
