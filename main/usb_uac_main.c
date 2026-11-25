/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "usb_device_uac.h"

static const char *TAG = "uac_inmp441_stereo";

#define I2S_BCLK_PIN    GPIO_NUM_4
#define I2S_WS_PIN      GPIO_NUM_5
#define I2S_DIN_PIN     GPIO_NUM_6

static i2s_chan_handle_t rx_chan = NULL;

static esp_err_t init_i2s_stereo_mic(uint32_t sample_rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_LOGI(TAG, "I2S Stereo Mic initialized at %" PRIu32 " Hz", sample_rate);
    return ESP_OK;
}

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    return ESP_OK;
}

static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    if (rx_chan == NULL || buf == NULL || len == 0) {
        if (bytes_read) *bytes_read = 0;
        return ESP_ERR_INVALID_STATE;
    }

    size_t out_samples_needed = len / sizeof(int16_t);
    size_t i2s_bytes_to_read = out_samples_needed * sizeof(int32_t);

    static int32_t i2s_raw_buf[512];
    size_t max_i2s_bytes = sizeof(i2s_raw_buf);
    if (i2s_bytes_to_read > max_i2s_bytes) {
        i2s_bytes_to_read = max_i2s_bytes;
        out_samples_needed = i2s_bytes_to_read / sizeof(int32_t);
    }

    size_t actual_i2s_read = 0;
    esp_err_t ret = i2s_channel_read(rx_chan, i2s_raw_buf, i2s_bytes_to_read, &actual_i2s_read, pdMS_TO_TICKS(20));
    if (ret != ESP_OK) {
        memset(buf, 0, len);
        if (bytes_read) *bytes_read = len;
        return ESP_OK;
    }

    size_t samples_read = actual_i2s_read / sizeof(int32_t);
    int16_t *pcm16_out = (int16_t *)buf;

    for (size_t i = 0; i < samples_read; i++) {
        pcm16_out[i] = (int16_t)(i2s_raw_buf[i] >> 16);
    }

    if (bytes_read) {
        *bytes_read = samples_read * sizeof(int16_t);
    }
    return ESP_OK;
}

static void uac_device_set_mute_cb(uint32_t mute, void *arg)
{
    ESP_LOGI(TAG, "uac_device_set_mute_cb: %" PRIu32, mute);
}

static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
    ESP_LOGI(TAG, "uac_device_set_volume_cb: %" PRIu32, volume);
}

void app_main(void)
{
    uint32_t sample_rate = 48000;
#ifdef CONFIG_UAC_SAMPLE_RATE
    sample_rate = CONFIG_UAC_SAMPLE_RATE;
#endif

    ESP_ERROR_CHECK(init_i2s_stereo_mic(sample_rate));

    uac_device_config_t config = {
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .cb_ctx = NULL,
    };

    ESP_ERROR_CHECK(uac_device_init(&config));
    ESP_LOGI(TAG, "ESP32-S3 Stereo USB UAC Mic started");
}
