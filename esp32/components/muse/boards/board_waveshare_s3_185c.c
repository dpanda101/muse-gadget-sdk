/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Waveshare ESP32-S3-Touch-LCD-1.85C: round 360 px ST77916 QSPI panel with
 * CST816 touch, ES8311 speaker DAC + ES7210 mic ADC, TCA9554 I/O expander (display
 * and touch resets live on its EXIO2/EXIO1 pins), no PMU. BOOT (GPIO0) is the
 * talk button; holding a finger on the face tile also talks.
 *
 * CONFIG_MUSE_WAVESHARE_S3_185_PCM5101 builds the plain 1.85 (no C): the same
 * panel, touch and expander, but a PCM5101A I2S DAC for the speaker and a MEMS
 * mic on a second I2S bus, wired like the Waveshare 1.46 (board_waveshare_s3_146.c).
 * There GPIO15 is the mic's clock, not an amplifier enable.
 */
#include <math.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "es8311_codec.h"
#include "es7210_adc.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_ui.h"
#include "muse_lcd_bands.h"
#include "muse_mem.h"

#include "esp_lcd_st77916.h"
#include "esp_lcd_touch_cst816s.h"
#include "waveshare_s3_185c/st77916_init_185c.h"

static const char *TAG = "board";

/* Pins (board schematic / vendor demo). */
#define LCD_SCLK GPIO_NUM_40
#define LCD_D0 GPIO_NUM_46
#define LCD_D1 GPIO_NUM_45
#define LCD_D2 GPIO_NUM_42
#define LCD_D3 GPIO_NUM_41
#define LCD_CS GPIO_NUM_21
#define LCD_TE GPIO_NUM_18   /* torn-effect output; read by the panel driver */
#define LCD_BL GPIO_NUM_5
#define LCD_RES 360

#define I2C_SDA GPIO_NUM_11
#define I2C_SCL GPIO_NUM_10
#define TOUCH_INT GPIO_NUM_4
#if CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
/* The plain 1.85's CST816 doesn't answer on the expander's bus: it is looked
 * for there first, then on a bus of its own on these pins. */
#define TP_ALT_SDA GPIO_NUM_1
#define TP_ALT_SCL GPIO_NUM_3
#define CST816_ADDR 0x15
#endif

#define TCA9554_ADDR 0x20
#define TCA9554_INPUT_REG 0x00
#define TCA9554_OUTPUT_REG 0x01
#define TCA9554_CONFIG_REG 0x03
#define EXIO_TOUCH_RST_BIT BIT(0)   /* EXIO1, to the CST816 */
#define EXIO_LCD_RST_BIT BIT(1)     /* EXIO2, to the ST77916 */

#if CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
/* Speaker: PCM5101A on its own I2S bus, no MCLK (its PLL runs from BCLK). */
#define SPK_BCLK GPIO_NUM_48
#define SPK_WS GPIO_NUM_38
#define SPK_DOUT GPIO_NUM_47
/* Mic: MEMS I2S mic on a second bus. */
#define MIC_BCLK GPIO_NUM_15
#define MIC_WS GPIO_NUM_2
#define MIC_DIN GPIO_NUM_39
#define MIC_GAIN_OFFSET_DB 0
#else
#define I2S_MCLK GPIO_NUM_2
#define I2S_BCLK GPIO_NUM_48
#define I2S_WS GPIO_NUM_38
#define I2S_DOUT GPIO_NUM_47   /* to the ES8311 DAC */
#define I2S_DIN GPIO_NUM_39    /* from the mic */
#define PA_EN GPIO_NUM_15
#endif

#define BATT_ADC ADC_CHANNEL_7  /* GPIO8 */

#define DRAW_BUF_LINES 72      /* five bands to the screen (muse_lcd_bands.h) */
#define LCD_CHUNK_BYTES (LCD_RES * 8 * 2)

static i2c_master_bus_handle_t s_i2c;
static i2c_master_dev_handle_t s_tca;
static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static esp_lcd_touch_handle_t s_tp;
static lv_indev_t *s_touch_indev;
static muse_gpio_button_t s_boot;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali = NULL;

/* The ST77916 needs even-aligned update windows over the QSPI command path. */
static void round_area(lv_event_t *e)
{
    lv_area_t *a = lv_event_get_param(e);
    a->x1 &= ~1;
    a->y1 &= ~1;
    a->x2 |= 1;
    a->y2 |= 1;
}

/* The TCA9554's reset lines are driven by the board, not by the panel and
 * touch drivers (they'd use plain GPIOs). */
static esp_err_t tca_write(uint8_t reg, uint8_t val)
{
    return i2c_master_transmit(s_tca, (uint8_t[]){ reg, val }, 2, -1);
}

static esp_err_t tca_reset_pulse(uint8_t bit)
{
    uint8_t out;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_tca, (uint8_t[]){ TCA9554_OUTPUT_REG }, 1, &out, 1, -1),
                        TAG, "tca read");
    ESP_RETURN_ON_ERROR(tca_write(TCA9554_OUTPUT_REG, out & ~bit), TAG, "tca low");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(tca_write(TCA9554_OUTPUT_REG, out | bit), TAG, "tca high");
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

static esp_err_t init(void)
{
    const i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_cfg, &s_i2c), TAG, "i2c");

    const i2c_device_config_t tca_cfg = {
        .device_address = TCA9554_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c, &tca_cfg, &s_tca), TAG, "tca9554");
#if CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
    /* As on the 1.46: only the two resets and EXIO3 (the SD card's CS) are
     * driven, high; the rest stay inputs, so nothing here drives against the
     * IMU's interrupt lines or holds the speaker amplifier in mute. */
    ESP_RETURN_ON_ERROR(tca_write(TCA9554_OUTPUT_REG, 0xFF), TAG, "tca outputs");
    ESP_RETURN_ON_ERROR(tca_write(TCA9554_CONFIG_REG,
                                  (uint8_t)~(EXIO_LCD_RST_BIT | EXIO_TOUCH_RST_BIT | BIT(2))),
                        TAG, "tca config");
#else
    /* All pins output; leave every line high, then pulse the reset lines. */
    ESP_RETURN_ON_ERROR(tca_write(TCA9554_CONFIG_REG, 0x00), TAG, "tca config");
    ESP_RETURN_ON_ERROR(tca_write(TCA9554_OUTPUT_REG, 0xFF), TAG, "tca outputs");
#endif
    ESP_RETURN_ON_ERROR(tca_reset_pulse(EXIO_LCD_RST_BIT), TAG, "lcd reset");
    ESP_RETURN_ON_ERROR(tca_reset_pulse(EXIO_TOUCH_RST_BIT), TAG, "touch reset");

    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_boot, GPIO_NUM_0), TAG, "boot button");
    /* BOOT is held at flashing time; don't count a held button as a press. */
    s_boot.pressed = gpio_get_level(GPIO_NUM_0) == 0;

#if !CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
    const gpio_config_t pa = {
        .pin_bit_mask = 1ULL << PA_EN,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pa), TAG, "pa en");
    ESP_RETURN_ON_ERROR(gpio_set_level(PA_EN, 1), TAG, "pa on");
#endif

    const adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_cfg, &s_adc), TAG, "adc");
    const adc_oneshot_chan_cfg_t ch_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, BATT_ADC, &ch_cfg), TAG, "adc channel");
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = BATT_ADC,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) != ESP_OK) {
        ESP_LOGW(TAG, "adc calibration unavailable; battery mV is uncalibrated");
    }
#endif
    return ESP_OK;
}

/* muse_lcd_bands_register takes over the panel IO's color-done callback and,
 * with the bands, needs the bus set up on the same core as the UI task. */
static lv_display_t *display_start(lv_indev_t **touch)
{
    const ledc_timer_config_t bl_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t bl_ch = {
        .gpio_num = LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    if (ledc_timer_config(&bl_timer) != ESP_OK || ledc_channel_config(&bl_ch) != ESP_OK) {
        ESP_LOGE(TAG, "backlight pwm");
        return NULL;
    }

    const spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK,
        .data0_io_num = LCD_D0,
        .data1_io_num = LCD_D1,
        .data2_io_num = LCD_D2,
        .data3_io_num = LCD_D3,
        .max_transfer_sz = LCD_CHUNK_BYTES,
    };
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        ESP_LOGE(TAG, "spi bus");
        return NULL;
    }
    const esp_lcd_panel_io_spi_config_t io_cfg = ST77916_PANEL_IO_QSPI_CONFIG(LCD_CS, NULL, NULL);
    if (esp_lcd_new_panel_io_spi(SPI2_HOST, &io_cfg, &s_io) != ESP_OK) {
        ESP_LOGE(TAG, "panel io");
        return NULL;
    }
    st77916_vendor_config_t vendor_cfg = {
        .init_cmds = ST77916_INIT_185C_CMDS,
        .init_cmds_size = ST77916_INIT_185C_SIZE,
        .flags = { .use_qspi_interface = 1 },
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,   /* driven through the TCA9554 in init() */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
    };
    if (esp_lcd_new_panel_st77916(s_io, &panel_cfg, &s_panel) != ESP_OK) {
        ESP_LOGE(TAG, "st77916");
        return NULL;
    }
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_disp_on_off(s_panel, true);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = s_io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_RES,
            .ver_res = LCD_RES,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = muse_lcd_bands_register(disp_cfg, DRAW_BUF_LINES, LCD_CHUNK_BYTES);
    if (!disp) {
        return NULL;
    }
    lv_display_add_event_cb(disp, round_area, LV_EVENT_INVALIDATE_AREA, NULL);

    i2c_master_bus_handle_t tp_bus = s_i2c;
#if CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
    /* Find the touch controller; without one, run on BOOT alone rather than
     * fail the display (which would restart the board in a loop). */
    if (i2c_master_probe(s_i2c, CST816_ADDR, 100) != ESP_OK) {
        static i2c_master_bus_handle_t s_tp_i2c;
        const i2c_master_bus_config_t tp_i2c_cfg = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = TP_ALT_SDA,
            .scl_io_num = TP_ALT_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        if (!s_tp_i2c && i2c_new_master_bus(&tp_i2c_cfg, &s_tp_i2c) != ESP_OK) {
            s_tp_i2c = NULL;
        }
        if (s_tp_i2c && i2c_master_probe(s_tp_i2c, CST816_ADDR, 100) == ESP_OK) {
            ESP_LOGI(TAG, "touch: CST816 on its own bus (SDA %d, SCL %d)", TP_ALT_SDA, TP_ALT_SCL);
            tp_bus = s_tp_i2c;
        } else {
            ESP_LOGW(TAG, "touch: no CST816 found; running without touch (BOOT still works)");
            *touch = NULL;
            if (esp_lv_adapter_start() != ESP_OK) {
                return NULL;
            }
            return disp;
        }
    } else {
        ESP_LOGI(TAG, "touch: CST816 on the expander's bus");
    }
#endif
    esp_lcd_panel_io_handle_t tp_io;
    const esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    if (esp_lcd_new_panel_io_i2c(tp_bus, &tp_io_cfg, &tp_io) != ESP_OK) {
        ESP_LOGE(TAG, "touch io");
        return NULL;
    }
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_RES,
        .y_max = LCD_RES,
        .rst_gpio_num = -1,   /* driven through the TCA9554 in init() */
        .int_gpio_num = TOUCH_INT,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    if (esp_lcd_touch_new_i2c_cst816s(tp_io, &tp_cfg, &s_tp) != ESP_OK) {
        ESP_LOGE(TAG, "cst816");
        return NULL;
    }
    const esp_lv_adapter_touch_config_t tp_lvgl_cfg = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, s_tp);
    *touch = esp_lv_adapter_register_touch(&tp_lvgl_cfg);
    if (!*touch || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    s_touch_indev = *touch;
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

static void set_brightness(int pct)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pct * 1023 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void panel_sleep(bool sleep)
{
    esp_lcd_panel_disp_sleep(s_panel, sleep);   /* SLPIN/SLPOUT; GRAM is kept */
    vTaskDelay(pdMS_TO_TICKS(120));             /* settle before the next command */
}

static void display_pause(bool pause)
{
    if (pause) {
        esp_lv_adapter_pause(-1);
    } else {
        esp_lv_adapter_resume();
    }
}

#if CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
/* Plain 1.85: no codec control anywhere, as on the 1.46. The PCM5101A takes
 * I2S as is, esp_codec_dev adds the volume in software, and the mic gain is
 * applied here. Both channels run all the time; idle, the speaker sends zeros
 * (auto_clear). */
static i2s_chan_handle_t s_tx, s_rx;
static int32_t s_mic_dc[2];
static int s_mic_gain_q8 = 256;

static int data_enable(const audio_codec_data_if_t *h, esp_codec_dev_type_t type, bool on)
{
    (void)h;
    (void)type;
    (void)on;
    return ESP_CODEC_DEV_OK;
}

static int spk_write(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    (void)h;
    size_t wrote;
    return i2s_channel_write(s_tx, data, size, &wrote, portMAX_DELAY) == ESP_OK ? ESP_CODEC_DEV_OK
                                                                                : ESP_CODEC_DEV_WRITE_FAIL;
}

/* A MEMS mic's slot has a small DC offset, which the gain would turn into
 * clipping: each slot's zero level is tracked and taken off first. */
static int mic_read(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    (void)h;
    size_t got;
    if (i2s_channel_read(s_rx, data, size, &got, pdMS_TO_TICKS(1000)) != ESP_OK || got != (size_t)size) {
        return ESP_CODEC_DEV_READ_FAIL;
    }
    int16_t *s = (int16_t *)data;
    for (int i = 0; i < size / 2; i++) {
        int32_t *dc = &s_mic_dc[i & 1];
        *dc += (s[i] * 256 - *dc) >> 8;
        int v = (s[i] - (*dc >> 8)) * s_mic_gain_q8 >> 8;
        s[i] = v > INT16_MAX ? INT16_MAX : v < INT16_MIN ? INT16_MIN : v;
    }
    return ESP_CODEC_DEV_OK;
}

static void set_mic_gain(esp_codec_dev_handle_t mic, int db)
{
    (void)mic;
    s_mic_gain_q8 = (int)(256.0f * powf(10.0f, (db - MIC_GAIN_OFFSET_DB) / 20.0f));
}

/* Two I2S buses: the PCM5101A on I2S1 with plain 16-bit stereo, the MEMS mic
 * on I2S0 with the 32-bit slots it needs (each sample is the slot's top
 * 16 bits). */
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    i2s_chan_config_t tx_chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    tx_chan.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&tx_chan, &s_tx, NULL), TAG, "i2s tx chan");
    const i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SPK_BCLK,
            .ws = SPK_WS,
            .dout = SPK_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &tx_cfg), TAG, "i2s tx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "i2s tx on");

    i2s_chan_config_t rx_chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    /* The mic's 32-bit slots double each DMA frame: half the frames keep its
     * buffers the size the 1.85C's shared bus uses, in internal RAM. */
    rx_chan.dma_frame_num /= 2;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&rx_chan, NULL, &s_rx), TAG, "i2s rx chan");
    i2s_std_config_t rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_BCLK,
            .ws = MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = MIC_DIN,
        },
    };
    rx_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    rx_cfg.slot_cfg.ws_width = 32;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &rx_cfg), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx), TAG, "i2s rx on");

    static const audio_codec_data_if_t spk_if = { .enable = data_enable, .write = spk_write };
    static const audio_codec_data_if_t mic_if = { .enable = data_enable, .read = mic_read };
    esp_codec_dev_cfg_t out_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_OUT, .data_if = &spk_if };
    esp_codec_dev_cfg_t in_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_IN, .data_if = &mic_if };
    *spk = esp_codec_dev_new(&out_cfg);
    *mic = esp_codec_dev_new(&in_cfg);
    ESP_RETURN_ON_FALSE(*spk && *mic, ESP_FAIL, TAG, "codec devs");
    ESP_LOGI(TAG, "audio: PCM5101A (spk, I2S1) + MEMS mic (I2S0)");
    return ESP_OK;
}
#else
/*
 * V2 hardware (confirmed by I2C scan 2026-10-02): ES8311 @ 0x18 for speaker
 * (DAC) + ES7210 @ 0x40 for mic (ADC). I2S_NUM_1, MCLK=2, BCLK=48, WS=38,
 * DOUT=47 (to ES8311), DIN=39 (from ES7210). Follows Waveshare vendor demo:
 * separate I2S data interfaces for play/record.
 * NOTE: audio_codec_new_i2c_ctrl expects the 8-bit shifted address, so use
 * ES7210_CODEC_DEFAULT_ADDR (0x80), NOT literal 0x40.
 */
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    i2s_chan_handle_t tx, rx;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx, &rx), TAG, "i2s chan");
    const i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_DOUT,
            .din = I2S_DIN,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &std_cfg), TAG, "i2s tx");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &std_cfg), TAG, "i2s rx");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "i2s tx on");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "i2s rx on");

    const audio_codec_data_if_t *spk_data = audio_codec_new_i2s_data(&(audio_codec_i2s_cfg_t){
        .port = I2S_NUM_1, .rx_handle = NULL, .tx_handle = tx });
    ESP_RETURN_ON_FALSE(spk_data, ESP_ERR_NO_MEM, TAG, "spk data if");
    const audio_codec_ctrl_if_t *spk_ctrl = audio_codec_new_i2c_ctrl(&(audio_codec_i2c_cfg_t){
        .port = I2C_NUM_0, .addr = ES8311_CODEC_DEFAULT_ADDR, .bus_handle = s_i2c });
    const audio_codec_gpio_if_t *spk_gpio = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(spk_ctrl && spk_gpio, ESP_ERR_NO_MEM, TAG, "spk ctrl");
    const audio_codec_if_t *spk_codec = es8311_codec_new(&(es8311_codec_cfg_t){
        .ctrl_if = spk_ctrl,
        .gpio_if = spk_gpio,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = PA_EN,
        .use_mclk = false,
    });
    ESP_RETURN_ON_FALSE(spk_codec, ESP_FAIL, TAG, "ES8311 not responding");
    *spk = esp_codec_dev_new(&(esp_codec_dev_cfg_t){
        .dev_type = ESP_CODEC_DEV_TYPE_OUT, .codec_if = spk_codec, .data_if = spk_data });
    ESP_RETURN_ON_FALSE(*spk, ESP_FAIL, TAG, "spk dev");

    const audio_codec_data_if_t *mic_data = audio_codec_new_i2s_data(&(audio_codec_i2s_cfg_t){
        .port = I2S_NUM_1, .rx_handle = rx, .tx_handle = NULL });
    ESP_RETURN_ON_FALSE(mic_data, ESP_ERR_NO_MEM, TAG, "mic data if");
    const audio_codec_ctrl_if_t *mic_ctrl = audio_codec_new_i2c_ctrl(&(audio_codec_i2c_cfg_t){
        .port = I2C_NUM_0, .addr = ES7210_CODEC_DEFAULT_ADDR, .bus_handle = s_i2c });
    ESP_RETURN_ON_FALSE(mic_ctrl, ESP_ERR_NO_MEM, TAG, "mic ctrl");
    const audio_codec_if_t *mic_codec = es7210_codec_new(&(es7210_codec_cfg_t){
        .ctrl_if = mic_ctrl,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2,
    });
    ESP_RETURN_ON_FALSE(mic_codec, ESP_FAIL, TAG, "ES7210 not responding");
    *mic = esp_codec_dev_new(&(esp_codec_dev_cfg_t){
        .dev_type = ESP_CODEC_DEV_TYPE_IN, .codec_if = mic_codec, .data_if = mic_data });
    ESP_RETURN_ON_FALSE(*mic, ESP_FAIL, TAG, "mic dev");

    ESP_LOGI(TAG, "audio: ES8311 (spk) + ES7210 (mic) on I2S1");
    return ESP_OK;
}
#endif

/* Hold-to-talk on the face tile: a finger held on the avatar for
 * TOUCH_HOLD_MS acts as the talk button. Taps and touches on other tiles
 * (e.g. settings) never trigger talk. Touch state comes from LVGL's input
 * device, which owns the CST816 driver handle; reading the driver here
 * would race LVGL's own reads. */
#define TOUCH_HOLD_MS 500

static unsigned poll_buttons(void)
{
    unsigned ev = muse_gpio_button_poll(&s_boot);

    static bool tp_down = false;          /* finger currently on screen */
    static bool tp_talking = false;       /* hold threshold reached */
    static TickType_t tp_press_tick = 0;  /* when the current press began */
    static bool tp_on_face = false;       /* press started on the face tile */

    bool pressed = false;
    if (s_touch_indev) {
        pressed = (lv_indev_get_state(s_touch_indev) == LV_INDEV_STATE_PRESSED);
    }

    if (pressed && !tp_down) {
        /* New press: only a hold that starts on the face tile can talk. */
        tp_down = true;
        tp_on_face = muse_ui_face_active();
        tp_press_tick = xTaskGetTickCount();
        tp_talking = false;
    } else if (!pressed && tp_down) {
        /* Release: end the talk if one started. */
        if (tp_talking) {
            ev |= MUSE_BTN_TALK_RELEASE;
        }
        tp_down = false;
        tp_talking = false;
    } else if (pressed && tp_down && tp_on_face && !tp_talking) {
        /* Still held on the face: start talking once the hold threshold passes. */
        if ((xTaskGetTickCount() - tp_press_tick) >= pdMS_TO_TICKS(TOUCH_HOLD_MS)) {
            tp_talking = true;
            ev |= MUSE_BTN_TALK_PRESS;
        }
    }
    return ev;
}

static void wait_buttons(int timeout_ms)
{
    muse_gpio_buttons_wait((muse_gpio_button_t *const[]){ &s_boot }, 1, timeout_ms);
}

/*
 * Battery voltage on GPIO8 (ADC1 CH7) through the board's divider. The
 * Waveshare demo reads it uncalibrated; curve-fitting calibration is used
 * when the eFuse bits are present.
 */
static esp_err_t read_power(muse_power_t *out)
{
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        int v;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, BATT_ADC, &v), TAG, "adc read");
        sum += v;
    }
    int raw = sum / 8;
    int mv;
    if (s_cali) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali, raw, &mv), TAG, "adc cali");
    } else {
        mv = raw * 3300 / 4096;   /* 12 dB attenuation, full scale ~3.3 V */
    }
    /* GPIO8 sees the battery through a 200K/100K divider:
     * V_bat = V_adc * (200 + 100) / 100 = V_adc * 3.
     * Matches the proven xiaozhi/Jarvis AdcBatteryMonitor (200000, 100000). */
    out->battery_mv = mv * 3;
    /* Rough Li-ion window: 3.3 V empty, 4.2 V full. Far below empty, no
     * cell is fitted: report no battery rather than 0%. */
    int pct = (out->battery_mv - 3300) * 100 / (4200 - 3300);
    out->battery_pct = out->battery_mv < 2500 ? -1 : pct < 0 ? 0 : pct > 100 ? 100 : pct;
    out->charging = false;   /* no charge-state line on this board */
    /* No VBUS sense either: a USB host on the chip's own port is the closest. */
    out->usb = usb_serial_jtag_is_connected();
    return ESP_OK;
}

static esp_err_t power_off(void)
{
    set_brightness(0);
    esp_lcd_panel_disp_on_off(s_panel, false);
    esp_lcd_panel_disp_sleep(s_panel, true);
    /* No PMU on this board; deep sleep is the closest thing to off. BOOT
     * wakes it. */
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_deep_sleep_start();
    return ESP_FAIL;
}

static const muse_board_t s_board = {
#if CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
    .name = "Waveshare ESP32-S3-Touch-LCD-1.85",
#else
    .name = "Waveshare ESP32-S3-Touch-LCD-1.85C",
#endif
    .width = LCD_RES,
    .height = LCD_RES,
    .round = true,
    .touch = true,
    .diagonal_in = 1.85f,
    .talk_button = "BOOT",
    /* BOOT is on the side edge, toward the top of the puck. */
    .talk_hint = { LV_ALIGN_TOP_LEFT, 20, 16 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .display_pause = display_pause,
    .audio_init = audio_init,
#if CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
    .mic_slot = 1,   /* one MEMS mic, on the right slot (as on the 1.46) */
#else
    .mic_slot = 0,   /* one mic */
#endif
#if CONFIG_MUSE_WAVESHARE_S3_185_PCM5101
    .set_mic_gain = set_mic_gain,   /* software gain in mic_read() */
#else
    .set_mic_gain = NULL,   /* default: esp_codec_dev_set_in_gain */
#endif
    .poll_buttons = poll_buttons,
    .wait_buttons = wait_buttons,
    .read_power = read_power,
    .power_off = power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
