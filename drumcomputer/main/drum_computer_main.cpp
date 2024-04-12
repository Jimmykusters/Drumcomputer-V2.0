/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// https://github.com/Ai-Thinker-Open/ESP32-A1S-AudioKit/issues/26

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "board.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_flash.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"

#include "esp_log.h"

#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"

#include "wav_decoder.h"

#include "esp_peripherals.h"
#include "periph_sdcard.h"

#include "sdcard_list.h"
#include "sdcard_scan.h"

// Eigen libraries
#include "drumpad.hpp"

int drumpadSamples[4] = {1, 1, 1, 1};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// https://github.com/itead/ITEADLIB_Arduino_Nextion/blob/master/NexHardware.cpp

#define NEXTION_MESSAGE_LENGHT 11

#define NEXTION_NUMBER_OF_PADS 4

#define NEXTION_PAD_1 "e\0\0ÿÿÿ"
#define NEXTION_PAD_2 "e\0\0ÿÿÿ"
#define NEXTION_PAD_3 "e\0\0ÿÿÿ"
#define NEXTION_PAD_4 "e\0\0ÿÿÿ"

const char padMessages[NEXTION_NUMBER_OF_PADS][NEXTION_MESSAGE_LENGHT] {
    NEXTION_PAD_1,
    NEXTION_PAD_2,
    NEXTION_PAD_3,
    NEXTION_PAD_4,
};

#define NEXTION_NUMBER_OF_SAMPLES 12

#define NEXTION_SAMPLE_1  "e\0ÿÿÿ"
#define NEXTION_SAMPLE_2  "e\0ÿÿÿ"
#define NEXTION_SAMPLE_3  "e\0ÿÿÿ"
#define NEXTION_SAMPLE_4  "e\0ÿÿÿ"
#define NEXTION_SAMPLE_5  "e\0ÿÿÿ" 
#define NEXTION_SAMPLE_6  "e\0ÿÿÿ" 
#define NEXTION_SAMPLE_7  "e\a\0ÿÿÿ"
#define NEXTION_SAMPLE_8  "e\0ÿÿÿ" 
#define NEXTION_SAMPLE_9  "e\t\0ÿÿÿ"
#define NEXTION_SAMPLE_10 "e\n\0ÿÿÿ"
#define NEXTION_SAMPLE_11 "e\v\0ÿÿÿ"
#define NEXTION_SAMPLE_12 "e\n\0ÿÿÿ"

const char padSampleMsg[NEXTION_NUMBER_OF_SAMPLES][NEXTION_MESSAGE_LENGHT] = {
    NEXTION_SAMPLE_1,
    NEXTION_SAMPLE_2,
    NEXTION_SAMPLE_3,
    NEXTION_SAMPLE_4,
    NEXTION_SAMPLE_5,
    NEXTION_SAMPLE_6,
    NEXTION_SAMPLE_7,
    NEXTION_SAMPLE_8,
    NEXTION_SAMPLE_9,
    NEXTION_SAMPLE_10,
    NEXTION_SAMPLE_11,
    NEXTION_SAMPLE_12,
};

#define NEXTION_TXD (1)
#define NEXTION_RXD (3)
#define NEXTION_UART_PORT_NUM (0)
#define NEXTION_UART_BAUD_RATE (9600)
#define NEXTION_TASK_STACK_SIZE (2048)
#define NEXTION_BUF_SIZE (1024)

static const char *TAG_uart = "UART debug";

typedef enum
{
    RECEIVE_PAD,
    RECEIVE_SAMP,
} nextion_recv_state_e;

nextion_recv_state_e state;

bool compareMsg(char *receivedData, const char *compareData, size_t lenMsg)
{
    for (size_t i = 0; i < lenMsg; i++)
    {
        if (receivedData[i] != compareData[i])
            return false;
    }
    return true;
}

static void display_task(void *arg)
{
    int padSelect = 0;


    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = NEXTION_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(NEXTION_UART_PORT_NUM, NEXTION_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(NEXTION_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NEXTION_UART_PORT_NUM, NEXTION_TXD, NEXTION_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(NEXTION_BUF_SIZE);
    state = RECEIVE_PAD;

    while (1)
    {
        // Read data from the UART
        int len = uart_read_bytes(NEXTION_UART_PORT_NUM, data, (NEXTION_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);

        if (len)
        {
            data[len] = '\0';
            switch (state)
            {
            case RECEIVE_PAD:
                for (size_t padSelIndex = 0; padSelIndex < NEXTION_NUMBER_OF_PADS; padSelIndex++)
                {
                    if (compareMsg((char*)data, padMessages[padSelIndex], 4))
                    {
                        padSelect = padSelIndex;
                        state = RECEIVE_SAMP;
                        break;
                    }
                }
                break;
            case RECEIVE_SAMP:
                for (size_t sampSelIndex = 0; sampSelIndex < NEXTION_NUMBER_OF_SAMPLES; sampSelIndex++)
                {
                    if (compareMsg((char*)data, padSampleMsg[sampSelIndex], 4))
                    {
                        drumpadSamples[padSelect] = sampSelIndex;
                        state = RECEIVE_PAD;
                        break;
                    }
                }
                break;
            default:
                break;
            }

        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QueueHandle_t drumpadQueue;

static void drumpads_task(void *arg)
{
    drumpad padList[NUMBER_OF_DRUMPADS];
    drumpads pads;
    uint32_t drumpadhit;

    padList[0].setChannel(ADS111X_MUX_3_GND);
    padList[1].setChannel(ADS111X_MUX_1_GND);
    padList[2].setChannel(ADS111X_MUX_2_GND);
    padList[3].setChannel(ADS111X_MUX_0_GND);

    pads.installDrumpads(padList, NUMBER_OF_DRUMPADS);

    while (true)
    {
        pads.sampleADC();
        for (size_t i = 0; i < NUMBER_OF_DRUMPADS; i++)
        {
            if (padList[i].getState() == true)
            {
                drumpadhit = i + 1;
                xQueueSend(drumpadQueue, (void *)&drumpadhit, 0);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

#define AUDIO_TASK_STACK_SIZE (16384)
static const char *TAG_audio = "MAIN debug";
playlist_operator_handle_t sdcard_list_handle = NULL;

void sdcard_url_save_cb(void *user_data, char *url)
{
    playlist_operator_handle_t sdcard_handle = (playlist_operator_handle_t)user_data;
    esp_err_t ret = sdcard_list_save(sdcard_handle, url);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_main, "Fail to save sdcard url to sdcard playlist");
    }
}

const char *filter[] = {"mp3", "wav"};

static void audio_task(void *arg)
{

    audio_pipeline_handle_t pipeline;
    audio_element_handle_t fatfs_stream_reader, i2s_stream_writer, music_decoder;

    esp_log_level_set(TAG_audio, ESP_LOG_INFO);

    ESP_LOGI(TAG_audio, "[ 1 ] Mount sdcard");
    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    // Initialize SD Card peripheral
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
    sdcard_list_create(&sdcard_list_handle);
    sdcard_scan(sdcard_url_save_cb, "/sdcard", 0, filter, 2, sdcard_list_handle);
    sdcard_list_show(sdcard_list_handle);
    char *url = "";

    ESP_LOGI(TAG_audio, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG_audio, "[3.0] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG_audio, "[3.1] Create fatfs stream to read data from sdcard");
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    fatfs_stream_reader = fatfs_stream_init(&fatfs_cfg);

    ESP_LOGI(TAG_audio, "[3.2] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG_audio, "[3.3] Create wav decoder");
    wav_decoder_cfg_t wav_dec_cfg = DEFAULT_WAV_DECODER_CONFIG();
    music_decoder = wav_decoder_init(&wav_dec_cfg);

    ESP_LOGI(TAG_audio, "[3.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, fatfs_stream_reader, "file");
    audio_pipeline_register(pipeline, music_decoder, "dec");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG_audio, "[3.5] Link it together [sdcard]-->fatfs_stream-->music_decoder-->i2s_stream-->[codec_chip]");
    const char *link_tag[3] = {"file", "dec", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

    ESP_LOGI(TAG_audio, "[3.6] Set up uri: /sdcard/sample1.wav");
    audio_element_set_uri(fatfs_stream_reader, "/sdcard/sample1.wav");

    ESP_LOGI(TAG_audio, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG_audio, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG_audio, "[4.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);
    i2s_alc_volume_set(i2s_stream_writer, 100);
    // Example of linking elements into an audio pipeline -- END
    ESP_LOGI(TAG_audio, "[ 6 ] Listen for all pipeline events");
    while (1)
    {
        uint32_t drumpadHit;

        audio_event_iface_msg_t msg;

        esp_err_t ret = audio_event_iface_listen(evt, &msg, 0);

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)music_decoder && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO)
        {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(music_decoder, &music_info);
            audio_element_setinfo(i2s_stream_writer, &music_info);
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
        }

        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        // if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)i2s_stream_writer && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED)))
        // {
        // }

        if (xQueueReceive(drumpadQueue,
                          &(drumpadHit),
                          0) == pdPASS)
        {
            if (audio_event_iface_listen(evt, &msg, 0))
            {
                audio_event_iface_discard(evt);
            }
            char *url = NULL;
            audio_pipeline_stop(pipeline);

            sdcard_list_choose(sdcard_list_handle, drumpadSamples[drumpadHit - 1], &url);
            ESP_LOGI(TAG_audio, "Drumpad[%" PRId32 "] playing %s ", drumpadHit, url);
            audio_element_set_uri(fatfs_stream_reader, url);

            audio_pipeline_reset_ringbuffer(pipeline);
            audio_pipeline_reset_elements(pipeline);
            audio_pipeline_change_state(pipeline, AEL_STATE_INIT);

            audio_pipeline_run(pipeline);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG_audio, "[ 7 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, fatfs_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, music_decoder);

    /* Terminal the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Stop all periph before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(fatfs_stream_reader);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(music_decoder);
    esp_periph_set_destroy(set);
}

static const char *TAG_main = "MAIN debug";

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
    ESP_LOGI(TAG_main, "Startup");
    drumpadQueue = xQueueCreate(10, sizeof(uint32_t));

    // I2C
    ESP_LOGI(TAG_main, "external adc Initializing");
    ESP_ERROR_CHECK(i2cdev_init());
    // Drumpads
    ESP_LOGI(TAG_main, "Drumpads starting");
    xTaskCreatePinnedToCore(drumpads_task, "drumpads", DEFAULT_DRUMPADS_STACK_SIZE, NULL, 10, NULL, 0);
    // Audio
    ESP_LOGI(TAG_main, "Audio starting");
    xTaskCreatePinnedToCore(audio_task, "audio_task", AUDIO_TASK_STACK_SIZE, NULL, 10, NULL, 1);
    // Display
    ESP_LOGI(TAG_main, "Display starting");
    xTaskCreatePinnedToCore(display_task, "display_task", NEXTION_TASK_STACK_SIZE, NULL, 10, NULL, 0);

    ESP_LOGI(TAG_main, "All set up Running");
}
