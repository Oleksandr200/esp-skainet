/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>

#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_board_init.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_process_sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "model_path.h"
#include "speech_commands_action.h"

static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;
static int play_voice = -2;
static int light = 0; // Toggle state: 1 = OFF, 0 = ON

#include "driver/rmt.h"
#include "led_strip.h"

#define LED_STRIP_GPIO 48
#define LED_STRIP_NUM 1
#define RELAY_GPIO 45

led_strip_t *led_strip = NULL;

void relay_init() {
  gpio_reset_pin(RELAY_GPIO);
  gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(RELAY_GPIO, light);
}

void led_init() {
  rmt_config_t rmt_tx = RMT_DEFAULT_CONFIG_TX(LED_STRIP_GPIO, RMT_CHANNEL_0);
  rmt_tx.clk_div = 2;

  ESP_ERROR_CHECK(rmt_config(&rmt_tx));
  ESP_ERROR_CHECK(rmt_driver_install(rmt_tx.channel, 0, 0));

  led_strip_config_t strip_config = {.max_leds = LED_STRIP_NUM,
                                     .dev = (led_strip_dev_t)rmt_tx.channel};

  led_strip = led_strip_new_rmt_ws2812(&strip_config);

  if (led_strip != NULL) {
    printf("LED WS2812 initialized successfully!\n");
    led_strip->clear(led_strip, 100);
  } else {
    printf("ERROR: Failed to initialize LED WS2812!\n");
  }
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
  if (led_strip != NULL) {
    led_strip->set_pixel(led_strip, 0, r, g, b);
    led_strip->refresh(led_strip, 100);
  } else {
    printf("ERROR: LED strip not initialized!\n");
  }
}

void play_music(void *arg) {
  while (task_flag) {
    switch (play_voice) {
    case -2:
      vTaskDelay(10);
      break;
    case -1:
      wake_up_action();
      play_voice = -2;
      break;
    default:
      speech_commands_action(play_voice);
      play_voice = -2;
      break;
    }
  }
  vTaskDelete(NULL);
}

void feed_Task(void *arg) {
  esp_afe_sr_data_t *afe_data = arg;
  int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
  int nch = afe_handle->get_channel_num(afe_data);
  int feed_channel = esp_get_feed_channel();
  assert(nch <= feed_channel);
  int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
  assert(i2s_buff);

  while (task_flag) {
    esp_get_feed_data(false, i2s_buff,
                      audio_chunksize * sizeof(int16_t) * feed_channel);
    afe_handle->feed(afe_data, i2s_buff);
  }

  if (i2s_buff) {
    free(i2s_buff);
    i2s_buff = NULL;
  }

  vTaskDelete(NULL);
}

void detect_Task(void *arg) {
  esp_afe_sr_data_t *afe_data = arg;
  int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
  char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);

  printf("multinet:%s\n", mn_name);

  esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
  model_iface_data_t *model_data = multinet->create(mn_name, 6000);
  int mu_chunksize = multinet->get_samp_chunksize(model_data);
  esp_mn_commands_update_from_sdkconfig(multinet, model_data);
  assert(mu_chunksize == afe_chunksize);
  multinet->print_active_speech_commands(model_data);

  printf("Detect task started\n");
  while (task_flag) {
    afe_fetch_result_t *res = afe_handle->fetch(afe_data);

    if (!res || res->ret_value == ESP_FAIL) {
      printf("Fetch error!\n");
      break;
    }

    if (res->wakeup_state == WAKENET_DETECTED) {
      printf("WAKEWORD DETECTED\n");
      multinet->clean(model_data);
      light = !light;

      if (light) {
        play_voice = 13;
      } else {
        play_voice = 14;
      }
      gpio_set_level(RELAY_GPIO, light);
      printf("Relay toggled to: %d\n", light);
    } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
      // play_voice = -1;
      printf("Channel verified: %d\n", res->trigger_channel_id);
    }
  }

  if (model_data) {
    multinet->destroy(model_data);
    model_data = NULL;
  }

  printf("Detect task exit\n");
  vTaskDelete(NULL);
}

void app_main() {
  models = esp_srmodel_init("model");
  ESP_ERROR_CHECK(esp_board_init(AUDIO_HAL_16K_SAMPLES, 1, 16));

  led_init();
  relay_init();

  afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;

  afe_config_t afe_config = AFE_CONFIG_DEFAULT();
  afe_config.wakenet_model_name =
      esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
  afe_config.aec_init = false;
  afe_config.pcm_config.total_ch_num = 2;
  afe_config.pcm_config.mic_num = 1;
  afe_config.pcm_config.ref_num = 1;

  esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);

  task_flag = 1;
  xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void *)afe_data, 5,
                          NULL, 1);
  xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void *)afe_data, 5,
                          NULL, 0);
  xTaskCreatePinnedToCore(&play_music, "play", 4 * 1024, NULL, 5, NULL, 1);
}
