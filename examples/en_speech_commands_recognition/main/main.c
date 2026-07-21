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

int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;
static int play_voice = -2;

#include "driver/rmt.h"
#include "led_strip.h"

#define LED_STRIP_GPIO 48
#define LED_STRIP_NUM 1

// Глобальный указатель на структуру из твоей библиотеки
led_strip_t *led_strip = NULL;

#define RELAY_GPIO 45

void relay_init() {
  gpio_reset_pin(RELAY_GPIO);
  gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
  // Изначально выключаем реле.
  // Если реле щелкнуло при старте — попробуй поменять 1 на 0.
  gpio_set_level(RELAY_GPIO, 0);
}

void led_init() {
  // 1. Настраиваем стандартный RMT канал ESP-IDF (Канал 0, GPIO 48)
  rmt_config_t rmt_tx = RMT_DEFAULT_CONFIG_TX(LED_STRIP_GPIO, RMT_CHANNEL_0);
  rmt_tx.clk_div = 2; // Стандартный делитель частоты для WS2812

  ESP_ERROR_CHECK(rmt_config(&rmt_tx));
  ESP_ERROR_CHECK(rmt_driver_install(rmt_tx.channel, 0, 0));

  // 2. Заполняем конфиг для твоей функции led_strip
  led_strip_config_t strip_config = {
      .max_leds = LED_STRIP_NUM,
      .dev =
          (led_strip_dev_t)rmt_tx.channel // Передаем настроенный RMT_CHANNEL_0
  };

  // 3. Вызываем твою функцию (с одним аргументом!)
  led_strip = led_strip_new_rmt_ws2812(&strip_config);

  if (led_strip != NULL) {
    printf("Успех: Светодиод WS2812 привязан к RMT каналу!\n");
    // Гасим диод при старте (твоя функция clear принимает таймаут)
    led_strip->clear(led_strip, 100);
  } else {
    printf("Критическая ошибка: led_strip_new_rmt_ws2812 вернул NULL!\n");
  }
}

// Функция установки цвета, написанная строго под твою библиотеку
void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
  if (led_strip != NULL) {
    // 1. Записываем цвет в буфер (индекс 0 - наш единственный диод)
    led_strip->set_pixel(led_strip, 0, r, g, b);

    // 2. Проталкиваем данные в диод (с таймаутом 100 мс, как требует функция
    // refresh)
    led_strip->refresh(led_strip, 100);
  } else {
    printf("Ошибка: led_strip не инициализирован!\n");
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
  esp_mn_commands_update_from_sdkconfig(
      multinet, model_data); // Add speech commands from sdkconfig
  assert(mu_chunksize == afe_chunksize);
  // print active speech commands
  multinet->print_active_speech_commands(model_data);

  printf("------------detect start------------\n");
  while (task_flag) {
    afe_fetch_result_t *res = afe_handle->fetch(afe_data);
    if (!res || res->ret_value == ESP_FAIL) {
      printf("fetch error!\n");
      break;
    }

    if (res->wakeup_state == WAKENET_DETECTED) {
      printf("WAKEWORD DETECTED\n");
      multinet->clean(model_data);
      set_led_color(0, 255, 0);
    } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
      play_voice = -1;
      detect_flag = 1;
      printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n",
             res->trigger_channel_id);
      // afe_handle->disable_wakenet(afe_data);
      // afe_handle->disable_aec(afe_data);
    }

    if (detect_flag == 1) {
      esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

      if (mn_state == ESP_MN_STATE_DETECTING) {
        continue;
      }

      if (mn_state == ESP_MN_STATE_DETECTED) {
        esp_mn_results_t *mn_result = multinet->get_results(model_data);
        for (int i = 0; i < mn_result->num; i++) {
          printf(
              "TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f\n",
              i + 1, mn_result->command_id[i], mn_result->phrase_id[i],
              mn_result->string, mn_result->prob[i]);

          // Command "vykluchi svet" (ID = 32)
          switch (mn_result->command_id[i]) {
          case 0: // Privet
          case 1: // Hello
            detect_flag = 2;
            play_voice = 13;
            led_strip->clear(led_strip, 100);
            gpio_set_level(RELAY_GPIO, 1);
            break;
          case 3: // Bye Bye
            detect_flag = 2;
            play_voice = 14;
            led_strip->clear(led_strip, 100);
            gpio_set_level(RELAY_GPIO, 0);
            break;
          default:
            break;
          }
        }
        printf("-----------listening-----------\n");
      }

      if (mn_state == ESP_MN_STATE_TIMEOUT) {
        esp_mn_results_t *mn_result = multinet->get_results(model_data);
        printf("timeout, string:%s\n", mn_result->string);
        afe_handle->enable_wakenet(afe_data);
        detect_flag = 0;
        led_strip->clear(led_strip, 100);
        printf("\n-----------awaits to be waken up-----------\n");
        continue;
      }
    }
  }
  if (model_data) {
    multinet->destroy(model_data);
    model_data = NULL;
  }
  printf("detect exit\n");
  vTaskDelete(NULL);
}

void app_main() {
  models =
      esp_srmodel_init("model"); // partition label defined in partitions.csv
  ESP_ERROR_CHECK(esp_board_init(AUDIO_HAL_16K_SAMPLES, 1, 16));
  // ESP_ERROR_CHECK(esp_sdcard_init("/sdcard", 10));

  led_init();
  relay_init();

#if defined CONFIG_ESP32_KORVO_V1_1_BOARD
  led_init();
#endif

#if CONFIG_IDF_TARGET_ESP32
  printf("This demo only support ESP32S3\n");
  return;
#else
  afe_handle = (esp_afe_sr_iface_t *)&ESP_AFE_SR_HANDLE;
#endif

  afe_config_t afe_config = AFE_CONFIG_DEFAULT();
  afe_config.wakenet_model_name =
      esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
  ;
#if defined CONFIG_ESP32_S3_BOX_BOARD || defined CONFIG_ESP32_S3_EYE_BOARD ||  \
    CONFIG_ESP32_S3_DEVKIT_C
  afe_config.aec_init = false;
#if defined CONFIG_ESP32_S3_EYE_BOARD || CONFIG_ESP32_S3_DEVKIT_C
  afe_config.pcm_config.total_ch_num = 2;
  afe_config.pcm_config.mic_num = 1;
  afe_config.pcm_config.ref_num = 1;
#endif
#endif
  esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);

  task_flag = 1;
  xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void *)afe_data, 5,
                          NULL, 1);
  xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void *)afe_data, 5,
                          NULL, 0);
#if defined CONFIG_ESP32_S3_KORVO_1_V4_0_BOARD
  xTaskCreatePinnedToCore(&led_Task, "led", 2 * 1024, NULL, 5, NULL, 0);
#endif
  // play_music task runs on all boards (including ESP32-S3-DevKit-C)
  xTaskCreatePinnedToCore(&play_music, "play", 4 * 1024, NULL, 5, NULL, 1);

  // // You can call afe_handle->destroy to destroy AFE.
  // task_flag = 0;

  // printf("destroy\n");
  // afe_handle->destroy(afe_data);
  // afe_data = NULL;
  // printf("successful\n");
}
