#ifdef ROLE_BTN_LED
#include "board_pins.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cstdio>
#include <cstring>
#include <cinttypes>
#include <atomic>
#include <algorithm>
#include <ssd1306.h>

const BaseType_t BUTTON_TASK_CORE = 0;
const BaseType_t LED_TASK_CORE = 1;
const BaseType_t UI_TASK_CORE = 0;

static const char *TAG = "FREE_RTOS_TASK";
const int MAX_LED_INTERVAL_MS = 2000;
const int MIN_LED_INTERVAL_MS = 250;
const int LED_INTERVAL_MULTIPLIER = 2;
const int DEBOUNCE_US = 20000;
const int DOUBLE_PRESS_THRESHOLD_US = 300000;
static std::atomic<int> dropped_button_events{0};

static QueueHandle_t button_event_queue = nullptr;
static QueueHandle_t led_event_queue = nullptr;
static QueueHandle_t ui_state_queue = nullptr;
static TaskHandle_t h_button = nullptr;
static TaskHandle_t h_led = nullptr;
static TaskHandle_t h_ui = nullptr;

static Ssd1306 oled;

struct button_event_t
{
  int64_t timestamp_us;
  int level;
};

enum class LedIntervalDirection : uint8_t
{
  FORWARD = 0,
  BACK = 1
};

struct led_event_t
{
  LedIntervalDirection interval_direction;
};

struct ui_state_t
{
  int interval_ms;
  LedIntervalDirection last_direction;
  uint32_t press_count;
};

int us_to_ticks(int us)
{
  return pdMS_TO_TICKS(us / 1000);
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  int64_t us = esp_timer_get_time();
  int level = gpio_get_level(app_board_pins::BTN_PIN);
  button_event_t event{us, level};

  BaseType_t hp_woken = pdFALSE;
  BaseType_t result = xQueueSendFromISR(button_event_queue, &event, &hp_woken);
  if (result != pdTRUE)
  {
    dropped_button_events.fetch_add(1, std::memory_order_relaxed);
  }

  portYIELD_FROM_ISR(hp_woken);
}

bool wait_button_event(TickType_t timeout, button_event_t *event_out)
{
  button_event_t event;
  if (xQueueReceive(button_event_queue, &event, timeout) != pdTRUE)
    return false;

  event_out->timestamp_us = event.timestamp_us;
  event_out->level = event.level;

  // debounce here
  while (xQueueReceive(button_event_queue, &event, us_to_ticks(DEBOUNCE_US)) == pdTRUE)
    event_out->level = event.level;

  return true;
}

void button_task(void *arg)
{
  button_event_t event;
  for (;;)
  {
    if (!wait_button_event(portMAX_DELAY, &event))
      continue;
    if (event.level != 0)
      continue; // only care about button press

    int64_t t0 = event.timestamp_us;
    bool is_double = false;

    for (;;)
    {
      int64_t remaining = DOUBLE_PRESS_THRESHOLD_US - (esp_timer_get_time() - t0);
      if (remaining <= 0)
        break;
      if (!wait_button_event(us_to_ticks(remaining), &event))
        break;
      if (event.level != 0)
        continue;
      is_double = true;
      break;
    }

    LedIntervalDirection direction = is_double ? LedIntervalDirection::BACK : LedIntervalDirection::FORWARD;

    led_event_t ev = led_event_t{direction};
    xQueueSend(led_event_queue, &ev, 0);
  }

  uint32_t n = dropped_button_events.load(std::memory_order_relaxed);
  if (n > 0)
  {
    ESP_LOGW(TAG, "Dropped %d button events", n);
    dropped_button_events.store(0, std::memory_order_relaxed);
  }
}

void ui_task(void *arg)
{
  if (oled.begin(oled_pins::SDA_PIN, oled_pins::SCL_PIN, oled_pins::I2C_ADDR) != ESP_OK)
  {
    ESP_LOGE(TAG, "OLED init failed, UI task stopped");
    vTaskDelete(nullptr);
  }

  ui_state_t st{MIN_LED_INTERVAL_MS, LedIntervalDirection::FORWARD, 0};
  char line[24];

  for (;;)
  {
    oled.clear(false);
    oled.text(0, 0, "BTN + LED", 1);

    snprintf(line, sizeof(line), "%dms", st.interval_ms);
    oled.text(0, 14, line, 3);

    snprintf(line, sizeof(line), "click: %s",
             st.last_direction == LedIntervalDirection::BACK ? "DOUBLE" : "SINGLE");
    oled.text(0, 44, line, 1);

    snprintf(line, sizeof(line), "count: %" PRIu32, st.press_count);
    oled.text(0, 54, line, 1);

    oled.flush();

    xQueueReceive(ui_state_queue, &st, portMAX_DELAY);
  }
}

void led_blinking_task(void *arg)
{
  led_event_t event;
  uint8_t prev_level = 0;
  uint32_t press_count = 0;
  int blink_interval_ms = MIN_LED_INTERVAL_MS;

  for (;;)
  {
    if (xQueueReceive(led_event_queue, &event, pdMS_TO_TICKS(blink_interval_ms)) == pdTRUE)
    {
      // blink interval change happened
      if (event.interval_direction == LedIntervalDirection::BACK)
      {
        blink_interval_ms = blink_interval_ms <= MIN_LED_INTERVAL_MS ? MAX_LED_INTERVAL_MS : blink_interval_ms / LED_INTERVAL_MULTIPLIER;
      }
      else
      {
        blink_interval_ms = blink_interval_ms >= MAX_LED_INTERVAL_MS ? MIN_LED_INTERVAL_MS : blink_interval_ms * LED_INTERVAL_MULTIPLIER;
      }

      // trun off led
      ESP_LOGI(TAG, "New blink interval is %d ms", blink_interval_ms);
      prev_level = 0;
      gpio_set_level(app_board_pins::LED_PIN, prev_level);

      ui_state_t st{blink_interval_ms, event.interval_direction, ++press_count};
      xQueueOverwrite(ui_state_queue, &st);
    }
    else
    {
      // just blinking, everything is good
      uint8_t new_level = prev_level == 0 ? 1 : 0;
      gpio_set_level(app_board_pins::LED_PIN, new_level);
      prev_level = new_level;
    }
  }
}

void btn_led_main()
{
  ESP_LOGI(TAG, "STARTED APPLICATION");
  gpio_install_isr_service(0);
  gpio_isr_handler_add(app_board_pins::BTN_PIN, gpio_isr_handler, nullptr);

  gpio_config_t btn_io_conf{
      .pin_bit_mask = 1ULL << app_board_pins::BTN_PIN,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_ANYEDGE, // press and release
  };

  ESP_ERROR_CHECK(gpio_config(&btn_io_conf));

  gpio_config_t led_io_conf{
      .pin_bit_mask = 1ULL << app_board_pins::LED_PIN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE};

  ESP_ERROR_CHECK(gpio_config(&led_io_conf));

  button_event_queue = xQueueCreate(10, sizeof(button_event_t));
  if (button_event_queue == nullptr)
  {
    ESP_LOGE(TAG, "Failed to create button event queue");
    return;
  }

  led_event_queue = xQueueCreate(10, sizeof(led_event_t));
  if (led_event_queue == nullptr)
  {
    ESP_LOGE(TAG, "Failed to create led event queue");
    return;
  }

  // length 1, required by xQueueOverwrite
  ui_state_queue = xQueueCreate(1, sizeof(ui_state_t));
  if (ui_state_queue == nullptr)
  {
    ESP_LOGE(TAG, "Failed to create ui state queue");
    return;
  }

  BaseType_t result = xTaskCreatePinnedToCore(button_task, "button_task", 1024, nullptr, 10, &h_button, BUTTON_TASK_CORE);
  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create button task");
    return;
  }

  result = xTaskCreatePinnedToCore(
      led_blinking_task,
      "led_task",
      1024,
      nullptr,
      5,
      &h_led,
      LED_TASK_CORE);

  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create led task");
    return;
  }

  result = xTaskCreatePinnedToCore(ui_task, "ui_task", 4096, nullptr, 3, &h_ui, UI_TASK_CORE);
  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create ui task");
    return;
  }
}
#endif // ROLE_BTN_LED
