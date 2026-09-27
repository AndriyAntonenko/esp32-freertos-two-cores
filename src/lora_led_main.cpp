#ifdef ROLE_LED_LORA
#include "board_pins.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cstdio>
#include <cstring>
#include <atomic>
#include <algorithm>
#include <sx127x.h>
#include <packet.h>

const BaseType_t BUTTON_TASK_CORE = 0;
const BaseType_t LED_TASK_CORE = 0;
const BaseType_t LORA_TASK_CORE = 1;

static const char *TAG = "LORA_LED";
const int LED_INTERVAL_MS = 100;
const int DEBOUNCE_US = 20000;
const int DOUBLE_PRESS_THRESHOLD_US = 300000;
static std::atomic<int> dropped_button_events{0};

static QueueHandle_t button_event_queue = nullptr;
static QueueHandle_t radio_msg_queue = nullptr;

static TaskHandle_t h_button = nullptr;
static TaskHandle_t h_led = nullptr;
static TaskHandle_t h_radio = nullptr;

uint32_t state_message_no = 0;

struct button_event_t
{
  int64_t timestamp_us;
  int level;
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

    state_message_no = state_message_no + 1;
    packet::Action act = packet::Action{};
    act.event = is_double ? packet::click_t::DOUBLE : packet::click_t::SINGLE;
    act.timestamp_us = t0;
    act.no = state_message_no;

    if (xQueueSend(radio_msg_queue, &act, 0) != pdTRUE)
    {
      ESP_LOGW(TAG, "radio busy, action #%" PRIu32 " dropped", act.no);
    }

    uint32_t n = dropped_button_events.load(std::memory_order_relaxed);
    if (n > 0)
    {
      ESP_LOGW(TAG, "Dropped %d button events", n);
      dropped_button_events.store(0, std::memory_order_relaxed);
    }
  }
}

void led_blinking_task(void *arg)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(LED_INTERVAL_MS);
  uint8_t prev_level = 0;
  uint32_t toggles_count = 0;
  int64_t next_deadline_us = esp_timer_get_time() + LED_INTERVAL_MS * 1000;
  int64_t max_late_us = 0;

  for (;;)
  {
    xTaskDelayUntil(&last_wake_time, frequency);
    int64_t late_us = esp_timer_get_time() - next_deadline_us;
    next_deadline_us += LED_INTERVAL_MS * 1000;
    if (late_us > max_late_us)
      max_late_us = late_us;

    if (toggles_count % 50 == 0)
    {
      ESP_LOGI(TAG, "LED max late %dus", max_late_us);
    }

    uint8_t new_level = prev_level == 0 ? 1 : 0;
    gpio_set_level(app_board_pins::LED_PIN, new_level);
    prev_level = new_level;
    toggles_count += 1;
  }
}

void radio_task(void *arg)
{
  Sx127x radio;
  ESP_ERROR_CHECK(radio.begin(lora_pins::CHIP_SELECT_PIN, lora_pins::SCK_PIN,
                              lora_pins::MOSI_PIN, lora_pins::MISO_PIN,
                              lora_pins::RST_PIN, lora_pins::DIO0_PIN));

  uint8_t chip = radio.read_register(0x42); // RegVersion, 0x12 on SX1276
  if (chip != 0x12)
  {
    ESP_LOGE(TAG, "SX1276 not responding, RegVersion 0x%02X", chip);
    vTaskDelete(nullptr);
  }

  Sx127x::Config cfg;
  cfg.sf = Sx127x::Sf::SF11;
  cfg.bw = Sx127x::Bw::BW_125K;
  ESP_ERROR_CHECK(radio.configure(cfg));

  ESP_LOGI(TAG, "[core %d] radio ready, %u B packet, predicted airtime %u ms",
           xPortGetCoreID(), (unsigned)packet::SIZE,
           (unsigned)radio.time_on_air_ms(packet::SIZE));

  packet::Action act;
  uint8_t buf[packet::SIZE];

  for (;;)
  {
    if (xQueueReceive(radio_msg_queue, &act, portMAX_DELAY) != pdTRUE)
      continue;

    packet::buildPacket(act, buf);
    esp_err_t err = radio.send(buf, packet::SIZE);

    int64_t latency_ms = (esp_timer_get_time() - act.timestamp_us) / 1000;
    ESP_LOGI(TAG, "send #%" PRIu32 " (%s): %s, %lld ms after press",
             act.no, act.event == packet::click_t::DOUBLE ? "double" : "single",
             esp_err_to_name(err), latency_ms);
  }
}

void lora_led_main()
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

  radio_msg_queue = xQueueCreate(10, sizeof(packet::Msg));
  if (radio_msg_queue == nullptr)
  {
    ESP_LOGE(TAG, "Failed to create radio event queue");
    return;
  }

  BaseType_t result = xTaskCreatePinnedToCore(button_task, "button_task", 1024, nullptr, 10, &h_button, BUTTON_TASK_CORE);
  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create button task");
    return;
  }

  result = xTaskCreatePinnedToCore(radio_task, "radio_task", 1300, nullptr, 5, &h_radio, LORA_TASK_CORE);
  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create radio task");
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
}
#endif // ROLE_LED_LORA
