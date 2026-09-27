#ifdef ROLE_MUTEX_DEADLOCK
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include "board_pins.h"
#include <esp_task_wdt.h>

constexpr uint8_t WORKER_A_CORE = 0;
constexpr uint8_t WORKER_B_CORE = 1;
constexpr uint8_t LED_CORE = 0;
constexpr uint8_t HEARTBEAT_CORE = 1;
static const char *TAG = "MUTEX_DEADLOCK";

// led blink interval, that will be guarded by mutex_a
static int blink_interval_ms = 250;

SemaphoreHandle_t mutex_a;
SemaphoreHandle_t mutex_b;

TaskHandle_t h_worker_a;
TaskHandle_t h_worker_b;
TaskHandle_t h_led;
TaskHandle_t h_heartbeat;

void worker_a(void *arg)
{
  ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));

  for (int i = 0; i < 6; i++)
  {
    esp_task_wdt_reset();
    ESP_LOGI(TAG, "A working normally (%d)", i);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  xSemaphoreTake(mutex_a, portMAX_DELAY);
  ESP_LOGI(TAG, "A holds mutex_a, wants mutex_b");
  vTaskDelay(pdMS_TO_TICKS(100)); // ensure that worker_b takes mutex first
  xSemaphoreTake(mutex_b, portMAX_DELAY);
}

void worker_b(void *arg)
{
  ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));

  for (int i = 0; i < 6; i++)
  {
    esp_task_wdt_reset();
    ESP_LOGI(TAG, "A working normally (%d)", i);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  xSemaphoreTake(mutex_b, portMAX_DELAY);
  ESP_LOGI(TAG, "B holds mutex_b, wants mutex_a");
  vTaskDelay(pdMS_TO_TICKS(100));
  xSemaphoreTake(mutex_a, portMAX_DELAY); // never returns
}

void led_task(void *arg)
{
  uint8_t level = 0;
  for (;;)
  {
    xSemaphoreTake(mutex_a, portMAX_DELAY); // <-- this will block forever once deadlock happened
    int interval = blink_interval_ms;
    xSemaphoreGive(mutex_a);

    level = level == 0 ? 1 : 0;
    gpio_set_level(app_board_pins::LED_PIN, level);
    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

// This task never touch mutexes, and proves that scheduler alive
void heartbeat_task(void *arg)
{
  for (uint32_t n = 0;; n++)
  {
    ESP_LOGI(TAG, "heartbeat %" PRIu32 ", led_task state %d",
             n, (int)eTaskGetState(h_led));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void mutex_deadlock()
{
  mutex_a = xSemaphoreCreateMutex();
  mutex_b = xSemaphoreCreateMutex();

  gpio_config_t led_io_conf{
      .pin_bit_mask = 1ULL << app_board_pins::LED_PIN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE};

  ESP_ERROR_CHECK(gpio_config(&led_io_conf));

  BaseType_t result = xTaskCreatePinnedToCore(
      worker_a,
      "worker_a",
      1024,
      nullptr,
      5,
      &h_worker_a,
      WORKER_A_CORE);

  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create worker_a task, returning from main function...");
    return;
  }

  result = xTaskCreatePinnedToCore(
      worker_b,
      "worker_b",
      1024,
      nullptr,
      5,
      &h_worker_b,
      WORKER_B_CORE);

  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create worker_b task, returning from main function...");
    return;
  }

  result = xTaskCreatePinnedToCore(
      led_task,
      "led",
      1024,
      nullptr,
      5,
      &h_led,
      LED_CORE);

  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create led task, returning from main function...");
    return;
  }

  result = xTaskCreatePinnedToCore(
      heartbeat_task,
      "heartbeat",
      1024,
      nullptr,
      5,
      &h_heartbeat,
      HEARTBEAT_CORE);

  if (result != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to create led task, returning from main function...");
    return;
  }
}

#endif // ROLE_MUTEX_DEADLOCK