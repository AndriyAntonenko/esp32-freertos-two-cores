#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class Sx127x
{
public:
  esp_err_t configure(uint32_t freq_hz);
  esp_err_t begin(gpio_num_t cs, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso, gpio_num_t rst, gpio_num_t dio0);
  uint8_t read_register(uint8_t reg);
  void write_register(uint8_t reg, uint8_t value);
  SemaphoreHandle_t irq_ = nullptr;
  esp_err_t send(const uint8_t *data, uint8_t len);
  void start_rx();
  esp_err_t recv(uint8_t *buf, uint8_t *len, TickType_t timeout);

  // getters rssi and snr

  int16_t rssi() { return last_rssi_; }
  int8_t snr() { return last_snr_; }

private:
  spi_device_handle_t dev_ = nullptr;
  int16_t last_rssi_ = 0;
  int8_t last_snr_ = 0;
  void reset(gpio_num_t rst);
};