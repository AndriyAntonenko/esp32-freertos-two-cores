#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class Sx127x
{
public:
  // RegModemConfig2 (0x1E) bits 7..4
  enum class Sf : uint8_t
  {
    // TODO: Add later but handle explicit header
    // SF6 = 6 << 4,
    SF7 = 7 << 4,
    SF8 = 8 << 4,
    SF9 = 9 << 4,
    SF10 = 10 << 4,
    SF11 = 11 << 4,
    SF12 = 12 << 4
  };

  // REgModemConfig1(0x1D), bits 7..4
  enum class Bw : uint8_t
  {
    BW_7K8 = 0 << 4,
    BW_10K4 = 1 << 4,
    BW_15K6 = 2 << 4,
    BW_20K8 = 3 << 4,
    BW_31K25 = 4 << 4,
    BW_41K7 = 5 << 4,
    BW_62K5 = 6 << 4,
    BW_125K = 7 << 4,
    BW_250K = 8 << 4,
    BW_500K = 9 << 4,
  };

  // RegModemConfig1, bits 3..1
  enum class Cr : uint8_t
  {
    CR_4_5 = 1 << 1,
    CR_4_6 = 2 << 1,
    CR_4_7 = 3 << 1,
    CR_4_8 = 4 << 1,
  };

  struct Config
  {
    uint32_t freq_hz = 433'000'000;
    Sf sf = Sf::SF7;
    Bw bw = Bw::BW_125K;
    Cr cr = Cr::CR_4_5;
    bool crc_on = true;
    uint16_t preamble = 8;
    uint8_t sync_word = 0x12;
  };

  esp_err_t configure(const Config &cfg);

  // Time on air for `len` payload bytes with current config (datasheet 4.1.1.7).
  // Valid after configure().
  uint32_t time_on_air_ms(uint8_t len) const;

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
  // timeout used by the last send(), for logging
  int32_t timeout_ms() { return timeout_ms_; }

private:
  Config cfg_{};
  spi_device_handle_t dev_ = nullptr;
  int16_t last_rssi_ = 0;
  int8_t last_snr_ = 0;
  int32_t timeout_ms_ = 0;
  void reset(gpio_num_t rst);
};