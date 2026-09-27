#pragma once
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <cstdint>

class Ssd1306
{
public:
  static constexpr int WIDTH = 128, HEIGHT = 64;

  esp_err_t begin(gpio_num_t sda, gpio_num_t scl, uint8_t addr = 0x3C);
  void clear(bool on);
  esp_err_t flush();
  void pixel(int x, int y, bool on);
  void draw_char(int x, int y, char c, int scale);
  void text(int x, int y, const char *s, int scale);

  void probe();

private:
  i2c_master_bus_handle_t bus_ = nullptr;
  i2c_master_dev_handle_t dev_ = nullptr;
  uint8_t fb_[WIDTH * HEIGHT / 8]; // 1024

  esp_err_t write_cmd(const uint8_t *cmds, size_t n);
};