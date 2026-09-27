#include "ssd1306.h"
#include "esp_check.h"
#include "font5x7.h"
#include <cstring>

static const char *TAG = "Ssd1306";

static const uint8_t INIT[] = {
    0xAE,
    0xD5,
    0x80,
    0xA8,
    0x3F,
    0xD3,
    0x00,
    0x40,
    0x8D,
    0x14, // charge pump on
    0x20,
    0x00,
    0xA1,
    0xC8,
    0xDA,
    0x12,
    0x81,
    0xCF,
    0xD9,
    0xF1,
    0xDB,
    0x40,
    0xA4,
    0xA6,
    0xAF, // display on
};

esp_err_t Ssd1306::begin(gpio_num_t sda, gpio_num_t scl, uint8_t addr)
{
  i2c_master_bus_config_t bus_cfg{};
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = sda;
  bus_cfg.scl_io_num = scl;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;
  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus_), TAG, "bus");

  i2c_device_config_t dev_cfg{};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = addr;
  dev_cfg.scl_speed_hz = 400000;

  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_, &dev_cfg, &dev_), TAG, "dev");

  ESP_RETURN_ON_ERROR(write_cmd(INIT, sizeof(INIT)), TAG, "init");

  clear(false);
  return flush();
}

void Ssd1306::probe()
{
  ESP_LOGI(TAG, "probe: %s", esp_err_to_name(i2c_master_probe(bus_, 0x3C, 100)));
}

void Ssd1306::clear(bool on)
{
  memset(fb_, on ? 0xFF : 0x00, sizeof(fb_));
}

esp_err_t Ssd1306::flush()
{
  const uint8_t win[] = {0x21, 0, 127, // column range
                         0x22, 0, 7};  // page range
  ESP_RETURN_ON_ERROR(write_cmd(win, sizeof(win)), TAG, "win");

  static uint8_t out[1 + sizeof(fb_)];
  out[0] = 0x40; // data follows
  memcpy(out + 1, fb_, sizeof(fb_));
  return i2c_master_transmit(dev_, out, sizeof(out), 1000);
}

esp_err_t Ssd1306::write_cmd(const uint8_t *cmds, size_t n)
{
  uint8_t buf[n + 1];
  buf[0] = 0x00;
  memcpy(buf + 1, cmds, n);
  return i2c_master_transmit(dev_, buf, n + 1, 100);
}

void Ssd1306::pixel(int x, int y, bool on)
{
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
    return;
  uint8_t &b = fb_[(y / 8) * WIDTH + x];
  uint8_t mask = 1 << (y % 8);
  on ? (b |= mask) : (b &= ~mask);
}

void Ssd1306::draw_char(int x, int y, char c, int scale)
{
  if (c < 0x20 || c > 0x7E)
    c = '?';
  const uint8_t *g = &FONT5X7[(c - 0x20) * 5];
  for (int col = 0; col < 5; col++)
    for (int row = 0; row < 8; row++)
      if (g[col] & (1 << row))
        for (int dy = 0; dy < scale; dy++)
          for (int dx = 0; dx < scale; dx++)
            pixel(x + col * scale + dx, y + row * scale + dy, true);
}

void Ssd1306::text(int x, int y, const char *s, int scale)
{
  while (*s)
  {
    draw_char(x, y, *s++, scale);
    x += 6 * scale;
  }
}