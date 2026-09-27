#pragma once
#include <driver/gpio.h>
#include <cstdint>

namespace lora_pins
{
  constexpr gpio_num_t RST_PIN = GPIO_NUM_23;
  constexpr gpio_num_t DIO0_PIN = GPIO_NUM_26;
  constexpr gpio_num_t DIO1_PIN = GPIO_NUM_33;
  constexpr gpio_num_t DIO2_PIN = GPIO_NUM_32;
  constexpr gpio_num_t CHIP_SELECT_PIN = GPIO_NUM_18;

  constexpr gpio_num_t SCK_PIN = GPIO_NUM_5;
  constexpr gpio_num_t MISO_PIN = GPIO_NUM_19;
  constexpr gpio_num_t MOSI_PIN = GPIO_NUM_27;
}

namespace oled_pins
{
  constexpr gpio_num_t SDA_PIN = GPIO_NUM_21;
  constexpr gpio_num_t SCL_PIN = GPIO_NUM_22;
  constexpr uint8_t I2C_ADDR = 0x3C;
}

namespace app_board_pins
{
  constexpr gpio_num_t BTN_PIN = GPIO_NUM_4;
  constexpr gpio_num_t LED_PIN = GPIO_NUM_25;
};
