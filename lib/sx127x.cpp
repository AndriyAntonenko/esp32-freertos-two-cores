#include "sx127x.h"
#include <freertos/FreeRTOS.h>

constexpr uint8_t REG_OP_MODE = 0x01, REG_FRF_MSB = 0x06, REG_FRF_MID = 0x07,
                  REG_FRF_LSB = 0x08, REG_PA_CONFIG = 0x09, REG_LNA = 0x0C,
                  REG_FIFO_ADDR_PTR = 0x0D, REG_FIFO_TX_BASE = 0x0E, REG_FIFO_RX_BASE = 0x0F,
                  REG_IRQ_FLAGS = 0x12, REG_MODEM_CONFIG_1 = 0x1D, REG_MODEM_CONFIG_2 = 0x1E,
                  REG_MODEM_CONFIG_3 = 0x26, REG_PREAMBLE_MSB = 0x20, REG_PREAMBLE_LSB = 0x21,
                  REG_PAYLOAD_LENGTH = 0x22, REG_SYNC_WORD = 0x39, REG_DIO_MAPPING_1 = 0x40;

constexpr uint8_t MODE_LORA = 0x80, MODE_SLEEP = 0x00, MODE_STDBY = 0x01, MODE_TX = 0x03;

constexpr uint8_t REG_FIFO = 0x00, IRQ_TX_DONE = 0x08;

constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10, REG_RX_NB_BYTES = 0x13,
                  REG_PKT_SNR_VALUE = 0x19, REG_PKT_RSSI_VALUE = 0x1A;
constexpr uint8_t MODE_RX_CONTINUOUS = 0x05;
constexpr uint8_t IRQ_RX_DONE = 0x40, IRQ_CRC_ERROR = 0x20;

static void IRAM_ATTR dio0_isr(void *arg)
{
  auto *self = static_cast<Sx127x *>(arg);
  BaseType_t hp = pdFALSE;
  xSemaphoreGiveFromISR(self->irq_, &hp);
  portYIELD_FROM_ISR(hp);
}

esp_err_t Sx127x::configure(uint32_t freq_hz)
{
  write_register(REG_OP_MODE, MODE_LORA | MODE_SLEEP);
  uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000;
  write_register(REG_FRF_MSB, (frf >> 16) & 0xFF);
  write_register(REG_FRF_MID, (frf >> 8) & 0xFF);
  write_register(REG_FRF_LSB, frf & 0xFF);

  write_register(REG_FIFO_TX_BASE, 0x00);
  write_register(REG_FIFO_RX_BASE, 0x00);
  write_register(REG_LNA, 0x23);
  write_register(REG_MODEM_CONFIG_1, 0x72);
  write_register(REG_MODEM_CONFIG_2, 0x74);
  write_register(REG_MODEM_CONFIG_3, 0x04);
  write_register(REG_PREAMBLE_MSB, 0x00);
  write_register(REG_PREAMBLE_LSB, 0x08);
  write_register(REG_SYNC_WORD, 0x12);
  write_register(REG_PA_CONFIG, 0x8F);

  write_register(REG_OP_MODE, MODE_LORA | MODE_STDBY);
  return ESP_OK;
}

esp_err_t Sx127x::begin(gpio_num_t cs, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso, gpio_num_t rst, gpio_num_t dio0)
{
  irq_ = xSemaphoreCreateBinary();

  gpio_config_t dio0_io{};
  dio0_io.pin_bit_mask = 1ULL << dio0;
  dio0_io.mode = GPIO_MODE_INPUT;
  dio0_io.intr_type = GPIO_INTR_POSEDGE;
  gpio_config(&dio0_io);

  gpio_isr_handler_add(
      dio0,
      dio0_isr,
      this);

  Sx127x::reset(rst);

  spi_bus_config_t bus{};
  bus.mosi_io_num = mosi;
  bus.miso_io_num = miso;
  bus.sclk_io_num = sck;
  bus.quadwp_io_num = -1;  // unused
  bus.quadhd_io_num = -1;  // unused
  bus.max_transfer_sz = 0; // 0 → default 4092 bytes

  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));

  spi_device_interface_config_t devcfg{};
  devcfg.clock_speed_hz = 8 * 1000 * 1000; // 8 MHz
  devcfg.mode = 0;                         // SPI mode 0
  devcfg.spics_io_num = cs;
  devcfg.queue_size = 1;

  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &dev_));

  return ESP_OK;
}

void Sx127x::reset(gpio_num_t rst)
{
  gpio_config_t io{};
  io.pin_bit_mask = 1ULL << rst;
  io.mode = GPIO_MODE_OUTPUT;
  gpio_config(&io);

  gpio_set_level(rst, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level(rst, 1);
  vTaskDelay(pdMS_TO_TICKS(10));
}

uint8_t Sx127x::read_register(uint8_t reg)
{
  spi_transaction_t t{};
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = 16;
  t.tx_data[0] = reg & 0x7F;
  t.tx_data[1] = 0x00;

  ESP_ERROR_CHECK(spi_device_polling_transmit(dev_, &t));
  return t.rx_data[1];
}

void Sx127x::write_register(uint8_t reg, uint8_t value)
{
  spi_transaction_t t{};
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = 16;
  t.tx_data[0] = reg | 0x80;
  t.tx_data[1] = value;
  ESP_ERROR_CHECK(spi_device_polling_transmit(dev_, &t));
}

esp_err_t Sx127x::send(const uint8_t *data, uint8_t len)
{
  write_register(REG_OP_MODE, MODE_LORA | MODE_STDBY);
  write_register(REG_DIO_MAPPING_1, 0x40);
  write_register(REG_IRQ_FLAGS, 0xFF);
  write_register(REG_FIFO_ADDR_PTR, 0x00);
  for (uint8_t i = 0; i < len; i++)
    write_register(REG_FIFO, data[i]);
  write_register(REG_PAYLOAD_LENGTH, len);

  xSemaphoreTake(irq_, 0);
  write_register(REG_OP_MODE, MODE_LORA | MODE_TX);

  if (xSemaphoreTake(irq_, pdMS_TO_TICKS(1000)) != pdTRUE)
    return ESP_ERR_TIMEOUT;

  write_register(REG_IRQ_FLAGS, IRQ_TX_DONE);
  return ESP_OK;
}

void Sx127x::start_rx()
{
  write_register(REG_DIO_MAPPING_1, 0x00);
  write_register(REG_IRQ_FLAGS, 0xFF);
  write_register(REG_OP_MODE, MODE_LORA | MODE_RX_CONTINUOUS);
}

esp_err_t Sx127x::recv(uint8_t *buf, uint8_t *len, TickType_t timeout)
{
  if (xSemaphoreTake(irq_, timeout) != pdTRUE)
    return ESP_ERR_TIMEOUT;

  uint8_t flags = read_register(REG_IRQ_FLAGS);
  write_register(REG_IRQ_FLAGS, 0xFF); // write-1-to-clear

  if (flags & IRQ_CRC_ERROR)
    return ESP_ERR_INVALID_CRC;

  uint8_t n = read_register(REG_RX_NB_BYTES);
  if (n > *len)
    return ESP_ERR_INVALID_SIZE;

  write_register(REG_FIFO_ADDR_PTR, read_register(REG_FIFO_RX_CURRENT_ADDR));
  for (uint8_t i = 0; i < n; i++)
    buf[i] = read_register(REG_FIFO);

  *len = n;
  last_rssi_ = (int16_t)read_register(REG_PKT_RSSI_VALUE) - 157;
  last_snr_ = (int8_t)read_register(REG_PKT_SNR_VALUE) / 4;
  return ESP_OK;
}