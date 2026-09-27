#include "sx127x.h"
#include <freertos/FreeRTOS.h>
#include <type_traits>
#include <algorithm>

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

template <typename E>
static constexpr uint8_t raw(E e)
{
  return static_cast<std::underlying_type_t<E>>(e);
}

static uint32_t bw_hz(Sx127x::Bw bw)
{
  static constexpr uint32_t table[] = {7800, 10400, 15600, 20800, 31250,
                                       41700, 62500, 125000, 250000, 500000};

  return table[raw(bw) >> 4];
}

static uint32_t get_sym_time_us(Sx127x::Sf sf, Sx127x::Bw bw)
{
  // T_sym = 2^SF / BW
  uint32_t sym_us = (1'000'000ULL << (raw(sf) >> 4)) / bw_hz(bw);
  return sym_us;
}

// See: Low Data Rate Optimization
static bool needs_ldro(Sx127x::Sf sf, Sx127x::Bw bw)
{
  return get_sym_time_us(sf, bw) > 16000;
}

static void IRAM_ATTR dio0_isr(void *arg)
{
  auto *self = static_cast<Sx127x *>(arg);
  BaseType_t hp = pdFALSE;
  xSemaphoreGiveFromISR(self->irq_, &hp);
  portYIELD_FROM_ISR(hp);
}

esp_err_t Sx127x::configure(const Config &cfg)
{
  write_register(REG_OP_MODE, MODE_LORA | MODE_SLEEP);
  uint64_t frf = ((uint64_t)cfg.freq_hz << 19) / 32000000;
  write_register(REG_FRF_MSB, (frf >> 16) & 0xFF);
  write_register(REG_FRF_MID, (frf >> 8) & 0xFF);
  write_register(REG_FRF_LSB, frf & 0xFF);

  write_register(REG_FIFO_TX_BASE, 0x00);
  write_register(REG_FIFO_RX_BASE, 0x00);
  write_register(REG_LNA, 0x23);

  // 0x1D: BW | CR | explicit header (bit0 = 0)
  write_register(REG_MODEM_CONFIG_1, raw(cfg.bw) | raw(cfg.cr));

  // 0x1E: SF | TxContinuous=0 | CrcOn | SymbTimeout MSB=0
  write_register(REG_MODEM_CONFIG_2, raw(cfg.sf) | (cfg.crc_on ? 0x04 : 0x00));

  // 0x26: LDRO (bit3) | AgcAutoOn (bit2)
  uint8_t mc3 = 0x04;
  if (needs_ldro(cfg.sf, cfg.bw))
  {
    mc3 |= 0x08;
  }
  write_register(REG_MODEM_CONFIG_3, mc3);

  write_register(REG_PREAMBLE_MSB, cfg.preamble >> 8);
  write_register(REG_PREAMBLE_LSB, cfg.preamble & 0xFF);
  write_register(REG_SYNC_WORD, cfg.sync_word);
  write_register(REG_PA_CONFIG, 0x8F);

  write_register(REG_OP_MODE, MODE_LORA | MODE_STDBY);

  cfg_ = cfg;
  return ESP_OK;
}

uint32_t Sx127x::time_on_air_ms(uint8_t len) const
{
  const uint32_t t_sym_us = get_sym_time_us(cfg_.sf, cfg_.bw);
  const int sf = raw(cfg_.sf) >> 4;
  const int cr = raw(cfg_.cr) >> 1; // 1..4
  const int crc = cfg_.crc_on ? 1 : 0;
  const int ih = 0; // explicit header only, TODO: add if adding SF6 support
  const int de = needs_ldro(cfg_.sf, cfg_.bw) ? 1 : 0;

  // T_preamble = (n_preamble + 4.25) * T_sym  -> keep integer by scaling x4
  const uint64_t t_preamble_us = (uint64_t)(4 * cfg_.preamble + 17) * t_sym_us / 4;

  // n_payload = 8 + max(ceil((8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*(SF - 2*DE))) * (CR + 4), 0)
  const int num = 8 * len - 4 * sf + 28 + 16 * crc - 20 * ih;
  const int den = 4 * (sf - 2 * de);
  int blocks = 0;
  if (num > 0)
    blocks = (num + den - 1) / den; // ceil
  const uint32_t n_payload = 8 + blocks * (cr + 4);

  const uint64_t t_packet_us = t_preamble_us + (uint64_t)n_payload * t_sym_us;
  return (t_packet_us + 999) / 1000; // ceil to ms
}

esp_err_t Sx127x::begin(gpio_num_t cs, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso, gpio_num_t rst, gpio_num_t dio0)
{
  esp_err_t err = gpio_install_isr_service(0);

  // handling ESP_ERR_INVALID_STATE, since this means that isr service already installed
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    return err;
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

  // expected airtime + 50% margin + 50 ms for ISR/tick latency
  timeout_ms_ = time_on_air_ms(len) * 3 / 2 + 50;

  xSemaphoreTake(irq_, 0);
  write_register(REG_OP_MODE, MODE_LORA | MODE_TX);

  if (xSemaphoreTake(irq_, pdMS_TO_TICKS(timeout_ms_)) != pdTRUE)
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