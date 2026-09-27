#pragma once
#include <cstdint>
#include <cstddef>

namespace packet
{
  constexpr uint8_t PROTOCOL_VERSION = 1;
  constexpr uint8_t MAGIC_BYTE = 0x67;

  enum class click_t : uint8_t
  {
    SINGLE = 1,
    DOUBLE = 2
  };

  struct Action
  {
    uint32_t no;
    int64_t timestamp_us;
    click_t event;
  };

  struct __attribute__((packed)) Msg
  {
    uint8_t version;
    uint8_t magic;
    uint32_t no;
    int64_t timestamp_us;
    packet::click_t event;
    uint8_t reserved[17];
  };

  constexpr size_t SIZE = sizeof(Msg);
  static_assert(sizeof(Msg) == 32, "on air packet must be 32 bytes long");

  void buildPacket(const Action &in, uint8_t *out);
  bool parsePacket(const uint8_t *data, size_t len, Action &out);
}
