#include "packet.h"
#include <cstring>

void packet::buildPacket(const Action &in, uint8_t *out)
{
  Msg m{};
  m.version = PROTOCOL_VERSION;
  m.magic = MAGIC_BYTE;
  m.event = in.event;
  m.no = in.no;
  m.timestamp_us = in.timestamp_us;

  memset(out, 0, SIZE);
  memcpy(out, &m, sizeof(m));
}

bool packet::parsePacket(const uint8_t *data, size_t len, packet::Action &out)
{
  if (len < sizeof(Msg))
  {
    return false;
  }

  Msg m{};
  memcpy(&m, data, sizeof(m));
  if (m.version != PROTOCOL_VERSION || m.magic != MAGIC_BYTE)
  {
    return false;
  }

  out.event = m.event;
  out.no = m.no;
  out.timestamp_us = m.timestamp_us;

  return true;
}