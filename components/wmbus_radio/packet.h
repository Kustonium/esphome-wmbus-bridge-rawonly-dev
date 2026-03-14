#pragma once
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <optional>
#include <string>
#include <vector>

#include "internal_wmbus.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace wmbus_radio {
struct Frame;

struct Packet {
  friend class Frame;

public:
  Packet();

  // Reserve/extend internal buffer and return pointer to the newly appended
  // region. The returned memory is valid until the next reallocation.
  uint8_t *append_space(size_t len);

  // Expected total packet size (including PHY header bytes as provided by
  // the transceiver). Returns 0 if it can't be determined from current data.
  size_t expected_size();

  void set_rssi(int8_t rssi);

  // Convert raw radio bytes into a validated wM-Bus frame.
  // The implementation now tries a small set of parser candidates (T1/C1,
  // format A/B) before giving up, instead of trusting a single hard mode
  // guess.
  std::optional<Frame> convert_to_frame();

protected:
  std::vector<uint8_t> data_;

  size_t expected_size_ = 0;

  uint8_t l_field();
  int8_t rssi_ = 0;

  LinkMode link_mode();
  LinkMode link_mode_ = LinkMode::UNKNOWN;

  std::string frame_format_;
};

struct Frame {
public:
  Frame(Packet *packet);

  std::vector<uint8_t> &data();
  LinkMode link_mode();
  int8_t rssi();
  std::string format();

  std::vector<uint8_t> as_raw();
  std::string as_hex();
  std::string as_rtlwmbus();

  void mark_as_handled();
  uint8_t handlers_count();

protected:
  std::vector<uint8_t> data_;
  LinkMode link_mode_;
  int8_t rssi_;
  std::string format_;
  uint8_t handlers_count_ = 0;
};

} // namespace wmbus_radio
} // namespace esphome
