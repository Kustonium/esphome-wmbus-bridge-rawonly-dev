// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace esphome {
namespace wmbus_radio {

// Whitelist decision for RAW telegram forwarding (`forward_meters`).
//
// Deliberately free of ESPHome/FreeRTOS includes so tests/host can link it
// without the component runtime.
//
// `allowed_ids` must be sorted and deduplicated - parse_meter_id_csv_ does both.
// An empty list means no filter is configured and everything is forwarded, which
// is the behaviour that predates this option.
//
// meter_id == 0 means the ID could not be decoded from the A-field (non-BCD).
// With a filter configured such frames are rejected: forwarding unidentified
// frames would defeat the point of the whitelist.
inline bool meter_id_allowed(const std::vector<uint32_t> &allowed_ids, uint32_t meter_id) {
  if (allowed_ids.empty()) return true;
  if (meter_id == 0) return false;
  return std::binary_search(allowed_ids.begin(), allowed_ids.end(), meter_id);
}

}  // namespace wmbus_radio
}  // namespace esphome
