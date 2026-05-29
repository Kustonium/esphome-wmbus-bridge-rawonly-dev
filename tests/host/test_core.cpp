#include <algorithm>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "decode3of6.h"
#include "dll_crc.h"
#include "packet.h"

using esphome::wmbus_common::crc16_en13757;
using esphome::wmbus_common::trim_dll_crc_format_a;
using esphome::wmbus_common::trim_dll_crc_format_b;
using esphome::wmbus_radio::Decode3of6Stats;
using esphome::wmbus_radio::Frame;
using esphome::wmbus_radio::LinkMode;
using esphome::wmbus_radio::Packet;
using esphome::wmbus_radio::decode3of6;
using esphome::wmbus_radio::encoded_size;

static int failures = 0;

static void check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    failures++;
  }
}

static void check(bool condition, const std::string &message) {
  check(condition, message.c_str());
}

struct GoldenFrameFixture {
  const char *name;
  const char *hex;
  uint32_t expected_meter_id;
};

static std::vector<uint8_t> encode_3of6(const std::vector<uint8_t> &decoded) {
  static const uint8_t lookup[16] = {
      0b010110, 0b001101, 0b001110, 0b001011,
      0b011100, 0b011001, 0b011010, 0b010011,
      0b101100, 0b100101, 0b100110, 0b100011,
      0b110100, 0b110001, 0b110010, 0b101001,
  };

  std::vector<uint8_t> out;
  uint8_t current = 0;
  uint8_t bits = 0;

  auto push_bit = [&](uint8_t bit) {
    current = (uint8_t) ((current << 1) | (bit & 0x01));
    bits++;
    if (bits == 8) {
      out.push_back(current);
      current = 0;
      bits = 0;
    }
  };

  for (uint8_t byte : decoded) {
    const uint8_t nibbles[2] = {
        (uint8_t) ((byte >> 4) & 0x0F),
        (uint8_t) (byte & 0x0F),
    };
    for (uint8_t nibble : nibbles) {
      uint8_t code = lookup[nibble];
      for (int bit = 5; bit >= 0; bit--) {
        push_bit((uint8_t) ((code >> bit) & 0x01));
      }
    }
  }

  if (bits != 0) {
    current = (uint8_t) (current << (8 - bits));
    out.push_back(current);
  }

  return out;
}

static void append_crc(std::vector<uint8_t> &payload, size_t pos, size_t len) {
  const uint16_t crc = crc16_en13757(payload.data() + pos, len);
  payload.push_back((uint8_t) ((crc >> 8) & 0xFF));
  payload.push_back((uint8_t) (crc & 0xFF));
}

static uint8_t hex_nibble(char c) {
  if (c >= '0' && c <= '9') return (uint8_t) (c - '0');
  if (c >= 'a' && c <= 'f') return (uint8_t) (10 + c - 'a');
  if (c >= 'A' && c <= 'F') return (uint8_t) (10 + c - 'A');
  check(false, "hex fixture contains an invalid digit");
  return 0;
}

static std::vector<uint8_t> hex_to_bytes(const std::string &hex) {
  check((hex.size() % 2) == 0, "hex fixture has an even number of digits");
  std::vector<uint8_t> out;
  out.reserve(hex.size() / 2);
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back((uint8_t) ((hex_nibble(hex[i]) << 4) | hex_nibble(hex[i + 1])));
  }
  return out;
}

static Packet make_packet(const std::vector<uint8_t> &raw, int8_t rssi,
                          std::optional<LinkMode> forced = std::nullopt) {
  Packet packet;
  auto *dst = packet.append_space(raw.size());
  std::copy(raw.begin(), raw.end(), dst);
  packet.set_rssi(rssi);
  if (forced) packet.set_forced_link_mode(*forced);
  return packet;
}

static std::vector<uint8_t> encode_s1_manchester(const std::vector<uint8_t> &decoded) {
  std::vector<uint8_t> out;
  uint8_t current = 0;
  uint8_t bits = 0;

  auto push_bit = [&](uint8_t bit) {
    current = (uint8_t) ((current << 1) | (bit & 0x01));
    bits++;
    if (bits == 8) {
      out.push_back(current);
      current = 0;
      bits = 0;
    }
  };

  for (uint8_t byte : decoded) {
    for (int bit = 7; bit >= 0; bit--) {
      const bool one = ((byte >> bit) & 0x01) != 0;
      // Parser polarity=false expects 01 -> 0 and 10 -> 1.
      push_bit(one ? 1 : 0);
      push_bit(one ? 0 : 1);
    }
  }

  if (bits != 0) {
    current = (uint8_t) (current << (8 - bits));
    out.push_back(current);
  }

  return out;
}

static std::vector<uint8_t> base_frame_body() {
  return {
      0x0B,  // L-field: 11 bytes follow, 12 bytes total including L-field.
      0x44, 0x12, 0x34, 0x56,
      0x78, 0x90, 0x01, 0x02,
      0x7A, 0xAA, 0x55,
  };
}

static std::vector<uint8_t> format_a_with_crc_from_body(const std::vector<uint8_t> &body) {
  check(body.size() >= 10, "Format A fixture has a first 10-byte block");
  std::vector<uint8_t> payload(body.begin(), body.begin() + 10);
  append_crc(payload, 0, 10);

  size_t offset = 10;
  while (body.size() - offset > 16) {
    const size_t crc_pos = payload.size();
    payload.insert(payload.end(), body.begin() + offset, body.begin() + offset + 16);
    append_crc(payload, crc_pos, 16);
    offset += 16;
  }

  if (offset < body.size()) {
    const size_t len = body.size() - offset;
    const size_t crc_pos = payload.size();
    payload.insert(payload.end(), body.begin() + offset, body.end());
    append_crc(payload, crc_pos, len);
  }

  return payload;
}

static std::vector<uint8_t> format_a_with_crc() {
  return format_a_with_crc_from_body(base_frame_body());
}

static std::vector<GoldenFrameFixture> golden_frame_fixtures() {
  return {
      {
          "long",
          "8e44b3380799080003027a09008005d518489e938d7741372ca5f34153b1f81e0e7f71fc2ce87ac30edb358dcd2b"
          "4731ecd84f7e705bb3f425aaa0a6df7f654a87b9289d49d7ad83cd2776a0627b6d528cb602da1b6455efca61"
          "f42dbc25387f1f6acdcbf633a5fde8e0b0e4f9153499cb94f3e64229969d9baaac567112988f3ba86728747"
          "b0dc9590d7a5afde493",
          89907,
      },
      {
          "short",
          "2f446850791255417462a2069f333904d00b09000000090000000000000000000002121113190b130400000000000100",
          41551279,
      },
      {
          "mid",
          "374468508107839027c3a2129f335c5600289e020000800e0000000030c0050f6cc1c7144b6921c12a0748e000000000000000000000c001",
          90830781,
      },
  };
}

static void test_decode3of6_round_trip() {
  const std::vector<uint8_t> decoded = {0x00, 0x12, 0xAB, 0xFF, 0x5C};
  auto coded = encode_3of6(decoded);
  Decode3of6Stats stats;
  auto actual = decode3of6(coded, &stats);

  check(actual.has_value(), "3-of-6 valid payload decodes");
  check(actual.value_or(std::vector<uint8_t>{}) == decoded, "3-of-6 round trip preserves bytes");
  check(stats.symbols_total == decoded.size() * 2, "3-of-6 stats count two symbols per byte");
  check(stats.symbols_invalid == 0, "3-of-6 stats report no invalid symbols");

  check(encoded_size(0) == 0, "encoded_size(0)");
  check(encoded_size(1) == 2, "encoded_size(1)");
  check(encoded_size(2) == 3, "encoded_size(2)");
  check(encoded_size(3) == 5, "encoded_size(3)");
}

static void test_decode3of6_invalid_symbol() {
  std::vector<uint8_t> coded = {0xFF, 0xFF};
  Decode3of6Stats stats;
  auto actual = decode3of6(coded, &stats);

  check(!actual.has_value(), "3-of-6 invalid payload is rejected");
  check(stats.symbols_invalid > 0, "3-of-6 invalid stats are populated");
}

static void test_dll_crc_format_a() {
  auto payload = format_a_with_crc();
  auto expected = base_frame_body();
  check(trim_dll_crc_format_a(payload), "Format A DLL CRC validates");
  check(payload == expected, "Format A DLL CRC is stripped");

  auto bad = format_a_with_crc();
  bad.back() ^= 0x01;
  check(!trim_dll_crc_format_a(bad), "Format A bad DLL CRC is rejected");
}

static void test_dll_crc_format_b() {
  auto payload = base_frame_body();
  append_crc(payload, 0, payload.size());
  auto expected = base_frame_body();

  check(trim_dll_crc_format_b(payload), "Format B DLL CRC validates");
  check(payload == expected, "Format B DLL CRC is stripped");

  auto bad = base_frame_body();
  append_crc(bad, 0, bad.size());
  bad[bad.size() - 2] ^= 0x01;
  check(!trim_dll_crc_format_b(bad), "Format B bad DLL CRC is rejected");
}

static void test_packet_t1_frame_conversion() {
  auto raw = encode_3of6(format_a_with_crc());
  Packet packet = make_packet(raw, -71);

  auto frame = packet.convert_to_frame();
  check(frame.has_value(), "T1 raw packet converts to frame");
  if (!frame) return;
  check(frame->link_mode() == LinkMode::T1, "T1 frame mode is retained");
  check(frame->rssi() == -71, "T1 frame RSSI is retained");
  check(frame->as_raw() == base_frame_body(), "T1 frame strips coding and DLL CRC");

  uint32_t meter_id = 0;
  check(frame->try_get_meter_id(meter_id), "T1 frame meter id is extractable");
  check(meter_id == 1907856, "T1 frame meter id value");
}

static void test_packet_c1_frame_conversion() {
  auto payload = format_a_with_crc();
  std::vector<uint8_t> raw = {0x54, 0xCD};
  raw.insert(raw.end(), payload.begin(), payload.end());

  Packet packet = make_packet(raw, -65);

  auto frame = packet.convert_to_frame();
  check(frame.has_value(), "C1 raw packet converts to frame");
  if (!frame) return;
  check(frame->link_mode() == LinkMode::C1, "C1 frame mode is retained");
  check(frame->rssi() == -65, "C1 frame RSSI is retained");
  check(frame->format() == "A", "C1 frame format A is detected");
  check(frame->as_raw() == base_frame_body(), "C1 frame strips suffix and DLL CRC");
}

static void test_packet_s1_frame_conversion() {
  auto raw = encode_s1_manchester(format_a_with_crc());
  Packet packet = make_packet(raw, -83, LinkMode::S1);

  auto frame = packet.convert_to_frame();
  check(frame.has_value(), "S1 Manchester raw packet converts to frame");
  if (!frame) return;
  check(frame->link_mode() == LinkMode::S1, "S1 frame mode is retained");
  check(frame->rssi() == -83, "S1 frame RSSI is retained");
  check(frame->format() == "A", "S1 frame format A is detected");
  check(frame->as_raw() == base_frame_body(), "S1 frame strips Manchester coding and DLL CRC");
}

static void test_packet_t1_bad_crc_is_rejected() {
  auto payload = format_a_with_crc();
  payload.back() ^= 0x01;
  Packet packet = make_packet(encode_3of6(payload), -72);

  auto frame = packet.convert_to_frame();
  check(!frame.has_value(), "T1 packet with bad DLL CRC is rejected");
  check(packet.drop_reason() == "dll_crc_failed", "T1 bad CRC reports dll_crc_failed");
  check(packet.drop_stage() == "dll_crc_final", "T1 bad final CRC reports dll_crc_final");
}

static void test_packet_t1_truncated_is_rejected() {
  auto payload = format_a_with_crc();
  payload.pop_back();
  Packet packet = make_packet(encode_3of6(payload), -74);

  auto frame = packet.convert_to_frame();
  check(!frame.has_value(), "T1 truncated packet is rejected");
  check(packet.is_truncated(), "T1 truncated packet sets truncated flag");
  check(packet.drop_reason() == "truncated", "T1 truncated packet reports truncated reason");
  check(packet.drop_stage() == "t1_length_check", "T1 truncated packet reports length-check stage");
}

static void test_packet_c1_unknown_preamble_is_rejected() {
  auto payload = format_a_with_crc();
  std::vector<uint8_t> raw = {0x54, 0x00};
  raw.insert(raw.end(), payload.begin(), payload.end());
  Packet packet = make_packet(raw, -67);

  auto frame = packet.convert_to_frame();
  check(!frame.has_value(), "C1 packet with unknown block preamble is rejected");
  check(packet.drop_stage() == "c1_preamble", "C1 bad preamble reports c1_preamble stage");
  check(packet.drop_reason() == "unknown_preamble", "C1 bad preamble reports unknown_preamble");
}

static void test_packet_s1_invalid_manchester_is_rejected() {
  std::vector<uint8_t> raw(8, 0x00);
  Packet packet = make_packet(raw, -90, LinkMode::S1);

  auto frame = packet.convert_to_frame();
  check(!frame.has_value(), "S1 invalid Manchester packet is rejected");
  check(packet.drop_stage() == "s1_l_field", "S1 invalid Manchester reports S1 l-field stage");
  check(packet.drop_reason() == "l_field_invalid", "S1 invalid Manchester reports invalid L-field");
  check(packet.t1_symbols_invalid() > 0, "S1 invalid Manchester records invalid symbols");
}

static void check_golden_body_shape(const std::vector<uint8_t> &body,
                                    const GoldenFrameFixture &fixture) {
  const std::string prefix = std::string(fixture.name) + " golden frame";
  check(!body.empty(), prefix + " is present");
  if (body.empty()) return;
  check(body.size() == (size_t) body[0] + 1, prefix + " L-field matches body length");

  Packet packet = make_packet(body, -60);
  uint32_t meter_id = 0;
  check(packet.try_get_meter_id(meter_id), prefix + " meter id is extractable");
  check(meter_id == fixture.expected_meter_id, prefix + " meter id matches expected value");
}

static void check_golden_round_trip_c1(const std::vector<uint8_t> &body, const char *name) {
  const std::string prefix = std::string(name) + " golden C1 frame";
  auto payload = format_a_with_crc_from_body(body);
  std::vector<uint8_t> raw = {0x54, 0xCD};
  raw.insert(raw.end(), payload.begin(), payload.end());
  Packet packet = make_packet(raw, -68);

  auto frame = packet.convert_to_frame();
  check(frame.has_value(), prefix + " converts through parser");
  if (!frame) return;
  check(frame->link_mode() == LinkMode::C1, prefix + " keeps C1 mode");
  check(frame->format() == "A", prefix + " uses Format A fixture");
  check(frame->as_raw() == body, prefix + " round-trips to normalized body");
}

static void check_golden_round_trip_t1(const std::vector<uint8_t> &body, const char *name) {
  const std::string prefix = std::string(name) + " golden T1 frame";
  auto raw = encode_3of6(format_a_with_crc_from_body(body));
  Packet packet = make_packet(raw, -73);

  auto frame = packet.convert_to_frame();
  check(frame.has_value(), prefix + " converts through parser");
  if (!frame) return;
  check(frame->link_mode() == LinkMode::T1, prefix + " keeps T1 mode");
  check(frame->format() == "A", prefix + " uses Format A fixture");
  check(frame->as_raw() == body, prefix + " round-trips to normalized body");
}

static void test_real_golden_frames_round_trip() {
  for (const auto &fixture : golden_frame_fixtures()) {
    const auto body = hex_to_bytes(fixture.hex);
    check_golden_body_shape(body, fixture);
    check_golden_round_trip_c1(body, fixture.name);
    check_golden_round_trip_t1(body, fixture.name);
  }
}

int main() {
  test_decode3of6_round_trip();
  test_decode3of6_invalid_symbol();
  test_dll_crc_format_a();
  test_dll_crc_format_b();
  test_packet_t1_frame_conversion();
  test_packet_c1_frame_conversion();
  test_packet_s1_frame_conversion();
  test_packet_t1_bad_crc_is_rejected();
  test_packet_t1_truncated_is_rejected();
  test_packet_c1_unknown_preamble_is_rejected();
  test_packet_s1_invalid_manchester_is_rejected();
  test_real_golden_frames_round_trip();

  if (failures == 0) {
    std::cout << "All host parser tests passed\n";
  }
  return failures == 0 ? 0 : 1;
}
