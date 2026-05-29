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

static std::vector<uint8_t> base_frame_body() {
  return {
      0x09,  // L-field: 9 bytes follow, 10 bytes total including L-field.
      0x44, 0x12, 0x34, 0x56,
      0x78, 0x90, 0x01, 0x02,
      0x7A,
  };
}

static std::vector<uint8_t> format_a_with_crc() {
  auto payload = base_frame_body();
  append_crc(payload, 0, 10);
  return payload;
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
  Packet packet;
  auto *dst = packet.append_space(raw.size());
  std::copy(raw.begin(), raw.end(), dst);
  packet.set_rssi(-71);

  auto frame = packet.convert_to_frame();
  check(frame.has_value(), "T1 raw packet converts to frame");
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

  Packet packet;
  auto *dst = packet.append_space(raw.size());
  std::copy(raw.begin(), raw.end(), dst);
  packet.set_rssi(-65);

  auto frame = packet.convert_to_frame();
  check(frame.has_value(), "C1 raw packet converts to frame");
  check(frame->link_mode() == LinkMode::C1, "C1 frame mode is retained");
  check(frame->rssi() == -65, "C1 frame RSSI is retained");
  check(frame->format() == "A", "C1 frame format A is detected");
  check(frame->as_raw() == base_frame_body(), "C1 frame strips suffix and DLL CRC");
}

int main() {
  test_decode3of6_round_trip();
  test_decode3of6_invalid_symbol();
  test_dll_crc_format_a();
  test_dll_crc_format_b();
  test_packet_t1_frame_conversion();
  test_packet_c1_frame_conversion();

  if (failures == 0) {
    std::cout << "All host parser tests passed\n";
  }
  return failures == 0 ? 0 : 1;
}
