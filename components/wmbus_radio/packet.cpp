#include "packet.h"

#include <algorithm>
#include <ctime>
#include <utility>
#include <vector>

#include "internal_wmbus.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "decode3of6.h"
#include "dll_crc.h"

#define WMBUS_PREAMBLE_SIZE (3)
#define WMBUS_MODE_C_SUFIX_LEN (2)
#define WMBUS_MODE_C_PREAMBLE (0x54)
#define WMBUS_BLOCK_A_PREAMBLE (0xCD)
#define WMBUS_BLOCK_B_PREAMBLE (0x3D)

namespace esphome {
namespace wmbus_radio {
static const char *TAG = "packet";

namespace {
// ParserPath documents which candidate path was tried. We keep this local to
// packet.cpp because it is only used to select the best parsing strategy.
enum class ParserPath {
  T1_FORMAT_A,
  T1_FORMAT_B,
  C1_FORMAT_A,
  C1_FORMAT_B,
};

const char *parser_path_name(ParserPath path) {
  switch (path) {
  case ParserPath::T1_FORMAT_A:
    return "T1/A";
  case ParserPath::T1_FORMAT_B:
    return "T1/B";
  case ParserPath::C1_FORMAT_A:
    return "C1/A";
  case ParserPath::C1_FORMAT_B:
    return "C1/B";
  default:
    return "?";
  }
}

struct ParseAttempt {
  ParserPath path;
  bool ok{false};
  LinkMode link_mode{LinkMode::UNKNOWN};
  std::string frame_format;
  std::vector<uint8_t> frame_data;

  // Lightweight diagnostics used only for verbose logs. We do not expose them
  // outside the parser to keep the public API unchanged.
  size_t raw_size{0};
  size_t expected_size{0};
  const char *fail_stage{"not_tried"};
  std::string fail_detail;
};

// Number of DLL blocks for a given L-field.
static inline size_t blocks_for_l_(uint8_t l_field) {
  return (l_field < 26) ? 2 : (size_t) ((l_field - 26) / 16 + 3);
}

// In Frame Format A the L-field excludes the CRC bytes.
static inline size_t total_len_format_a_with_crc_(uint8_t l_field) {
  return (size_t) l_field + 1 + 2 * blocks_for_l_(l_field);
}

// In Frame Format B the L-field already includes the CRC bytes.
static inline size_t total_len_format_b_with_crc_(uint8_t l_field) {
  return (size_t) l_field + 1;
}

// Sanity range used after we have an L-field candidate. We keep it generous on
// purpose: the precheck should reject obvious garbage, not valid edge cases.
static inline bool l_field_is_sane_(uint8_t l_field) {
  return l_field >= 9 && l_field <= 250;
}

// Best-effort initial guess used only to decide parsing order. The parser is no
// longer hard-bound to this decision: if the first choice fails, we try a small
// set of fallbacks before giving up.
ParserPath initial_parser_guess_(const std::vector<uint8_t> &raw) {
  if (raw.size() >= 2 && raw[0] == WMBUS_MODE_C_PREAMBLE) {
    if (raw[1] == WMBUS_BLOCK_B_PREAMBLE)
      return ParserPath::C1_FORMAT_B;
    return ParserPath::C1_FORMAT_A;
  }
  return ParserPath::T1_FORMAT_A;
}

std::vector<ParserPath> parser_attempt_order_(ParserPath guess) {
  switch (guess) {
  case ParserPath::C1_FORMAT_A:
    return {ParserPath::C1_FORMAT_A, ParserPath::C1_FORMAT_B,
            ParserPath::T1_FORMAT_A, ParserPath::T1_FORMAT_B};
  case ParserPath::C1_FORMAT_B:
    return {ParserPath::C1_FORMAT_B, ParserPath::C1_FORMAT_A,
            ParserPath::T1_FORMAT_A, ParserPath::T1_FORMAT_B};
  case ParserPath::T1_FORMAT_B:
    return {ParserPath::T1_FORMAT_B, ParserPath::T1_FORMAT_A,
            ParserPath::C1_FORMAT_A, ParserPath::C1_FORMAT_B};
  case ParserPath::T1_FORMAT_A:
  default:
    return {ParserPath::T1_FORMAT_A, ParserPath::T1_FORMAT_B,
            ParserPath::C1_FORMAT_A, ParserPath::C1_FORMAT_B};
  }
}

bool frame_passes_wmbus_validation_(std::vector<uint8_t> payload) {
  int dummy;
  return checkWMBusFrame(payload, (size_t *) &dummy, &dummy, &dummy, false) ==
         FrameStatus::FullFrame;
}

// Attempt to parse a packet as T1. We support both DLL frame formats. The main
// improvement over the previous code is that we no longer assume that the first
// successful mode guess is the only possible one.
ParseAttempt try_parse_t1_(const std::vector<uint8_t> &raw, bool format_b) {
  ParseAttempt out;
  out.path = format_b ? ParserPath::T1_FORMAT_B : ParserPath::T1_FORMAT_A;
  out.raw_size = raw.size();
  out.link_mode = LinkMode::T1;
  out.frame_format = format_b ? "B" : "A";

  // Soft precheck: we only require enough bytes to decode the first decoded
  // byte (L-field). The old code was much stricter and could reject borderline
  // but otherwise valid packets before we even looked at L.
  if (raw.size() < 3) {
    out.fail_stage = "soft_precheck";
    out.fail_detail = "need at least 3 encoded bytes to decode T1 L-field";
    return out;
  }

  std::vector<uint8_t> l_field_prefix(raw.begin(), raw.begin() + 3);
  auto decoded_prefix = decode3of6(l_field_prefix);
  if (!decoded_prefix || decoded_prefix->empty()) {
    out.fail_stage = "t1_l_field_decode";
    out.fail_detail = "failed to decode T1 L-field from 3-of-6 prefix";
    return out;
  }

  const uint8_t l_field = decoded_prefix->at(0);
  if (!l_field_is_sane_(l_field)) {
    out.fail_stage = "t1_l_field_sanity";
    out.fail_detail = str_sprintf("L=%u outside sane range", l_field);
    return out;
  }

  const size_t decoded_len_with_crc =
      format_b ? total_len_format_b_with_crc_(l_field)
               : total_len_format_a_with_crc_(l_field);
  const size_t expected_encoded = encoded_size(decoded_len_with_crc);
  out.expected_size = expected_encoded;

  if (raw.size() < expected_encoded) {
    out.fail_stage = "t1_length_check";
    out.fail_detail =
        str_sprintf("need %zu encoded bytes, have %zu", expected_encoded, raw.size());
    return out;
  }

  std::vector<uint8_t> encoded_candidate(raw.begin(), raw.begin() + expected_encoded);
  auto decoded_full = decode3of6(encoded_candidate);
  if (!decoded_full) {
    out.fail_stage = "t1_decode3of6";
    out.fail_detail = "full 3-of-6 decode failed";
    return out;
  }

  std::vector<uint8_t> payload = std::move(decoded_full.value());
  DLLCRCResult crc_diag;
  const bool crc_ok = format_b ? trim_dll_crc_format_b(payload, &crc_diag)
                               : trim_dll_crc_format_a(payload, &crc_diag);
  if (!crc_ok) {
    out.fail_stage = crc_diag.stage;
    out.fail_detail = str_sprintf(
        "DLL CRC failed (%s calc=%04x expected=%04x data_pos=%zu data_len=%zu)",
        crc_diag.format, crc_diag.calculated, crc_diag.expected, crc_diag.data_pos,
        crc_diag.data_len);
    return out;
  }

  if (!frame_passes_wmbus_validation_(payload)) {
    out.fail_stage = "checkWMBusFrame";
    out.fail_detail = "decoded T1 candidate did not validate as a full frame";
    return out;
  }

  out.ok = true;
  out.frame_data = std::move(payload);
  return out;
}

// Attempt to parse a packet as C1. We keep the old suffix trimming logic, but
// now treat A/B as cheap parser variants instead of a single hard-coded path.
ParseAttempt try_parse_c1_(const std::vector<uint8_t> &raw, bool format_b) {
  ParseAttempt out;
  out.path = format_b ? ParserPath::C1_FORMAT_B : ParserPath::C1_FORMAT_A;
  out.raw_size = raw.size();
  out.link_mode = LinkMode::C1;
  out.frame_format = format_b ? "B" : "A";

  if (raw.size() < 3) {
    out.fail_stage = "soft_precheck";
    out.fail_detail = "need at least 3 raw bytes to inspect C1 preamble/L-field";
    return out;
  }
  if (raw[0] != WMBUS_MODE_C_PREAMBLE) {
    out.fail_stage = "c1_preamble";
    out.fail_detail = str_sprintf("first byte 0x%02X is not C1 preamble 0x54", raw[0]);
    return out;
  }

  const uint8_t expected_block = format_b ? WMBUS_BLOCK_B_PREAMBLE : WMBUS_BLOCK_A_PREAMBLE;
  if (raw[1] != expected_block) {
    out.fail_stage = "c1_block_preamble";
    out.fail_detail =
        str_sprintf("second byte 0x%02X does not match expected block preamble 0x%02X",
                    raw[1], expected_block);
    return out;
  }

  std::vector<uint8_t> payload(raw.begin() + WMBUS_MODE_C_SUFIX_LEN, raw.end());
  if (payload.empty()) {
    out.fail_stage = "c1_suffix";
    out.fail_detail = "payload empty after removing C1 suffix bytes";
    return out;
  }

  const uint8_t l_field = payload[0];
  if (!l_field_is_sane_(l_field)) {
    out.fail_stage = "c1_l_field_sanity";
    out.fail_detail = str_sprintf("L=%u outside sane range", l_field);
    return out;
  }

  const size_t needed = format_b ? total_len_format_b_with_crc_(l_field)
                                 : total_len_format_a_with_crc_(l_field);
  out.expected_size = needed + WMBUS_MODE_C_SUFIX_LEN;
  if (payload.size() < needed) {
    out.fail_stage = "c1_length_check";
    out.fail_detail = str_sprintf("need %zu bytes after suffix, have %zu", needed,
                                  payload.size());
    return out;
  }

  payload.resize(needed);

  DLLCRCResult crc_diag;
  const bool crc_ok = format_b ? trim_dll_crc_format_b(payload, &crc_diag)
                               : trim_dll_crc_format_a(payload, &crc_diag);
  if (!crc_ok) {
    out.fail_stage = crc_diag.stage;
    out.fail_detail = str_sprintf(
        "DLL CRC failed (%s calc=%04x expected=%04x data_pos=%zu data_len=%zu)",
        crc_diag.format, crc_diag.calculated, crc_diag.expected, crc_diag.data_pos,
        crc_diag.data_len);
    return out;
  }

  if (!frame_passes_wmbus_validation_(payload)) {
    out.fail_stage = "checkWMBusFrame";
    out.fail_detail = "decoded C1 candidate did not validate as a full frame";
    return out;
  }

  out.ok = true;
  out.frame_data = std::move(payload);
  return out;
}

ParseAttempt try_parse_path_(const std::vector<uint8_t> &raw, ParserPath path) {
  switch (path) {
  case ParserPath::T1_FORMAT_A:
    return try_parse_t1_(raw, false);
  case ParserPath::T1_FORMAT_B:
    return try_parse_t1_(raw, true);
  case ParserPath::C1_FORMAT_A:
    return try_parse_c1_(raw, false);
  case ParserPath::C1_FORMAT_B:
    return try_parse_c1_(raw, true);
  default:
    ParseAttempt out;
    out.path = path;
    out.fail_stage = "internal";
    out.fail_detail = "unsupported parser path";
    return out;
  }
}
} // namespace

Packet::Packet() { this->data_.reserve(WMBUS_PREAMBLE_SIZE); }

// Determine the link mode based on the first byte of the data.
// NOTE: this is still only a best-effort guess and is intentionally kept cheap.
// convert_to_frame() now has a fallback parser path and no longer trusts this
// guess blindly.
LinkMode Packet::link_mode() {
  if (this->link_mode_ == LinkMode::UNKNOWN)
    if (this->data_.size())
      if (this->data_[0] == WMBUS_MODE_C_PREAMBLE)
        this->link_mode_ = LinkMode::C1;
      else
        this->link_mode_ = LinkMode::T1;

  return this->link_mode_;
}

void Packet::set_rssi(int8_t rssi) { this->rssi_ = rssi; }

// Get value of L-field.
// This helper is only used to estimate the total radio read size after the
// first bytes have arrived from the transceiver. We keep it intentionally
// simple; the more expensive multi-path parsing happens later in
// convert_to_frame().
uint8_t Packet::l_field() {
  switch (this->link_mode()) {
  case LinkMode::C1:
    if (this->data_.size() < 3)
      return 0;
    return this->data_[2];
  case LinkMode::T1: {
    const size_t n = std::min<size_t>(this->data_.size(), 3);
    std::vector<uint8_t> tmp(this->data_.begin(), this->data_.begin() + n);
    auto decoded = decode3of6(tmp);
    if (decoded)
      return (*decoded)[0];
  }
  }
  return 0;
}

size_t Packet::expected_size() {
  if (this->data_.size() < WMBUS_PREAMBLE_SIZE)
    return 0;
  if (!this->expected_size_) {
    auto l_field = this->l_field();
    if (l_field == 0)
      return 0;

    auto nrBlocks = l_field < 26 ? 2 : (l_field - 26) / 16 + 3;
    auto nrBytes = l_field + 1 + 2 * nrBlocks;

    if (this->link_mode() != LinkMode::C1) {
      this->expected_size_ = encoded_size(nrBytes);
    } else if (this->data_[1] == WMBUS_BLOCK_A_PREAMBLE) {
      this->expected_size_ = WMBUS_MODE_C_SUFIX_LEN + nrBytes;
    } else if (this->data_[1] == WMBUS_BLOCK_B_PREAMBLE) {
      this->expected_size_ = WMBUS_MODE_C_SUFIX_LEN + 1 + l_field;
    }
  }
  ESP_LOGV(TAG, "expected_size: %zu", this->expected_size_);
  return this->expected_size_;
}

uint8_t *Packet::append_space(size_t len) {
  const size_t old = this->data_.size();
  this->data_.resize(old + len);
  return this->data_.data() + old;
}

std::optional<Frame> Packet::convert_to_frame() {
  std::optional<Frame> frame = {};

  ESP_LOGD(TAG, "Have data from radio (%zu bytes)", this->data_.size());
  debugPayload("raw packet", this->data_);

  const ParserPath guess = initial_parser_guess_(this->data_);
  const auto attempts = parser_attempt_order_(guess);

  ParseAttempt best_failure;
  bool have_failure = false;

  // Try a small, deterministic set of parser candidates. This gives us three
  // practical benefits without changing the external API:
  //  1. we can fall back between T1 and C1 if the initial guess was wrong,
  //  2. we can try both DLL frame formats A/B,
  //  3. we stop treating one early parser decision as an irreversible truth.
  for (ParserPath path : attempts) {
    auto attempt = try_parse_path_(this->data_, path);
    if (attempt.ok) {
      if (path != guess) {
        ESP_LOGI(TAG, "Accepted packet via fallback path %s (initial guess was %s)",
                 parser_path_name(path), parser_path_name(guess));
      } else {
        ESP_LOGV(TAG, "Accepted packet via %s", parser_path_name(path));
      }

      this->data_ = std::move(attempt.frame_data);
      this->link_mode_ = attempt.link_mode;
      this->frame_format_ = attempt.frame_format;
      frame.emplace(this);
      delete this;
      return frame;
    }

    ESP_LOGV(TAG, "Parser path %s rejected packet at stage %s (%s)",
             parser_path_name(path), attempt.fail_stage,
             attempt.fail_detail.c_str());

    if (!have_failure) {
      // Preserve the first failure as a compact "why nothing matched" summary.
      best_failure = attempt;
      have_failure = true;
    }
  }

  if (have_failure) {
    ESP_LOGD(TAG, "All parser paths rejected packet; first failure: %s at %s (%s)",
             parser_path_name(best_failure.path), best_failure.fail_stage,
             best_failure.fail_detail.c_str());
  } else {
    ESP_LOGD(TAG, "All parser paths rejected packet");
  }

  delete this;
  return frame;
}

Frame::Frame(Packet *packet)
    : data_(std::move(packet->data_)), link_mode_(packet->link_mode_),
      rssi_(packet->rssi_), format_(packet->frame_format_) {}

std::vector<uint8_t> &Frame::data() { return this->data_; }
LinkMode Frame::link_mode() { return this->link_mode_; }
int8_t Frame::rssi() { return this->rssi_; }
std::string Frame::format() { return this->format_; }

std::vector<uint8_t> Frame::as_raw() { return this->data_; }
std::string Frame::as_hex() { return format_hex(this->data_); }
std::string Frame::as_rtlwmbus() {
  const size_t time_repr_size = sizeof("YYYY-MM-DD HH:MM:SS.00Z");
  char time_buffer[time_repr_size];
  auto t = std::time(NULL);
  std::strftime(time_buffer, time_repr_size, "%F %T.00Z", std::gmtime(&t));

  auto output = std::string{};
  output.reserve(2 + 5 + 24 + 1 + 4 + 5 + 2 * this->data_.size() + 1);

  output += linkModeName(this->link_mode_); // size 2
  output += ";1;1;";                        // size 5
  output += time_buffer;                    // size 24
  output += ';';                            // size 1
  output += std::to_string(this->rssi_);    // size up to 4
  output += ";;;0x";                        // size 5
  output += this->as_hex();                 // size 2 * frame.size()
  output += "\n";                           // size 1

  return output;
}

void Frame::mark_as_handled() { this->handlers_count_++; }
uint8_t Frame::handlers_count() { return this->handlers_count_; }

} // namespace wmbus_radio
} // namespace esphome
