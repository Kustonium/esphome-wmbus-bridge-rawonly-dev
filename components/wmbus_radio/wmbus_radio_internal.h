// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Shared protocol constants for the wmbus_radio component. These were defined at
// the top of component.cpp; they are collected here so the translation units
// component.cpp was split into (rf_runtime, ...) can share them without
// duplication. Move-only refactor: values are unchanged.

#define WMBUS_PREAMBLE_SIZE (3)
#define WMBUS_MODE_C_PREAMBLE (0x54)
#define WMBUS_T1_LEN_PROBE_BYTES (18)
// SX1276-specific recovery path: if length cannot be derived from the initial
// probe, keep draining the raw stream until idle and let the packet parser make
// the final decision. Sized to cover long T1 telegrams (>255 B after decode).
#define WMBUS_RAW_DRAIN_MAX_BYTES (416)
