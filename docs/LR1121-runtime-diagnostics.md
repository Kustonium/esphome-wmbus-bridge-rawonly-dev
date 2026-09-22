# LR1121 runtime diagnostics (2026-09-06)

Diagnostic-only change; modulation, IRQ masks, BUSY timeout policy and RX restart
decisions are unchanged. Other radio drivers return no runtime diagnostic.

Every 60 seconds the main task logs a JSON snapshot and, when summary diagnostics
and MQTT are enabled, publishes it retained with QoS 1 to
`<diagnostic_topic>/radio_runtime`. Counters are cumulative since device boot.
Archive this topic during tests: retained preserves only the latest snapshot.
`uptime_ms` restarts on reboot and wraps after about 49 days.

- `busy_timeouts`: failed waits, including boot and direct-read waits.
- `status_samples`, `cmd_fail_observations`, `cmd_perr_observations`: sampled Stat1
  values, NOT unique failed commands. Consecutive transactions can report the same
  previous-command result. Invalid SPI responses can also look like CMD_FAIL.
- `stat1`, `stat2`, `chip_mode`, `reset_status`: last command/direct-read snapshot,
  not a live claim about the mode at publication time. ResetStatus is sticky;
  this patch does not clear it and cannot count separate resets.
- `irq_samples`, `rx_done_observations`, `timeout_observations`,
  `len_error_observations`, `read_without_rx_done`, `last_irq`: sampled before
  buffer reads. Repeated reads of an uncleared IRQ latch may be counted repeatedly.
  These are not preamble counts or numbers of on-air emissions.
- `packet_samples`, `packet_received_observations`, `packet_abort_observations`,
  `packet_length`, `packet_flags`: GetPacketStatus values already read by the
  existing RSSI path. Packet flags are raw; bit 1 = received, bit 2 = aborted.

Cross-task fields are atomic, but the complete JSON is not a single instantaneous
snapshot. Reading IRQ adds one direct SPI read and a BUSY wait per buffer-load
attempt. This is not a zero-overhead change: verify nominal reception before
comparing weak-signal results. No extra IRQ sources are enabled.

Sources: LR1121 User Manual rev 2.2, pp. 27-31, 38-39, 51, 80;
Semtech SWDR001 2.4.1, lr11xx_system.c and lr11xx_radio.c.

Known separate S1 issue: the existing SYNC_WORD_VALID constant incorrectly uses
bit 2 (TX_DONE); the documented bit is 5. Intentionally not switched here: enabling
the real early interrupt without fixing the receive dispatcher could abort packets.
This does not affect the T1 IRQ mask used in the attenuation experiment.

## Bounded FIFO and rejection samples

With summary diagnostics enabled, LR1121 also publishes retained QoS 1 messages:

- `<diagnostic_topic>/lr_pipeline`: cumulative main-task conversion counters,
  every 60 seconds. `converted = valid + decode_failed + length_failed +
  crc_failed + other_failed`. Counts precede the post-parse listen-mode filter.
  They do NOT include packets rejected in the receiver task before conversion.
  Compare against the existing summary `rx_path` counters for early rejection.
- `.../lr_fifo/0` through `.../lr_fifo/7`: rolling eight samples of the RX
  buffer, taken at most once per five seconds, regardless of eventual success or
  failure. A two-element, nonblocking FreeRTOS queue transfers copies from RX to
  main. If full, a sample is lost, never a received packet.
  These bytes may include noise after a short actual telegram due to fixed-length
  reception; do not count invalid trailing symbols as telegram corruption.

  Since 2026-09-22 the sample is **the whole 255-byte buffer read from offset 0**
  (`ReadBuffer8(0, 255)`, UM 2.2 p.35; RX RAM is addressable outside sleep, p.88),
  not the packet-sized read the decoder consumes. `fifo_dump` is 1 for such a
  sample and `raw_length` is 255. The reason: the packet-sized read takes
  `payload_len` bytes as reported by `GetRxBufferStatus`, and that is the length
  the packet engine was *declared* to expect - so it can never show whether the
  engine kept writing past it. With `payload_length` set below 255, the bytes
  past it are the only place that question can be answered. Cost is one extra
  255-byte SPI read after `RX_DONE`, at most once per five seconds; the decoder
  still receives the packet-sized read, unchanged.

  `probe` carries a **read-only** sample of four undocumented register
  addresses, taken right after `RX_DONE` in the same pass. `F20384` and
  `F20368` are the position-counter / end-of-packet pair that Semtech's own
  Sidewalk driver polls and writes on LR11xx with no named macro in the public
  SDK; `F30028` and `F30030` are the Rx FIFO base address and size registers
  documented for the LR20xx successor in the LR2021 datasheet rev 2.2, Tables
  5-2 and 5-3. Whether LR1121 has anything wired up at those addresses is the
  open question these values exist to answer. **Nothing is written to any of
  them.** Values are only interpretable against the `Register probe baseline
  (pre-RX, read-only)` line logged once at boot, taken after the radio is
  configured but before RX is ever armed - at that moment nothing has been
  received, so a position counter cannot legitimately hold a byte count. A
  value that never moves off the baseline, or reads as all-zero or all-ones,
  is a result and not a malfunction.

  That baseline is published to its own retained topic,
  `<diagnostic_topic>/probe_baseline`, and logged once from the main task as
  `Register probe baseline (pre-RX, read-only)`. Two reasons it is not simply
  logged from `setup()` and not folded into `radio_runtime`: component
  `setup()` runs before WiFi and the API are up, so a line emitted there never
  reaches `esphome logs` (restarting with the log attached does not help), and
  appending the four values to the `radio_runtime` JSON pushed that line past
  the logger's buffer - the log then printed the object cut off mid-key while
  MQTT still carried all of it. A diagnostic silently truncated in one of its
  two outputs is worse than one split across two lines.

  `packet_start` and `packet_len` carry the `GetRxBufferStatus` values for that
  capture, so the dump can be split into the declared packet
  (`[packet_start, packet_start + packet_len)`) and everything outside it.
  Without them the split would have to be assumed from the configured
  `payload_length`, which is exactly the assumption this dump exists to test.
  Bytes outside that range are not a decoded telegram and are not claimed to be
  one - they are whatever the RX RAM held at read time, which may be this
  capture, a previous one, or noise.

  Consequence for `verify`: the double-read comparison still covers only
  `payload_len` bytes, so `differing_bytes` and `first_difference` are indices
  into the packet-sized read, **not** into the 255-byte dump they are published
  next to. The dump is not the buffer that was compared.
- `.../lr_drop/0` through `.../lr_drop/7`: rolling eight failed conversions,
  at most once per five seconds. Includes actual parser reason/stage, lengths,
  3-of-6 symbol statistics and its raw input (up to 256 bytes). This input may
  already be trimmed by the receiver task; it is not necessarily the full FIFO.

No radio settings, restart policy or decoder decisions are changed. Sampling
adds bounded CPU/memory/MQTT overhead; verify nominal control reception.
Existing `diagnostic_publish_raw` and verbose logging need not be enabled.
Raw FIFO and drop sampling run independently: do not pair them by slot number.
Use boot ID and capture/wakeup uptime to correlate. Retained slots are overwritten;
old slots from earlier boots remain until overwritten. Always filter by boot ID
and test time. Export all these topics after the test as well as `radio_runtime`,
the existing diagnostic summary and receiver JSON. No data is published while
MQTT is disconnected; these samples are not a durable recorder.

## Early RX outcomes and optional SPI verification

`lr_pipeline` now contains terminal RX-task counters since boot:
`rx_entered`, `rx_queued`, `rx_queue_failed`, `rx_preamble_failed`,
`rx_weak_probe_aborted`, `rx_size_failed`, `rx_payload_failed`, `rx_s1_failed`.
At rest, entered equals the sum of all seven outcomes. During reception atomic
snapshots may differ by an in-flight attempt. These count IRQ-woken receive
attempts, not unique radio captures. Queue-to-conversion discrepancies can also
include pending queue entries and listen-mode filtering. Existing wakeup uptime
on dropped packets and capture uptime on FIFO samples remain the correlation
keys; no exact one-to-one identity between independently sampled records is claimed.

To run the intrusive buffer experiment on LR1121 only:

```yaml
wmbus_radio:
  lr1121_verify_buffer: true
```

Default is false. For at most one sampled capture per 5 seconds, after RX_DONE:
SetStandby(XOSC), wait BUSY, verify command success and standby mode, obtain
buffer offset/length, read the FIFO twice at the same address, verify offset,
length and mode again. No ClearRxBuffer or reset is issued. The original first
copy continues through the unchanged decoder; the second copy is only compared.
Existing restart_rx re-arms reception afterwards. This can interrupt a following
packet and is NOT a passive sensitivity measurement. UM 2.2 pp.16,35,88 documents
standby and addressable RX RAM accessible outside sleep.

FIFO sample fields: `fifo_dump` = 1 when `raw` is the whole 255-byte buffer read
from offset 0, 0 when it is the packet-sized read (see above); `packet_start`
and `packet_len` locate the declared packet inside that dump. `verify` = 0
disabled/not requested, 1 inconclusive (BUSY, command status, mode or pointer
checks did not pass), 2 identical, 3 different. `differing_bytes` counts unequal
bytes; `first_difference` is a zero-based byte offset, or 255 when none.

The `verify` fields describe **the packet-sized read of `payload_len` bytes**,
which is not the same span as the `raw` dump published beside them, and is
shorter than 255 whenever `payload_length` is configured below 255. Equality
does not prove correct RF demodulation, and a stable deterministic SPI error is
not excluded. A mismatch under validated standby is evidence to investigate the
read path, not automatic proof of bad RF reception.

## Writing the expected packet length (bench experiment, default off)

```yaml
wmbus_radio:
  lr1121_expected_len_override: 64   # 0 = off, the default
```

Non-zero writes that value into bits `[31:20]` of `0x00F20368` after every
`SetStandby(XOSC)` and before `SetRx`, overriding the length `SetPacketParams`
declared. **This is the first write to an undocumented register in this
component.** It is bench work, not a supported configuration, and it logs one
loud warning when it engages.

What makes the address usable rather than a guess, measured 2026-09-22: the
field read 255 with the radio configured for 255 and *nothing yet received*,
and 64 after `payload_length` was changed to 64. The pre-RX baseline is what
separates "mirrors `SetPacketParams`" from "holds the last packet's length" -
in fixed-length mode those two are otherwise always equal. The field is
12-bit, so it can express up to 4095, past the 8-bit `pld_len_in_bytes` of the
public API.

The write is gated on the radio firmware version and refuses, with a warning,
on anything other than the image it was verified against (`0x0101` here). An
undocumented register is a property of a firmware image, not a promise.

**Reading the result:** `packet_len` in the FIFO samples comes from
`GetRxBufferStatus`. If it follows the override rather than the configured
`payload_length`, the engine obeys the register and the write is effective.
If it keeps reporting the configured value, the register is not the control
path from this direction - which closes the question rather than failing.

Start in the safe direction, override *below* the configured length: the
buffer is 255 bytes and lowering the expectation cannot overrun it. Raising it
past 255 is a separate step and needs a transmitter that actually sends more
than 255 raw bytes; the longest telegram in normal field traffic here is 245.

## Sampling the position counter mid-frame (bench experiment, default off)

```yaml
wmbus_radio:
  lr1121_sync_probe: true
```

Adds `SYNC_WORD_VALID` to the T1/C1 interrupt mask so the receiver task wakes
while a frame is still arriving, reads `0x00F20384` bits `[27:16]` once,
clears **only** that latch, and returns. Results go to
`<diagnostic_topic>/sync_probe` every 60 s: `sync_wakes`, `ptr_last`,
`ptr_max`.

Clearing that one bit is load-bearing. DIO1 stays asserted while any unmasked
interrupt stands and the pin is read on a rising edge, so an uncleared
sync-word latch holds the line high and `RX_DONE` never produces an edge at
all. The first version of this probe deliberately cleared nothing, to "stay
out of the way", and measured 59 sync wakes with zero captures - it had
silently disabled reception. Only bit 5 is cleared; `RX_DONE` and the error
bits must survive. If the packet happens to finish during the sample, the
handler re-reads `GetRxBufferStatus` and continues into a normal capture
rather than waiting for an edge that has already passed.

Why it exists: a frame longer than the 255-byte buffer wraps, and the bytes it
overwrites are gone - after `RX_DONE` the start of such a frame no longer
exists anywhere. Capturing one therefore means draining during reception, and
that needs to know how much has arrived. `0x00F20384` reads 0 after `RX_DONE`;
whether it is live mid-frame is exactly what this measures. A counter that
stays 0 here means a drain would have to be timed off the bit rate instead.

**`IRQ_SYNC_WORD_VALID` was wrong until 2026-09-22** - it said bit 2, which is
`TX_DONE`. In a receive-only driver that bit never fires, so the S1 mask that
included it carried a dead bit and the S1 "sync matched but no packet"
diagnostic behind it never ran. The constant is now the documented bit 5 (UM
2.2 pp.37-39). Correcting the number changes nothing for S1 in practice, but
*enabling* the real interrupt there would: the S1 path clears the entire IRQ
latch, which would take `RX_DONE` with it. S1 therefore no longer requests the
bit at all, and enabling it for S1 needs dispatcher work first.

**Known side effect while this is on:** each early wake runs the normal receive
path, finds no packet and counts as `rx_preamble_failed`. That counter is
inflated for as long as the probe is enabled and should not be compared against
runs without it.

No automatic firmware deployment or experiment start is part of this change.
