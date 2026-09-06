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
- `.../lr_fifo/0` through `.../lr_fifo/7`: rolling eight samples of the full FIFO
  read (up to 255 bytes), taken at most once per five seconds, regardless of
  eventual success or failure. A two-element, nonblocking FreeRTOS queue transfers
  copies from RX to main. If full, a sample is lost, never a received packet.
  These bytes may include noise after a short actual telegram due to fixed-length
  reception; do not count invalid trailing symbols as telegram corruption.
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
