# ESPHome YAML examples

[Polska wersja](README_PL.md)

This directory contains public board examples for the SX1262/SX1276 path.

CC1101 is intentionally not documented in public examples. It is still experimental even if the code is present in `main`.

Each supported board can have two variants:

- `*_commented.yaml` — recommended learning/reference config with comments.
- `*_clean.yaml` — minimal copy/paste base.

## MQTT topic convention

The examples use the new safer `topic_name` option:

```yaml
substitutions:
  devicename: esphome-wmbus-xiao-s3

wmbus_radio:
  topic_name: "${devicename}"
```

Do **not** include the `wmbus/` prefix in `topic_name`.

The component generates:

```text
wmbus/<topic_name>/telegram
wmbus/<topic_name>/diag
wmbus/<topic_name>/diag/summary
wmbus/<topic_name>/diag/summary_15min
wmbus/<topic_name>/diag/meter_snapshot
wmbus/<topic_name>/diag/boot
```

Recommended Home Assistant add-on subscription:

```text
wmbus/+/telegram
```

Diagnostic topics:

```text
wmbus/+/diag/#
```

Legacy manual options `telegram_topic` and `diagnostic_topic` are still supported for old YAML files, but new examples should use `topic_name`.

## Diagnostics

Use presets instead of many individual flags:

```yaml
diagnostic_mode: "normal"
```

Preset meaning:

- `off` — MQTT diagnostics disabled.
- `low` — global summary + hint.
- `normal` — summary + 15-minute summary + `meter_snapshot` for `highlight_meters`.
- `debug` — `normal` plus drop/RX-path events.
- `dev` — full developer diagnostics, including raw/debug payloads.

For per-meter statistics, set `highlight_meters` and use `diagnostic_mode: normal`:

```yaml
highlight_meters:
  - "00089907"
  - "03534159"

diagnostic_mode: "normal"
```

`diagnostic_publish_highlight_only` is deprecated. If you need to limit detailed diagnostic events to highlighted IDs, use:

```yaml
diagnostic_events_highlight_only: true
```

## listen_mode_filter_after_parse

Default:

```yaml
listen_mode_filter_after_parse: false
```

This is the conservative/stable mode, recommended when meters are nearby and reception is already good.

Experimental:

```yaml
listen_mode_filter_after_parse: true
```

This may help with distant meters, walls, or partially lost frames. It can increase valid frames, but usually also increases `false_start_like`, `payload_size_unknown`, and `t1_decode3of6` drops.

Compare using `meter_snapshot` for your actual meters, not global `drop_pct` alone.
