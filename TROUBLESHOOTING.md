# wmbus_radio troubleshooting — EN

[Polska wersja](TROUBLESHOOTING_PL.md)

## Golden rule

Do not judge reception only by global `summary` and `drop_pct`.

Recommended order:

1. `boot`
2. `summary`
3. `summary_15min`
4. `meter_snapshot`
5. `meter_window`
6. `dropped` / `truncated`
7. `rx_path`
8. `suggestion`
9. `busy_ether_changed` — SX1276 only

If you skip `meter_snapshot`, you can fool yourself. Global `summary` describes the whole RF environment, not only your meters.

## Recommended minimal diagnostic YAML

```yaml
wmbus_radio:
  topic_name: "${devicename}"
  listen_mode: t1

  highlight_meters:
    - "00089907"
    - "03534159"
    - "03528221"

  diagnostic_mode: "normal"
```

This automatically publishes:

```text
wmbus/<topic_name>/diag/summary
wmbus/<topic_name>/diag/summary_15min
wmbus/<topic_name>/diag/meter_snapshot
```

You no longer need the old explicit:

```yaml
diagnostic_publish_summary_highlight_meters: true
```

Old options still work, but are treated as deprecated.

## MQTT topics

Recommended:

```yaml
topic_name: "xiao_s3"
```

Do not include `wmbus/`.

The component generates:

```text
wmbus/xiao_s3/telegram
wmbus/xiao_s3/diag
wmbus/xiao_s3/diag/summary
wmbus/xiao_s3/diag/meter_snapshot
```

If `topic_name` is omitted, the component uses `esphome.name`.

`friendly_name` is intentionally not used for topics because it may contain spaces, uppercase letters or national characters.

Legacy manual options:

```yaml
telegram_topic: "..."
diagnostic_topic: "..."
```

still work, but are legacy/manual overrides and log bilingual EN/PL warnings.

## Diagnostic modes

```yaml
diagnostic_mode: "off"
diagnostic_mode: "low"
diagnostic_mode: "normal"
diagnostic_mode: "debug"
diagnostic_mode: "dev"
```

Meaning:

- `off` — MQTT diagnostics disabled.
- `low` — global summary + hint.
- `normal` — summary + 15-minute summary + `meter_snapshot` for `highlight_meters`.
- `debug` — `normal` plus drop/RX-path events.
- `dev` — full developer diagnostics, including raw/debug payloads.

Compatibility aliases:

- `medium` -> `normal`
- `full` -> `dev`
- `raw` -> `dev`

## `diagnostic_publish_highlight_only`

This name was misleading.

Old:

```yaml
diagnostic_publish_highlight_only: true
```

did not enable meter statistics. It only filtered detailed diagnostic events to `highlight_meters`.

New name:

```yaml
diagnostic_events_highlight_only: true
```

The old option remains as a deprecated alias.

## Meter statistics

Recommended:

```yaml
diagnostic_mode: "normal"
highlight_meters:
  - "00089907"
```

Optional explicit override:

```yaml
diagnostic_meter_stats: "off"
diagnostic_meter_stats: "highlighted"
diagnostic_meter_stats: "all"
```

`all` is intended for dev/testing because it may increase RAM usage and MQTT/log traffic.

## `listen_mode_filter_after_parse`

Default:

```yaml
listen_mode_filter_after_parse: false
```

This is the conservative/stable mode. Recommended when meters are nearby and reception is already good.

Experimental:

```yaml
listen_mode_filter_after_parse: true
```

This mode may help with meters farther away, behind walls, or with partially lost frames.

It can increase the number of valid frames, but usually also increases:

- `false_start_like`
- `payload_size_unknown`
- `t1_decode3of6` drops

Compare using `meter_snapshot` and real meter statistics, not global `drop_pct` alone.

## `busy_ether_state`

`busy_ether_state` is SX1276-only.

Correct behavior:

```text
SX1276 -> adaptive_active / adaptive_passive / aggressive / normal
SX1262 -> n/a
CC1101 -> n/a
```

If SX1262 or CC1101 shows `adaptive_passive`, that is a diagnostic bug.

## `summary` looks good, but a meter still misses packets

Common reasons:

- losses happen before final decode,
- dense RF environment,
- meter transmits rarely,
- `both` mode adds reception cost,
- you are looking at global `summary`, not the specific meter.

Check:

- `meter_snapshot.count_window`
- `meter_snapshot.win_avg_interval_s`
- `meter_window.count_window`
- whether the meter interval matches expectations.

## Higher `drop_pct`, but better meter reception

This can be normal in the more aggressive mode.

Example:

- conservative mode sees fewer candidates and has low `drop_pct`,
- aggressive mode sees more candidates,
- some fail as drops,
- but specific meters have a higher `count_window`.

Conclusion: `drop_pct` is not the only metric. Use `meter_snapshot`.

## `both` works, but T1 got worse

This is expected in many real environments.

`both` does not mean two parallel receivers. It is a compromise. It can hurt especially on SX1276.

Recommendation:

- start with `listen_mode: t1`,
- use `c1` only if you know the meter transmits C1,
- use `both` deliberately,
- for stable mixed-mode, two dedicated devices are better.

## CC1101

CC1101 is still experimental.

There are no public CC1101 YAML examples in the repository, even if the code is present in `main`.

Explicit opt-in is required:

```yaml
cc1101_allow_experimental: true
```

CC1101 requires separate pins:

```yaml
gdo0_pin:
gdo2_pin:
```

Do not use `irq_pin` for CC1101.
