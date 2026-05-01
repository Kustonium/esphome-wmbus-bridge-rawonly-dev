# Migration note

If you previously used old external component lists:

```yaml
components: [wmbus_common, wmbus_radio]
```

or:

```yaml
components: [wmbus_bridge_common, wmbus_radio]
```

switch to:

```yaml
components: [wmbus_radio]
```

## MQTT topics

New recommended configuration:

```yaml
wmbus_radio:
  topic_name: "xiao_s3"
```

Do not include the `wmbus/` prefix. The component generates:

```text
wmbus/xiao_s3/telegram
wmbus/xiao_s3/diag
wmbus/xiao_s3/diag/summary
wmbus/xiao_s3/diag/summary_15min
wmbus/xiao_s3/diag/meter_snapshot
wmbus/xiao_s3/diag/boot
```

If `topic_name` is omitted, the component uses `esphome.name`.

Legacy options still work:

```yaml
telegram_topic: "..."
diagnostic_topic: "..."
```

They are treated as manual/legacy overrides and will log bilingual EN/PL warnings. Prefer `topic_name`.

Recommended add-on raw subscription:

```text
wmbus/+/telegram
```

## Diagnostics

Use presets:

```yaml
diagnostic_mode: "off"
diagnostic_mode: "low"
diagnostic_mode: "normal"
diagnostic_mode: "debug"
diagnostic_mode: "dev"
```

Compatibility aliases:

- `medium` -> `normal`
- `full` -> `dev`
- `raw` -> `dev`

Old detailed flags still compile, but are deprecated and log warnings. Example:

```yaml
diagnostic_publish_highlight_only: true
```

should become:

```yaml
diagnostic_events_highlight_only: true
```

For meter statistics, prefer:

```yaml
diagnostic_mode: "normal"
highlight_meters:
  - "00089907"
```

`normal` automatically publishes `summary`, `summary_15min`, and `meter_snapshot` for highlighted meters.

## CC1101

CC1101 remains experimental and intentionally has no public example YAML in this repository. The code path requires explicit opt-in and correct GDO0/GDO2 wiring.
