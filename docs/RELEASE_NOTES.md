# Release notes / Informacje o zmianach

## EN

### Summary
Clean up MQTT topics and diagnostics, keep old YAML compatibility, and restore important radio diagnostics.

### Changes
- Added `topic_name` as the preferred MQTT topic configuration.
  - User provides only a safe device topic name, for example `topic_name: "xiao_s3"`.
  - The component generates `wmbus/<topic_name>/telegram` and `wmbus/<topic_name>/diag/...`.
  - If `topic_name` is omitted, `esphome.name` is used.
  - `friendly_name` is intentionally not used for topics.
- Added validation for `topic_name`.
  - Allowed: letters, digits, `_`, `-`.
  - Forbidden: `/`, `+`, `#`, spaces, `wmbus/` prefix, national characters.
- Kept `telegram_topic` and `diagnostic_topic` as legacy/manual overrides with bilingual EN/PL warnings.
- Reworked diagnostic presets:
  - `off`
  - `low`
  - `normal`
  - `debug`
  - `dev`
- Added compatibility aliases:
  - `medium` -> `normal`
  - `full` -> `dev`
  - `raw` -> `dev`
- Kept old detailed diagnostic options as deprecated compatibility flags with bilingual EN/PL warnings.
- Replaced the misleading `diagnostic_publish_highlight_only` name with `diagnostic_events_highlight_only`.
  - The old option remains as a deprecated alias.
- Added `diagnostic_meter_stats: off/highlighted/all`.
  - `normal` automatically enables highlighted meter snapshots when `highlight_meters` is configured.
  - `dev` can track all seen meters.
- Restored RF parameter logging in the startup `Radio active` message.
- Fixed `busy_ether_state` reporting:
  - SX1276 reports adaptive/normal/aggressive state.
  - SX1262 and CC1101 report `n/a`.
- Kept `listen_mode_filter_after_parse` defaulting to `false`.
  - `false` is conservative/stable.
  - `true` is experimental/aggressive and should be compared using `meter_snapshot`.

### Notes
- Existing YAML files should still compile.
- Legacy options now warn instead of silently confusing users.
- CC1101 remains experimental and is not documented in public examples.

---

## PL

### Podsumowanie
Uporządkowano topiki MQTT i diagnostykę, zachowano kompatybilność starych YAML-i oraz przywrócono ważne logi diagnostyczne radia.

### Zmiany
- Dodano `topic_name` jako zalecaną konfigurację topiców MQTT.
  - Użytkownik podaje tylko bezpieczną nazwę urządzenia, np. `topic_name: "xiao_s3"`.
  - Komponent generuje `wmbus/<topic_name>/telegram` oraz `wmbus/<topic_name>/diag/...`.
  - Jeśli `topic_name` nie jest podany, używane jest `esphome.name`.
  - `friendly_name` celowo nie jest używane do topiców.
- Dodano walidację `topic_name`.
  - Dozwolone: litery, cyfry, `_`, `-`.
  - Zakazane: `/`, `+`, `#`, spacje, prefiks `wmbus/`, polskie znaki.
- Zachowano `telegram_topic` i `diagnostic_topic` jako legacy/manual override z warningami EN/PL.
- Przebudowano presety diagnostyczne:
  - `off`
  - `low`
  - `normal`
  - `debug`
  - `dev`
- Dodano aliasy kompatybilności:
  - `medium` -> `normal`
  - `full` -> `dev`
  - `raw` -> `dev`
- Stare szczegółowe opcje diagnostyczne nadal działają, ale są deprecated i wypisują warningi EN/PL.
- Zastąpiono mylącą nazwę `diagnostic_publish_highlight_only` przez `diagnostic_events_highlight_only`.
  - Stara opcja zostaje jako alias deprecated.
- Dodano `diagnostic_meter_stats: off/highlighted/all`.
  - `normal` automatycznie włącza snapshoty wyróżnionych liczników, gdy skonfigurowano `highlight_meters`.
  - `dev` może śledzić wszystkie widziane liczniki.
- Przywrócono logowanie parametrów RF w komunikacie startowym `Radio active`.
- Poprawiono raportowanie `busy_ether_state`:
  - SX1276 raportuje stan adaptive/normal/aggressive.
  - SX1262 i CC1101 raportują `n/a`.
- `listen_mode_filter_after_parse` domyślnie pozostaje `false`.
  - `false` to tryb konserwatywny/stabilny.
  - `true` to tryb eksperymentalny/agresywniejszy, który należy porównywać przez `meter_snapshot`.

### Uwagi
- Istniejące YAML-e powinny nadal się kompilować.
- Stare opcje teraz ostrzegają, zamiast po cichu mylić użytkownika.
- CC1101 nadal jest eksperymentalny i nie jest opisany w publicznych przykładach.
