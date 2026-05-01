# Przykłady YAML ESPHome

[English version](README.md)

Ten katalog zawiera publiczne przykłady dla toru SX1262/SX1276.

CC1101 celowo nie jest opisany w publicznych przykładach. Nadal jest eksperymentalny, nawet jeśli kod znajduje się w `main`.

Dla obsługiwanych płytek mogą występować dwie wersje:

- `*_commented.yaml` — przykład z komentarzami, dobry do nauki i jako baza.
- `*_clean.yaml` — krótka baza do kopiowania.

## Konwencja topiców MQTT

Przykłady używają nowej, bezpieczniejszej opcji `topic_name`:

```yaml
substitutions:
  devicename: esphome-wmbus-xiao-s3

wmbus_radio:
  topic_name: "${devicename}"
```

Nie wpisuj prefiksu `wmbus/` w `topic_name`.

Komponent sam generuje:

```text
wmbus/<topic_name>/telegram
wmbus/<topic_name>/diag
wmbus/<topic_name>/diag/summary
wmbus/<topic_name>/diag/summary_15min
wmbus/<topic_name>/diag/meter_snapshot
wmbus/<topic_name>/diag/boot
```

Zalecana subskrypcja dodatku Home Assistant:

```text
wmbus/+/telegram
```

Topiki diagnostyczne:

```text
wmbus/+/diag/#
```

Stare ręczne opcje `telegram_topic` i `diagnostic_topic` nadal działają dla starych YAML-i, ale nowe przykłady powinny używać `topic_name`.

## Diagnostyka

Używaj presetów zamiast wielu osobnych flag:

```yaml
diagnostic_mode: "normal"
```

Znaczenie presetów:

- `off` — diagnostyka MQTT wyłączona.
- `low` — globalne summary + hint.
- `normal` — summary + summary 15-minutowe + `meter_snapshot` dla `highlight_meters`.
- `debug` — `normal` plus eventy drop/RX-path.
- `dev` — pełna diagnostyka developerska, w tym raw/debug payloady.

Dla statystyk per licznik ustaw `highlight_meters` i użyj `diagnostic_mode: normal`:

```yaml
highlight_meters:
  - "00089907"
  - "03534159"

diagnostic_mode: "normal"
```

`diagnostic_publish_highlight_only` jest przestarzałe. Jeśli chcesz ograniczyć szczegółowe eventy diagnostyczne do wyróżnionych ID, użyj:

```yaml
diagnostic_events_highlight_only: true
```

## listen_mode_filter_after_parse

Domyślnie:

```yaml
listen_mode_filter_after_parse: false
```

To tryb konserwatywny/stabilny, zalecany gdy liczniki są blisko i odbiór jest już dobry.

Eksperymentalnie:

```yaml
listen_mode_filter_after_parse: true
```

Może pomóc przy licznikach dalej od odbiornika, za ścianami albo z częściowo traconymi ramkami. Może zwiększyć liczbę poprawnych odczytów, ale zwykle zwiększa też `false_start_like`, `payload_size_unknown` i dropy `t1_decode3of6`.

Porównuj po `meter_snapshot` dla konkretnych liczników, nie po samym globalnym `drop_pct`.
