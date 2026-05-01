# Diagnostyka wmbus_radio — PL

[English version](TROUBLESHOOTING.md)

## Złota zasada

Nie oceniaj odbioru wyłącznie po globalnym `summary` i `drop_pct`.

Najważniejsza kolejność:

1. `boot`
2. `summary`
3. `summary_15min`
4. `meter_snapshot`
5. `meter_window`
6. `dropped` / `truncated`
7. `rx_path`
8. `suggestion`
9. `busy_ether_changed` — tylko SX1276

Jeśli pominiesz `meter_snapshot`, możesz dojść do złego wniosku. Globalny `summary` pokazuje cały eter, a nie tylko Twoje liczniki.

## Zalecany minimalny YAML diagnostyczny

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

To automatycznie daje:

```text
wmbus/<topic_name>/diag/summary
wmbus/<topic_name>/diag/summary_15min
wmbus/<topic_name>/diag/meter_snapshot
```

Nie musisz już ręcznie dopisywać starego:

```yaml
diagnostic_publish_summary_highlight_meters: true
```

Stare opcje nadal działają, ale są traktowane jako deprecated.

## Topic MQTT

Zalecane:

```yaml
topic_name: "xiao_s3"
```

Nie wpisuj `wmbus/`.

Komponent sam wygeneruje:

```text
wmbus/xiao_s3/telegram
wmbus/xiao_s3/diag
wmbus/xiao_s3/diag/summary
wmbus/xiao_s3/diag/meter_snapshot
```

Jeśli `topic_name` nie jest podany, komponent użyje `esphome.name`.

Nie używamy `friendly_name` jako domyślnej nazwy topicu, bo może mieć spacje, wielkie litery albo polskie znaki.

Stare ręczne opcje:

```yaml
telegram_topic: "..."
diagnostic_topic: "..."
```

nadal działają, ale są legacy/manual override i generują warning EN/PL.

## Tryby diagnostyki

```yaml
diagnostic_mode: "off"
diagnostic_mode: "low"
diagnostic_mode: "normal"
diagnostic_mode: "debug"
diagnostic_mode: "dev"
```

Znaczenie:

- `off` — diagnostyka MQTT wyłączona.
- `low` — globalne summary + hint.
- `normal` — summary + summary 15-minutowe + `meter_snapshot` dla `highlight_meters`.
- `debug` — `normal` plus eventy drop/RX-path.
- `dev` — pełna diagnostyka developerska, także raw/debug payloady.

Aliasy kompatybilności:

- `medium` -> `normal`
- `full` -> `dev`
- `raw` -> `dev`

## `diagnostic_publish_highlight_only`

Ta nazwa była myląca.

Stare:

```yaml
diagnostic_publish_highlight_only: true
```

nie włączało statystyk liczników. Ono tylko filtrowało szczegółowe eventy diagnostyczne do `highlight_meters`.

Nowa nazwa:

```yaml
diagnostic_events_highlight_only: true
```

Stara opcja nadal działa jako alias deprecated.

## Statystyki liczników

Zalecane:

```yaml
diagnostic_mode: "normal"
highlight_meters:
  - "00089907"
```

Opcjonalny jawny override:

```yaml
diagnostic_meter_stats: "off"
diagnostic_meter_stats: "highlighted"
diagnostic_meter_stats: "all"
```

`all` jest dla testów/dev, bo może zwiększyć zużycie RAM oraz ilość logów/MQTT.

## `listen_mode_filter_after_parse`

Domyślnie:

```yaml
listen_mode_filter_after_parse: false
```

To tryb konserwatywny/stabilny. Zalecany, gdy liczniki są blisko i odbiór jest już dobry.

Eksperymentalnie:

```yaml
listen_mode_filter_after_parse: true
```

Ten tryb może pomóc przy licznikach dalej od odbiornika, za ścianami albo z częściowo traconymi ramkami.

Może zwiększyć liczbę poprawnych odczytów, ale zwykle zwiększa też:

- `false_start_like`
- `payload_size_unknown`
- dropy `t1_decode3of6`

Porównuj po `meter_snapshot` i statystykach konkretnych liczników, nie po samym globalnym `drop_pct`.

## `busy_ether_state`

`busy_ether_state` dotyczy tylko SX1276.

Prawidłowo:

```text
SX1276 -> adaptive_active / adaptive_passive / aggressive / normal
SX1262 -> n/a
CC1101 -> n/a
```

Jeśli SX1262 albo CC1101 pokazuje `adaptive_passive`, to jest błąd diagnostyki.

## `summary` wygląda dobrze, ale licznik nadal gubi pakiety

Najczęstsze przyczyny:

- straty dzieją się przed finalnym decode,
- eter jest gęsty,
- licznik nadaje rzadko,
- tryb `both` dokłada koszt odbioru,
- patrzysz na globalny `summary`, a nie na konkretny licznik.

Sprawdź:

- `meter_snapshot.count_window`
- `meter_snapshot.win_avg_interval_s`
- `meter_window.count_window`
- czy licznik ma spodziewany interwał.

## `drop_pct` jest wyższy, ale licznik działa lepiej

To może być normalne przy trybie bardziej agresywnym.

Przykład:

- stary tryb widzi mniej kandydatów i ma niski `drop_pct`,
- nowy tryb widzi więcej kandydatów,
- część odpada jako drop,
- ale konkretne liczniki mają większy `count_window`.

Wniosek: `drop_pct` nie jest jedyną metryką. Liczy się `meter_snapshot`.

## `both` działa, ale T1 zrobiło się gorsze

To jest spodziewane w wielu realnych środowiskach.

`both` nie oznacza dwóch równoległych odbiorników. To kompromis. Szczególnie na SX1276 może zwiększyć straty.

Zalecenie:

- zacznij od `listen_mode: t1`,
- używaj `c1` tylko jeśli wiesz, że licznik nadaje C1,
- `both` tylko świadomie,
- dla stabilnego mixed-mode lepsze są dwa dedykowane urządzenia.

## CC1101

CC1101 jest nadal eksperymentalny.

Nie ma publicznych przykładów YAML dla CC1101 w repo, nawet jeśli kod znajduje się w `main`.

Wymagane jest jawne włączenie:

```yaml
cc1101_allow_experimental: true
```

CC1101 wymaga osobnych pinów:

```yaml
gdo0_pin:
gdo2_pin:
```

Nie używaj `irq_pin` dla CC1101.
