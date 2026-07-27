# Fix: meters with a non-BCD ID can be matched by forward_meters and highlight_meters

## EN

### Fixed
- Meter matching decoded the A-field as BCD and gave up otherwise, so meters that do not use a BCD ID (Diehl/IZAR among others) had no usable ID at all. They could never be listed in `highlight_meters`, and with `forward_meters` active their telegrams were dropped silently - the one case where the whitelist discarded frames the user could not get back by any configuration.
- Both options now also match the raw A-field value, written the way the log prints it: `id:417F0666` is configured as `"0x417F0666"` (quoted, or YAML turns it into a number). Decimal entries keep their existing meaning, so no configuration changes behaviour.
- The two forms are told apart without ambiguity: a non-BCD A-field always contains a nibble above 9 and therefore always prints a hex letter, while a BCD ID never does. The `0x` form works for BCD meters too (`"0x00089907"` is meter `89907`).
- Per-meter statistics were keyed on the BCD ID, so every non-BCD meter collapsed into a single shared entry at key 0. They are now keyed on the raw A-field value, which is unique for every meter.
- `target_meter_id` still accepts only BCD IDs. A hex value there used to be accepted and then never match; it now logs a warning at boot pointing to `forward_meters`.

## PL

### Naprawiono
- Dopasowanie liczników dekodowało A-field jako BCD i w przeciwnym razie rezygnowało, więc liczniki bez ID w BCD (m.in. Diehl/IZAR) nie miały żadnego użytecznego ID. Nie dało się ich wpisać do `highlight_meters`, a przy aktywnym `forward_meters` ich telegramy znikały bez śladu - jedyny przypadek, w którym whitelista odrzucała ramki, których użytkownik nie mógł odzyskać żadną konfiguracją.
- Obie opcje dopasowują teraz również surową wartość A-field, zapisywaną tak, jak pokazuje ją log: `id:417F0666` konfigurujesz jako `"0x417F0666"` (w cudzyslowie, inaczej YAML zamieni to na liczbe). Wpisy dziesiętne zachowują dotychczasowe znaczenie, więc żadna konfiguracja nie zmienia zachowania.
- Rozróżnienie obu form jest jednoznaczne: A-field poza BCD zawsze zawiera półbajtówkę powyżej 9, więc zawsze wypisuje literę szesnastkową, a ID w BCD nigdy. Forma `0x` działa też dla liczników BCD (`"0x00089907"` to licznik `89907`).
- Statystyki per licznik były kluczowane po ID z BCD, więc wszystkie liczniki nie-BCD zlewały się w jeden wspólny wpis pod kluczem 0. Teraz kluczem jest surowa wartość A-field, unikalna dla każdego licznika.
- `target_meter_id` nadal przyjmuje wyłącznie ID w BCD. Wartość szesnastkowa była tam dotąd przyjmowana i po cichu nigdy nie pasowała; teraz przy starcie pojawia się ostrzeżenie kierujące do `forward_meters`.

---

# Fix: NO_METERS_DETECTED no longer fires on a quiet summary window

## EN

### Fixed
- `NO_METERS_DETECTED` claimed a wiring or radio-configuration fault whenever a single summary window contained no frames. That counter (`diag_total_`) is reset after every window, so a receiver that had been working all day still reported `total == 0` for any quiet minute - and meters are routinely quiet at night. The suggestion is now limited to receivers that have not seen a single frame since boot, and additionally suppressed for the first 5 minutes of uptime, where silence carries no information because meters transmit tens of seconds to several minutes apart.
- A receiver that used to work and then went silent is a different diagnosis and stays with the health pulse (`sec_since_last_rx`); it was never what this suggestion measured.
- The `NO_DATA` summary hint said "no packets received yet", which reads as "nothing ever arrived" even though it describes a single window. It now says "no packets in this window". The machine-readable `hint_code` is unchanged.

## PL

### Naprawiono
- `NO_METERS_DETECTED` sugerowało usterkę okablowania lub konfiguracji radia za każdym razem, gdy pojedyncze okno podsumowania nie zawierało ramek. Ten licznik (`diag_total_`) jest zerowany po każdym oknie, więc odbiornik działający cały dzień i tak raportował `total == 0` w dowolnej cichej minucie - a liczniki nocą standardowo milczą. Sugestia ogranicza się teraz do odbiorników, które od startu nie odebrały ani jednej ramki, i dodatkowo jest wyciszona przez pierwsze 5 minut pracy, gdzie cisza nic nie znaczy, bo liczniki nadają co kilkadziesiąt sekund do kilku minut.
- Odbiornik, który działał i zamilkł, to inna diagnoza - pozostaje przy pulsie health (`sec_since_last_rx`); ta sugestia nigdy tego nie mierzyła.
- Hint `NO_DATA` w podsumowaniu mówił "no packets received yet", co brzmi jak "nic nigdy nie przyszło", choć opisuje pojedyncze okno. Teraz mówi "no packets in this window". Maszynowe `hint_code` bez zmian.

---

# Feature: forward_meters - forwarding whitelist for the RAW telegram topic

## EN

### Added
- `forward_meters` limits which meters are published to `wmbus/<topic_name>/telegram`. Useful where most of the received traffic belongs to neighbouring meters and only a few are your own.
- Accepts an explicit list of meter IDs, or `true` to reuse the IDs already listed in `highlight_meters` so the same list is not written twice.
- An empty list (the default) or `false` forwards every decoded frame, so existing configurations are unaffected.
- The parsed IDs are logged from `setup()` and again in the delayed boot status block, next to the YAML sanity output. The repetition is deliberate: `setup()` runs before the network logger attaches, so over `esphome logs` only the second one is visible. `dump_config()` reports the same state, but is filtered out at `logger: level: info` - it needs `level: config` or higher.

### Notes
- The filter runs after decoding and the DLL CRC check, so it matches a meter ID the parser has already validated. Filtering on the raw header would be cheaper but unreliable: an ID read from a frame that failed CRC can be corrupted.
- Matching uses the BCD-decoded 8-digit meter ID. A meter whose log line shows a hex ID (`id:417F0666`) has a non-BCD A-field and cannot be whitelisted - it is dropped while the filter is active.
- `forward_meters: true` with an empty `highlight_meters` does not silence the stream: filtering stays off and a configuration warning is printed at boot.
- Diagnostics are unaffected. Counters and RSSI statistics are updated before publishing, so summaries still cover the whole ether including neighbours; only the RAW stream is reduced.
- `target_meter_id` keeps its own topic and is deliberately not subject to the whitelist.

## PL

### Dodano
- `forward_meters` ogranicza to, które liczniki trafiają na `wmbus/<topic_name>/telegram`. Przydatne tam, gdzie większość odbieranego ruchu pochodzi od liczników sąsiadów, a własnych jest kilka.
- Przyjmuje jawną listę ID liczników albo `true`, które bierze ID już wpisane w `highlight_meters` - dzięki temu ta sama lista nie musi być pisana dwa razy.
- Pusta lista (domyślnie) albo `false` przepuszcza każdą zdekodowaną ramkę, więc istniejące konfiguracje działają bez zmian.
- Sparsowane ID trafiają do logu z `setup()` oraz ponownie w opóźnionym bloku startowym, obok informacji sanity YAML. Powtórzenie jest celowe: `setup()` wykonuje się, zanim podłączy się logger sieciowy, więc przy `esphome logs` widać dopiero to drugie. `dump_config()` raportuje ten sam stan, ale przy `logger: level: info` jest odfiltrowany - wymaga `level: config` lub wyżej.

### Uwagi
- Filtr działa po dekodowaniu i sprawdzeniu DLL CRC, więc dopasowuje ID, które parser już zweryfikował. Filtrowanie po surowym nagłówku byłoby tańsze, ale zawodne: ID odczytane z ramki, która nie przeszła CRC, bywa przekłamane.
- Dopasowanie idzie po ośmiocyfrowym ID zdekodowanym z BCD. Licznik, którego log pokazuje ID szesnastkowo (`id:417F0666`), ma A-field poza BCD i nie da się go wpisać na whitelistę - przy aktywnym filtrze zostanie odrzucony.
- `forward_meters: true` przy pustym `highlight_meters` nie wycisza strumienia: filtr się nie włącza, a przy starcie pojawia się ostrzeżenie konfiguracyjne.
- Diagnostyka jest nietknięta. Liczniki i statystyki RSSI powstają przed publikacją, więc summary dalej obejmuje cały eter razem z sąsiadami; obcinany jest sam strumień RAW.
- `target_meter_id` ma własny topic i celowo nie podlega whiteliście.

---

# Fix: guard gmtime()/strftime() in rtlwmbus timestamp

## EN

### Fixed
- `Frame::as_rtlwmbus()` now guards `std::gmtime()` returning `nullptr` and `std::strftime()` returning `0`, falling back to a fixed `1970-01-01 00:00:00.00Z` timestamp. Prevents a potential null-dereference / unterminated-buffer read when the system clock holds a `time_t` value that cannot be represented (e.g. an unset or out-of-range clock). Hardening only — no wire or format change during normal operation.

## PL

### Naprawiono
- `Frame::as_rtlwmbus()` zabezpiecza teraz przypadki, gdy `std::gmtime()` zwraca `nullptr`, a `std::strftime()` zwraca `0` — z fallbackiem do stałego znacznika `1970-01-01 00:00:00.00Z`. Zapobiega potencjalnemu null-dereference / odczytowi niedokończonego bufora, gdy zegar systemowy trzyma wartość `time_t` niemożliwą do reprezentacji (np. nieustawiony lub poza zakresem). Wyłącznie utwardzenie — bez zmiany formatu wyjścia w normalnej pracy.

---

# Current documentation note

## EN

### Added
- Experimental S1 receive mode.
- `listen_mode: s1` uses a dedicated S1 receive path and does not fall back to T1/C1 parsing.
- `listen_mode: both` remains T1/C1 only.
- `listen_mode: s1` defaults to `868.300 MHz`; T1/C1/both remain at `868.950 MHz`.
- Explicit `frequency:` in YAML still overrides the mode default.

### Notes
- S1 support is intended for diagnostics and compatibility testing.
- If a valid S1 telegram is received, it is published to MQTT like other validated wM-Bus telegrams.
- Meter-value decoding remains external and depends on the backend driver and encryption key.
- Proprietary or polling-based systems may not produce standard passive S1 telegrams.

## PL

### Dodano
- Eksperymentalny tryb odbioru S1.
- `listen_mode: s1` używa dedykowanej ścieżki odbioru S1 i nie przechodzi przez logikę T1/C1.
- `listen_mode: both` nadal oznacza tylko T1/C1.
- `listen_mode: s1` domyślnie ustawia `868.300 MHz`; T1/C1/both zostają przy `868.950 MHz`.
- Jawne `frequency:` w YAML nadal nadpisuje domyślną częstotliwość trybu.

### Uwagi
- Obsługa S1 jest przeznaczona do diagnostyki i testów kompatybilności.
- Jeżeli poprawny telegram S1 zostanie odebrany, zostanie opublikowany do MQTT tak jak inne zweryfikowane telegramy wM-Bus.
- Dekodowanie wartości licznika pozostaje po stronie backendu i zależy od drivera oraz klucza szyfrowania.
- Systemy zamknięte albo odpytywane mogą nie nadawać standardowych pasywnych telegramów S1.

---

**EN**

**Summary**
Improve adaptive SX1276 behavior, add MQTT diagnostic suggestions, and expand runtime diagnostics.

**Description**
This release improves the real-world behavior of the RAW-only wM-Bus bridge, especially on SX1276 in noisy RF environments.

Main changes:

* improved `sx1276_busy_ether_mode: adaptive` logic so activation reacts to actual reception loss, not just RF noise
* added MQTT `suggestion` events with actionable diagnostic hints and YAML snippets
* added `busy_ether_changed` MQTT events for adaptive state transitions
* expanded diagnostic summaries with new runtime fields, including `busy_ether_state`
* added/expanded `summary_15min`, `summary_60min`, and per-meter snapshot reporting
* fixed multiple logic and documentation inconsistencies discovered during real hardware testing
* fixed: `busy_ether_state` in `/summary` JSON now emits `"n/a"` on SX1262 instead of the misleading `"adaptive_passive"` (the algorithm never ran on SX1262, only the stored mode value was serialised)
* fixed: `hint_code` no longer stays `"OK"` for windows with 11-99% drop rate and no specific diagnosis — new code `MODERATE_DROPS` is emitted instead so elevated drops are always visible as WARN in serial log
* changed: `highlight_meters` per-packet serial log now shows `packet #N received` instead of the previous `stats / statystyki: count=N interval=... avg_rssi=...`; per-meter stats remain available via MQTT `meter_window` events

This version does not change the project architecture: the ESP device still focuses on RF reception and RAW MQTT publishing, while meter decoding remains external.

**PL**

**Summary**
Poprawa działania adaptive dla SX1276, dodanie sugestii diagnostycznych MQTT oraz rozbudowa diagnostyki runtime.

**Description**
To wydanie poprawia rzeczywiste zachowanie mostka RAW-only wM-Bus, szczególnie dla SX1276 w zaszumionym środowisku RF.

Najważniejsze zmiany:

* poprawiono logikę `sx1276_busy_ether_mode: adaptive`, tak aby aktywacja reagowała na realne straty odbioru, a nie tylko sam szum radiowy
* dodano eventy MQTT `suggestion` z praktycznymi wskazówkami diagnostycznymi i gotowymi snippetami YAML
* dodano eventy MQTT `busy_ether_changed` dla zmian stanu adaptive
* rozszerzono raporty diagnostyczne o nowe pola runtime, w tym `busy_ether_state`
* dodano/rozszerzono raporty `summary_15min`, `summary_60min` oraz snapshoty per-meter
* poprawiono kilka niespójności logicznych i dokumentacyjnych wykrytych podczas testów na realnym sprzęcie
* poprawka: `busy_ether_state` w JSON `/summary` emituje teraz `"n/a"` na SX1262 zamiast mylącego `"adaptive_passive"` (algorytm nigdy nie działał na SX1262, serializowana była tylko wartość pola)
* poprawka: `hint_code` nie pozostaje już `"OK"` przy 11-99% dropów bez konkretnej diagnozy — nowy kod `MODERATE_DROPS` sprawia że podwyższone straty zawsze widoczne są jako WARN w logu
* zmiana: log per-pakietu `highlight_meters` wyświetla teraz `packet #N received` zamiast poprzedniego `stats / statystyki: count=N interval=... avg_rssi=...`; statystyki per-licznik nadal dostępne przez MQTT `meter_window`

To wydanie nie zmienia architektury projektu: urządzenie ESP nadal odpowiada za odbiór RF i publikację RAW do MQTT, a dekodowanie liczników pozostaje poza nim.
