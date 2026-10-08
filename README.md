# leaf-telematics-vehicle-profiles

Data-driven vehicle profiles for the ESP32 telematics device. Each profile
describes, as JSON, everything the firmware needs to talk to one vehicle:
periodic keep-alive CAN frames, UDS/OBD polling, and passive CAN-frame decode
rules. Adding support for a new vehicle means adding a JSON file here — no
firmware recompilation.

The ESP32 downloads profiles over HTTPS from this repo at runtime. The base
URL is:

```
https://raw.githubusercontent.com/<USER>/<REPO>/<BRANCH>/
```

It downloads `vehicle_profiles.json` (the index) first, then appends the
repo-relative `profile` path of the chosen vehicle to the base URL, e.g.:

```
https://raw.githubusercontent.com/<USER>/<REPO>/<BRANCH>/nissan/leaf-ze0.json
```

## Layout

```
vehicle_profiles.json      index of all vehicles (id, name, profile path)
generic_obd2.json          generic OBD-II mode-01 profile (any vehicle)
nissan/leaf-ze0.json       Nissan Leaf ZE0/AZE0 (2011-2017) full battery profile
tools/validate_profiles.py schema validator (Python 3, stdlib only)
```

Run the validator after every change (also usable as a CI step):

```
python tools/validate_profiles.py
```

It parses every referenced profile, checks the field formats below, and fully
parses every signal expression with a recursive-descent parser implementing
exactly the grammar in this README. Exit code is non-zero on any error.

## Index format (`vehicle_profiles.json`)

```json
{
  "version": 1,
  "vehicles": [
    {"id": "nissan_leaf_ze0", "name": "Nissan : Leaf (ZE0/AZE0 2011-2017)", "profile": "nissan/leaf-ze0.json"},
    {"id": "generic_obd2", "name": "Generic : OBD-II (any vehicle)", "profile": "generic_obd2.json"}
  ]
}
```

- `version` — integer, currently `1`.
- `vehicles[]` — non-empty list.
  - `id` — unique lowercase slug, must match the profile's own `id`.
  - `name` — human-readable model name.
  - `profile` — repo-relative path the ESP32 appends to the raw base URL.

## Profile format

```json
{
  "id": "nissan_leaf_ze0",
  "car_model": "Nissan : Leaf (ZE0/AZE0 2011-2017)",
  "comment": "optional free text",
  "bus_speed": 500000,
  "tx":   [ ... ],
  "poll": [ ... ],
  "rx":   [ ... ]
}
```

| field | type | meaning |
|---|---|---|
| `id` | string | unique lowercase slug, must match the index |
| `car_model` | string | human-readable name |
| `comment` | string | optional, free text (profiles use it to quote the ported C++ logic) |
| `bus_speed` | int | optional, CAN baud rate, default `500000` |
| `tx` | list | periodic transmit frames (keep-alive) |
| `poll` | list | periodic UDS/OBD-style request frames |
| `rx` | list | passive decode rules |

### `tx[]` — periodic transmit

```json
{
  "id": "0x1D4",
  "data": "6E 0E 80 00 00 00 00 00",
  "interval_ms": 100,
  "counter": {"byte": 4, "modulus": 4},
  "checksum": "leaf_sum8"
}
```

| field | type | meaning |
|---|---|---|
| `id` | hex string | CAN id, e.g. `"0x1D4"` |
| `data` | 8-byte hex | frame template, exactly `"AA BB CC DD EE FF GG HH"` |
| `interval_ms` | int | transmission period |
| `counter` | object | optional rolling counter |
| `checksum` | string | optional, see checksum types below |

`counter` semantics: before each transmission, the counter value `n`
(starting at 0) is written **raw** into the 1-based `byte` of the frame, then
`n = (n + 1) % modulus`. In other words byte `n` for a normal counter frame.

> **Leaf caveat:** some Nissan keep-alive frames do not store the counter raw
> (0x1D4 keeps it in bits 6–7 of byte 4) and pair it with checksum bytes that
> the original firmware hardcodes in switch statements (verified: neither
> `leaf_crc` nor `leaf_sum8` reproduces them). Those entries carry the exact
> per-step byte tables in their `comment` and must be implemented by the
> firmware as rotating frame templates. See `nissan/leaf-ze0.json`.

### `poll[]` — periodic requests

```json
{
  "id": "0x79B",
  "data": "02 21 01 00 00 00 00 00",
  "interval_ms": 10000,
  "flow_control": "30 01 00 FF FF FF FF FF",
  "flow_control_id": "0x7BB"
}
```

Same fields as `tx[]` except no `counter`/`checksum`. `flow_control` is
optional: the frame the ESP32 transmits when a multi-frame response starts
arriving for this request's response id. (Leaf note: the firmware sends the
flow-control frame on **every** incoming response frame, not just the first.)

`flow_control_id` is optional (hex CAN id): the response id this poll's
flow-control frame serves. Default when absent: `request id + 8`. Use it when
responses arrive somewhere else (WiCAN `ATCRA` ids — e.g. Fiat/Stellantis
`0x694`, Nissan `0x7BB`); the flow-control frame itself is still transmitted
on the poll's request id. Firmware that predates this key ignores it and
falls back to `request id + 8`.

> **Leaf caveat:** the C++ firmware rotates the group byte of the 0x79B
> request round-robin over 7 groups, one per 10 s tick. A static JSON frame
> can only ever request one group, so the firmware must rotate byte 3 through
> `[01, 02, 04, 06, 83, 84, 90]` itself (see the poll entry's comment).

### `rx[]` — passive decode rules

```json
{
  "id": "0x1DB",
  "mask": "FF 00 00 00 00 00 00 00",
  "match": "1F 00 00 00 00 00 00 00",
  "checksum": "leaf_crc",
  "isotp": true,
  "signals": [ ... ]
}
```

| field | type | meaning |
|---|---|---|
| `id` | hex string | CAN id to match |
| `mask` / `match` | 8-byte hex | optional; frame matches only if `(frame[i] & mask[i]) == match[i]` for all `i`. Default: mask `FF..FF`, match `00..00` (match every frame). |
| `checksum` | string | optional; frame is dropped if the check fails |
| `isotp` | bool | optional (default `false`); multi-frame ISO-TP response, see below |
| `signals` | list | signals to evaluate when the rule matches |

Multiple rules with the same `id` are allowed (the Leaf profile uses this for
muxed frames); all matching rules are evaluated.

**ISO-TP assembly semantics (`"isotp": true`):**

- **Multi-frame response:** the ESP32 assembles one logical payload from the
  consecutive frames: the first frame (PCI `0x1n`) contributes its bytes
  2..7 (i.e. the PCI byte 0 and the length byte 1 are skipped), every
  consecutive frame (PCI `0x2n`) contributes its bytes 1..7. `B1`..`Bn` in
  expressions index this assembled payload (max index 64 for expressions, 256
  for array signals — size the reassembly buffer accordingly).
- **Single-frame response** (PCI low nibble `0x0n`): no assembly takes place;
  `B1`..`B8` are the raw frame bytes, so the PCI byte sits at `B1`. The
  generic OBD-II profile relies on this: data byte A of a single-frame OBD
  response is `B4` (`B1`=PCI, `B2`=mode+0x40, `B3`=PID, `B4..`=data).
- For `isotp` rules, `mask`/`match` are evaluated against the **first frame**
  of the response (which for a first frame carrying `10 .. <group>` lets rules
  select a UDS group, e.g. Leaf 0x7BB byte 4).

### `signals[]`

Scalar signal (expression):

```json
{"name": "HV_A", "expr": "([B1:B2]-1024)*0.5", "unit": "A"}
```

Array signal (packed consecutive unsigned ints) — exactly one of `expr` /
`array`:

```json
{"name": "CELL_VOLTAGES_MV", "array": {"first_byte": 3, "count": 96, "bytes_per": 2, "factor": 1, "offset": 0, "order": "big"}, "unit": "mV"}
```

| field | type | meaning |
|---|---|---|
| `name` | string | `UPPER_SNAKE_CASE`, unique per profile |
| `unit` | string | unit suffix, may be `""` |
| `expr` | string | see expression grammar |
| `array.first_byte` | int | 1-based start byte of the first element |
| `array.count` | int | number of elements |
| `array.bytes_per` | 1 or 2 | bytes per element (unsigned) |
| `array.factor` | number | multiply (default `1`) |
| `array.offset` | number | add after scaling (default `0`) |
| `array.order` | `"big"`/`"little"` | byte order per element (default `"big"`); little = Intel |

Value extraction: `value = raw * factor + offset`.

### Expression grammar

Byte references `B1`..`Bn` (1-based; for isotp payloads `B1`..`B64` over the
assembled payload), big-endian unsigned byte ranges `[B1:B2]` (`a < b`,
max 8 bytes apart implicitly by the index limit), decimal constants
(`12`, `0.5`, `.25`), operators `+ - * / & | >>`, parentheses, unary minus.
Nothing else — no functions, no comparisons, no variables.

```
expr    := or_expr
or_expr := and_expr ("|" and_expr)*
and_expr:= shift_expr ("&" shift_expr)*
shift   := add_expr (">>" add_expr)*
add     := mul_expr (("+" | "-") mul_expr)*
mul     := unary (("*" | "/") unary)*
unary   := "-" unary | primary
primary := NUMBER | BYTE | RANGE | "(" expr ")"
```

Precedence, lowest to highest: `|` , `&` , `>>` , `+ -` , `* /` , unary `-`.
Evaluation is floating point; for `&` and `>>` operands are truncated to
unsigned integers first (as in the C++ firmware).

Sign-extension idiom (used for the Leaf 11-bit signed current): for an
n-bit unsigned value `v` with sign bit `1 << (n-1)`,
`v - ((v & (1 << (n-1))) >> (n-1)) * (1 << n)` yields the signed value.

`tools/validate_profiles.py` contains a reference recursive-descent parser
implementing exactly this grammar.

### Checksum types

| name | definition |
|---|---|
| `leaf_crc` | Nissan Leaf CRC-8, table-driven: `crc = 0; for j in 0..6: crc = crctable_nissan_leaf[(crc ^ data[j]) % 256]`; `data[7]` must equal `crc`, else the frame is dropped. The 256-byte table is in the firmware's `src/devboard/utils/common_functions.h`. (For `tx[]` the same computation fills byte 7.) |
| `leaf_sum8` | 8-bit additive checksum: `data[7] = (data[0] + ... + data[6]) & 0xFF`. **Not currently used by any profile** — the Leaf keep-alive frames that look like checksums actually use hardcoded per-step tables (see caveat above); the type is reserved for future profiles. |

## Firmware caveats — Nissan Leaf (things the schema cannot express)

These behaviors of the original C++ decoder (`src/battery/NISSAN-LEAF-BATTERY.cpp`)
must be implemented in firmware, not in JSON:

1. **TX/poll gating:** nothing is transmitted until the first `0x5BC` frame
   has been received (`battery_can_alive`).
2. **LeafSpy coexistence:** receiving any frame on `0x79B` means LeafSpy is
   polling — suppress all UDS polling for 10 s. Polling also starts disabled
   for ~30 s after boot (`stop_battery_query` / `hold_off_with_polling_10seconds`).
3. **UDS group rotation:** rotate poll byte 3 through
   `[01, 02, 04, 06, 83, 84, 90]`, one group per 10 s tick (the C++
   pre-increments, so the first poll is group `0x02`).
4. **Generation detection:** default ZE0; receiving `0x59E` switches to AZE0;
   receiving any of `0x380 / 0x5EB / 0x5BF / 0x1ED / 0x1C2` switches to ZE1.
   Generation selects the authoritative temperature source (ZE0: `0x5BC`
   `TEMP_AVG_C`; AZE0: `0x5C0` `TEMP_MIN_C`/`TEMP_MAX_C`) and whether the
   ZE1-only keep-alive frames (`0x355`, `0x3B8`, `0x5C5`, `0x626`, `0x5EC`)
   are sent.
5. **Keep-alive frame templates:** `0x1D4` (4-step, counter in bits 6–7 of
   byte 4 + hardcoded byte 8), `0x1F2` (20-step pattern), `0x50C` (4-step
   with hardcoded bytes 5–6), `0x50B` byte 7 = `0x20` while the `0x5C0`
   heater-mail flag is set, `0x3B8` byte 2 toggles `0xE8`/`0xC8` every send.
   Exact tables are in each `tx` entry's comment.
6. **Piecewise temperature conversion:** `Temp_fromRAW_to_F()` (13-segment
   piecewise-linear map, then `(F-320)*5/9` for °C) is not expressible in the
   grammar. The profile publishes raw `TEMP_RAW_1/3/4` (group 0x04);
   `TEMP_RAW_2` is never populated by the C++ either. Firmware should port
   `Temp_fromRAW_to_F()` verbatim.
7. **Omitted UDS groups:** group `0x06` (96 balancing-shunt bits) and groups
   `0x83`/`0x84`/`0x90` (part number, serial number, BMS ID — ASCII strings)
   have no signal representation (no bit-array or string type yet).
8. **Sentinel values:** `0x3FF` in `0x1DB` `HV_V` and `0x55B` `SOC` means
   "measurement unavailable" and is ignored by the C++ — the JSON rules
   publish the value as-is.
9. **Application-layer safety logic** in `update_values()` (GIDS < 10 ⇒ SOC 0
   and discharge 0; full-charge flag ⇒ charge 0; capacity-empty or relay-cut ⇒
   power limits 0) is energy-management policy, not decoding, and is not
   ported into the profile.
10. **HV_W / IS_CHARGING** are not signals in the C++ decoder (power limits
    come from `0x1DC`); they are intentionally absent from the profile.

## WiCAN compatibility (converted profiles)

The `wican/` tree holds profiles converted from the
[meatpiHQ/wican-fw](https://github.com/meatpiHQ/wican-fw) `vehicle_profiles`
collection (an ELM327/OBD-II command-script format) by
`tools/convert_wican.py`:

```
python tools/convert_wican.py ["<path to wican-fw>/vehicle_profiles"]
```

The converter is mechanical and re-runnable; per-profile and per-signal
decisions (dropped pids, assumed defaults, flow-control warnings) are
recorded in `CONVERSION-NOTES.md`. WiCAN's `extends` chains are resolved with
the same merge semantics as wican-fw's own tooling.

### Byte-index re-basing

WiCAN expressions are 0-based over the ELM327 response buffer, which keeps
every frame's PCI byte. They are re-based to this schema's ISO-TP semantics
(see above): single-frame responses +1 (raw frame, PCI at `B1`), multi-frame
responses map onto the assembled payload (`FF` bytes 2..7 + `CF` bytes 1..7).
The frame layout per pid comes from the trailing digit of the WiCAN pid
string (an ELM327 expected-frame-count hint: odd-length pid strings), and is
inferred from the highest referenced byte when the digit is absent.

### Supported ELM327 subset

| command | mapping |
|---|---|
| `ATSP6` / `ATSP7` | `bus_speed` 500000 (11- vs 29-bit is derived from the id value) |
| `ATSP8` / `ATSP9` / `ATSPA` | `bus_speed` 250000 |
| `ATSHxxxx` | poll `id` (last one wins across profile `init` + per-pid `pid_init`) |
| `ATCRAxxxx` | rx rule `id` (default: request id + 8; `ATSH 7DF` defaults to rx `0x7E8`); when it differs from request id + 8 the poll entry also gets `flow_control_id` = response id |
| `ATFCSD<hex>` | poll `flow_control` data (zero-padded to 8 bytes; default `30 01 00 FF FF FF FF FF`) |
| pid hex string | poll `data`, with ISO-TP single-frame PCI length byte prepended |
| `ATE0/L0/H0/H1/S0/M0/ATATx/ATCAFx/ATSTxx/ATR1/STCAFCP...` | ignored (formatting/timeout/auto-FC-pair hints) |

WiCAN extras carried over: signal `class` (Home Assistant device class) is
kept as an extra signal key (the firmware ignores unknown keys); units are
looked up from wican's `params.json`.

### Not supported (profiles skipped, see CONVERSION-NOTES.md)

- `ATCEA` / `ATTAF` / `ATCERF` — ISO-TP extended addressing (needs firmware support)
- `ATCP` — CAN priority-bit addressing forms
- `ATSP0`–`ATSP5` — auto-detect / non-CAN protocols (J1850, K-line, KWP2000)
- Seed-key / `ATR`-style security-session handshakes
- 29-bit request ids without `ATCRA` (response id not mechanically derivable)
- Signals addressing assembled-payload bytes beyond `B64` (large cell-voltage
  dumps) — the schema's `array` signal type can reach `B256` if a hand-port is
  wanted

Flow-control delivery for responses that do not arrive on `request id + 8`
(WiCAN `ATCRA` ids): 30 converted profiles carry the optional
`poll.flow_control_id` key (= response id) so the firmware can deliver flow
control to them. Firmware that supports the key handles them fully; older
firmware falls back to `request id + 8`, in which case those profiles'
single-frame pids still work and only multi-frame pids stall. A handful of
pids with a genuinely different `ATFCSH` (flow-control *transmit* id, e.g.
Hyundai Ioniq 9, Mazda EZ-6) remain noted in `CONVERSION-NOTES.md` — the
schema has no separate FC-transmit-id field.

## Adding a vehicle

1. Copy an existing profile as a starting point.
2. Give it a unique `id` (lowercase slug) and a `car_model` name.
3. Add `{"id", "name", "profile"}` to `vehicle_profiles.json` with the
   repo-relative path.
4. Run `python tools/validate_profiles.py` — everything must pass.
5. Keep `comment` fields quoting the source decoder's logic wherever a
   scaling or bit layout is non-obvious.
