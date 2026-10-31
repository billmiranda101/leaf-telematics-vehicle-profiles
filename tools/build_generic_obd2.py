#!/usr/bin/env python3
"""Generate generic_obd2.json from the OBDex Mode 01 PID dataset.

Source: https://foerbsnavi.github.io/OBDex/pids/mode01.json (CC0-1.0).
OBDex documents the standard SAE J1979 Mode 01 PIDs with formulas in
"A, B, C, D, E" data-byte notation. This script translates each eligible
PID into the leaf-telematics profile schema:

  - request frame: 02 01 <PID> 00 00 00 00 00 on the functional 0x7DF
  - single-frame response: B1 = PCI, B2 = 0x41, B3 = PID, B4.. = data A..
    (so OBDex byte A -> B4, B -> B5, C -> B6, D -> B7, E -> B8)
  - formula: A..E single-letter variables become firmware Bn byte refs
    (the firmware grammar supports + - * / ( ) and decimal constants only,
    which covers every arithmetic formula in the dataset)

PIDs that need more than 5 data bytes (multi-frame ISO-TP: 78, 79, 68)
or are bitmap/encoded/enumerated (no numeric formula: 00, 01, 02, 03,
12, 13, 1C, 1D, 1E, 20, 40, 41, 4F, 51, 5F, 60, 65, 7C, 7D, 7E, 80, 83)
are skipped - they cannot be expressed as a single scalar signal.

PIDs with multiple ';'-separated signals (O2 voltage+trim, lambda+current,
per-bank trims, multi-sensor frames) become one rx rule with several
signals; the parenthetical annotation ("(voltage)", "(Bank 1)", ...) is
used for the signal-name suffix and unit selection.

Intervals are tiered to keep bus load sane: core driving parameters at
1000 ms, everything else at 5000 ms.

Usage (from repo root):
    python tools/build_generic_obd2.py                # fetch OBDex live
    python tools/build_generic_obd2.py mode01.json    # use a local copy

Then review + commit the regenerated generic_obd2.json.
"""

import json
import os
import re
import sys
import urllib.request

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(REPO_ROOT, "generic_obd2.json")
OBDEX_URL = "https://foerbsnavi.github.io/OBDex/pids/mode01.json"

REQUEST_ID = "0x7DF"
RESPONSE_ID = "0x7E8"

# Core driving parameters: poll at 1000 ms; everything else at 5000 ms.
FAST_PIDS = {"04", "05", "0B", "0C", "0D", "0F", "10", "11", "2F"}
FAST_INTERVAL_MS = 1000
SLOW_INTERVAL_MS = 5000

# Annotation keyword -> (signal-name suffix, unit) for multi-signal formulas.
ANNOTATION_MAP = {
    "voltage": ("VOLTAGE", "V"),
    "trim": ("TRIM", "%"),
    "lambda": ("LAMBDA", "ratio"),
    "current": ("CURRENT", "mA"),
    "idle": ("IDLE", "%"),
    "point2": ("POINT2", "%"),
    "point3": ("POINT3", "%"),
    "point4": ("POINT4", "%"),
    "point5": ("POINT5", "%"),
    "bank 1": ("BANK_1", "%"),
    "bank 2": ("BANK_2", "%"),
    "bank 3": ("BANK_3", "%"),
    "bank 4": ("BANK_4", "%"),
    "sensor 1": ("SENSOR_1", None),   # unit inherited from PID
    "sensor 2": ("SENSOR_2", None),
    "sensor a": ("SENSOR_A", None),
    "sensor b": ("SENSOR_B", None),
    "engine fuel rate": ("ENGINE", "g/s"),
    "vehicle fuel rate": ("VEHICLE", "g/s"),
}

UNIT_NORMALIZE = {
    "°C": "degC",
    "°": "deg",
}


def load_mode01():
    if len(sys.argv) > 1:
        with open(sys.argv[1], "r", encoding="utf-8") as f:
            return json.load(f)
    with urllib.request.urlopen(OBDEX_URL, timeout=30) as r:
        return json.loads(r.read().decode("utf-8"))


def normalize_unit(unit):
    if not unit:
        return ""
    # Multi-signal units like "V; %" are resolved per signal via annotations.
    unit = unit.split(";")[0].strip()
    return UNIT_NORMALIZE.get(unit, unit)


# ---------------------------------------------------------------------------
# Formula translation (OBDex A..E -> firmware B4..B8)
# ---------------------------------------------------------------------------

_VAR_RE = re.compile(r"\b([A-E])\b")

# Every token the firmware grammar allows, after variable substitution.
_EXPR_TOKEN_RE = re.compile(r"""
    \s*
    (?:
        \d+\.\d+ | \d+ | \.\d+      # decimal constant
      | B(?:[1-9]|[1-5]\d?)          # byte ref (B1..B59 isotp payload)
      | [+\-*/&|()]
      | >>
    )
""", re.VERBOSE)


def translate_formula(formula):
    """Translate an OBDex formula to firmware expr syntax.

    Returns the translated expr, or None if the formula uses constructs the
    firmware grammar cannot represent (bit tests with hex, functions, etc.).
    """
    expr = _VAR_RE.sub(lambda m: "B" + str(4 + (ord(m.group(1)) - ord("A"))),
                       formula)
    # Verify the result is fully inside the firmware grammar.
    pos = 0
    while pos < len(expr):
        m = _EXPR_TOKEN_RE.match(expr, pos)
        if not m:
            return None
        pos = m.end()
    return expr


# ---------------------------------------------------------------------------
# Signal naming
# ---------------------------------------------------------------------------

def base_name(name_en, multi=False):
    # Drop any em-dash tail ("... — voltage and short term fuel trim"):
    # the per-signal suffix carries that information in multi-signal PIDs.
    s = re.split(r"[—–]", name_en)[0]
    if multi:
        # Parenthetical detail ("(A and B)", "(engine and vehicle, in g/s)")
        # is redundant with the per-signal suffix; drop it to keep names short.
        s = re.sub(r"\(.*?\)", " ", s)
    s = s.upper()
    s = re.sub(r"[()/]", " ", s)
    s = re.sub(r"[^A-Z0-9]+", "_", s).strip("_")
    return s


# Trailing "(...)" is only an annotation when its content is one of the
# known annotation keywords; "(256 * A + B)" is math, not an annotation.
_ANNOTATION_KEYS = tuple(ANNOTATION_MAP.keys())

_ANNOTATION_RE = re.compile(r"^(.*)\(([^()]*)\)\s*$")


def split_multi_formula(formula):
    """Split a ';'-separated multi-signal formula into (expr, annotation).

    "A/200 (voltage); 100/128 * B - 100 (trim)"
      -> [("A/200", "voltage"), ("100/128 * B - 100", "trim")]
    "10 * (256 * A + B)" (single signal, math parens) -> [("10 * (256 * A + B)", None)]
    """
    parts = []
    for part in formula.split(";"):
        part = part.strip()
        m = _ANNOTATION_RE.match(part)
        if m and any(k in m.group(2).strip().lower() for k in _ANNOTATION_KEYS):
            parts.append((m.group(1).strip(), m.group(2).strip().lower()))
        else:
            parts.append((part, None))
    return parts


def build_pid(pid_entry):
    """Build (poll_entry, rx_rule) for one OBDex PID, or None to skip."""
    pid_hex = pid_entry["pid"].upper()
    name_en = pid_entry.get("name", {}).get("en", "PID " + pid_hex)
    formula = pid_entry.get("formula")
    nbytes = pid_entry.get("bytes", 0)

    if not formula:
        return None, "bitmap/encoded (no numeric formula)"
    if nbytes > 5:
        return None, "needs %d data bytes (multi-frame, max 5)" % nbytes

    pid_byte = int(pid_hex, 16)
    request = "02 01 %02X 00 00 00 00 00" % pid_byte
    match = "00 41 %02X 00 00 00 00 00" % pid_byte
    interval = FAST_INTERVAL_MS if pid_hex in FAST_PIDS else SLOW_INTERVAL_MS

    signals = []
    parts = split_multi_formula(formula)
    multi = len(parts) > 1
    base = base_name(name_en, multi=multi)
    pid_unit = normalize_unit(pid_entry.get("unit", ""))
    for expr_src, annotation in parts:
        expr = translate_formula(expr_src)
        if expr is None:
            return None, "untranslatable formula %r" % expr_src
        suffix, ann_unit = (None, None)
        if annotation:
            for key, val in ANNOTATION_MAP.items():
                if key in annotation:
                    suffix, ann_unit = val
                    break
        if multi:
            if suffix is None:
                suffix = base_name(annotation) if annotation else "S%d" % (len(signals) + 1)
            sig_name = (base + "_" + suffix).strip("_")[:48].rstrip("_")
        else:
            sig_name = base[:48].rstrip("_")
        signals.append({
            "name": sig_name,
            "expr": expr,
            "unit": ann_unit or pid_unit,
        })

    # Keep signal names unique even if two PIDs derive the same base name.
    return ({
        "id": REQUEST_ID,
        "data": request,
        "interval_ms": interval,
        "comment": "Mode 01 PID 0x%s - %s." % (pid_hex, name_en),
    }, {
        "id": RESPONSE_ID,
        "isotp": True,
        "mask": "00 FF FF 00 00 00 00 00",
        "match": match,
        "comment": "Single-frame response to 01%s: B1 = PCI, B2 = 0x41, B3 = PID, B4.. = data (%s)." % (pid_hex, formula),
        "signals": signals,
    })


def main():
    pids = load_mode01()
    pids = [p for p in pids if p.get("mode") == "01"]
    pids.sort(key=lambda p: int(p["pid"], 16))

    polls, rules, skipped = [], [], []
    used_names = set()
    for p in pids:
        poll, rule = build_pid(p)
        if poll is None:
            skipped.append((p["pid"], rule))
            continue
        # Uniqueness pass over signal names.
        for sig in rule["signals"]:
            name = sig["name"]
            n, i = name, 2
            while n in used_names:
                n = ("%s_%X" % (name[:43].rstrip("_"), i))[:48]
                i += 1
            sig["name"] = n
            used_names.add(n)
        polls.append(poll)
        rules.append(rule)

    profile = {
        "id": "generic_obd2",
        "car_model": "Generic : OBD-II (any vehicle)",
        "comment": (
            "Generic OBD-II profile for ICE vehicles, generated from the OBDex "
            "Mode 01 PID dataset (https://github.com/foerbsnavi/obdex, CC0-1.0) "
            "by tools/build_generic_obd2.py. Polls %d standard PIDs on the "
            "functional broadcast address 0x7DF with single-frame responses "
            "decoded from 0x7E8 (ECU #1). BYTE OFFSETS (single-frame response): "
            "B1 = PCI byte (ISO-15765 single-frame header), B2 = 0x41 (mode 01 "
            "+ 0x40), B3 = PID, B4.. = data bytes (A, B, ...). The mask/match "
            "on B2:B3 (0x41:PID) also rejects multi-frame responses, which is "
            "intentional: all polled PIDs answer in a single frame. Intervals "
            "are tiered (core driving parameters 1000 ms, the rest 5000 ms); "
            "all requests share 0x7DF, so the firmware round-robins them to "
            "spread bus load. PIDs that are bitmaps/enumerations or need "
            "multi-frame responses (O2 location bitmaps, supported-PID "
            "bitmaps, EGT bank frames) are intentionally omitted."
        ) % len(polls),
        "bus_speed": 500000,
        "tx": [],
        "poll": polls,
        "rx": rules,
    }

    with open(OUT_PATH, "w", encoding="utf-8") as f:
        json.dump(profile, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print("Wrote %s: %d PIDs polled, %d rx rules, %d signals"
          % (os.path.relpath(OUT_PATH, REPO_ROOT), len(polls), len(rules),
             sum(len(r["signals"]) for r in rules)))
    if skipped:
        print("\nSkipped %d PIDs:" % len(skipped))
        for pid_hex, why in skipped:
            print("  %3s  %s" % (pid_hex, why))
    return 0


if __name__ == "__main__":
    sys.exit(main())
