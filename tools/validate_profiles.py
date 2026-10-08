#!/usr/bin/env python3
"""Validate all vehicle profiles in this repo against the profile schema.

Usage:
    python tools/validate_profiles.py        (from the repo root, any CWD)

Stdlib only. The expression parser below implements EXACTLY the grammar the
ESP32 firmware implements; it doubles as the reference implementation for the
firmware porter. Exit code 0 = all profiles valid, 1 = one or more errors.
"""

import json
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# Limits
# ---------------------------------------------------------------------------
MAX_FRAME_BYTES = 8        # bytes per classic CAN frame
MAX_ISOTP_EXPR_BYTES = 64  # max byte index usable in an expr on an isotp payload
MAX_ISOTP_ARRAY_BYTES = 256  # max byte index reachable by an array on an isotp payload
MAX_CAN_ID = 0x1FFFFFFF

HEX_CAN_ID_RE = re.compile(r"^0[xX][0-9A-Fa-f]{1,8}$")
HEX_BYTE_RE = re.compile(r"^[0-9A-Fa-f]{2}$")
SLUG_RE = re.compile(r"^[a-z0-9][a-z0-9_-]*$")  # dashes allowed (WiCAN-style ids like "toyota--bz4x")
SIGNAL_NAME_RE = re.compile(r"^[A-Z][A-Z0-9]*(_[A-Z0-9]+)*$")
TX_CHECKSUMS = ("leaf_sum8", "leaf_crc")
RX_CHECKSUMS = ("leaf_crc",)
ARRAY_ORDERS = ("big", "little")

errors = []


def err(ctx, msg):
    errors.append("%s: %s" % (ctx, msg))


# ---------------------------------------------------------------------------
# Expression parser — grammar (reference implementation, mirrors ESP32 code):
#
#   expr    := or_expr
#   or_expr := and_expr ("|" and_expr)*
#   and_expr:= shift_expr ("&" shift_expr)*
#   shift   := add_expr (">>" add_expr)*
#   add     := mul_expr (("+" | "-") mul_expr)*
#   mul     := unary (("*" | "/") unary)*
#   unary   := "-" unary | primary
#   primary := NUMBER | BYTE | RANGE | "(" expr ")"
#   BYTE    := "B" 1..max_byte            (1-based byte, unsigned 8-bit value)
#   RANGE   := "[" "B" a ":" "B" b "]"    (big-endian unsigned, a <= b, both
#                                          within 1..max_byte)
#
# Precedence (lowest to highest): | , & , >> , + - , * / , unary - , primary.
# Evaluation is floating point; "&" and ">>" truncate their operands to
# unsigned integers first (firmware: cast to uint32).
# ---------------------------------------------------------------------------

class ExprError(Exception):
    pass


TOKEN_RE = re.compile(r"""
    (?P<space>\s+)
  | (?P<range>\[\s*B(?P<r1>\d+)\s*:\s*B(?P<r2>\d+)\s*\])
  | (?P<byte>B(?P<b>\d+))
  | (?P<num>(?:\d+\.\d+|\d+|\.\d+))
  | (?P<op>>>|[-+*/&|()])
""", re.VERBOSE)


def _tokenize(text):
    tokens = []
    pos = 0
    while pos < len(text):
        m = TOKEN_RE.match(text, pos)
        if not m:
            raise ExprError("unexpected character %r" % text[pos])
        pos = m.end()
        kind = m.lastgroup
        if kind == "space":
            continue
        if kind == "range":
            tokens.append(("range", (int(m.group("r1")), int(m.group("r2")))))
        elif kind == "byte":
            tokens.append(("byte", int(m.group("b"))))
        elif kind == "num":
            tokens.append(("num", float(m.group("num"))))
        else:
            tokens.append(("op", m.group("op")))
    tokens.append(("end", None))
    return tokens


class ExprParser:
    def __init__(self, text, max_byte):
        self.tokens = _tokenize(text)
        self.pos = 0
        self.max_byte = max_byte

    def peek(self):
        return self.tokens[self.pos]

    def next(self):
        tok = self.tokens[self.pos]
        self.pos += 1
        return tok

    def expect_op(self, *ops):
        kind, val = self.peek()
        if kind != "op" or val not in ops:
            raise ExprError("expected one of %s, got %r" % ("/".join(ops), val if kind != "end" else "end of expression"))
        return self.next()[1]

    def parse(self):
        node = self.parse_or()
        kind, val = self.peek()
        if kind != "end":
            raise ExprError("trailing token %r" % (val,))
        return node

    def parse_or(self):
        node = self.parse_and()
        while self.peek() == ("op", "|"):
            self.next()
            node = ("bin", "|", node, self.parse_and())
        return node

    def parse_and(self):
        node = self.parse_shift()
        while self.peek() == ("op", "&"):
            self.next()
            node = ("bin", "&", node, self.parse_shift())
        return node

    def parse_shift(self):
        node = self.parse_add()
        while self.peek() == ("op", ">>"):
            self.next()
            node = ("bin", ">>", node, self.parse_add())
        return node

    def parse_add(self):
        node = self.parse_mul()
        while self.peek() in (("op", "+"), ("op", "-")):
            op = self.next()[1]
            node = ("bin", op, node, self.parse_mul())
        return node

    def parse_mul(self):
        node = self.parse_unary()
        while self.peek() in (("op", "*"), ("op", "/")):
            op = self.next()[1]
            node = ("bin", op, node, self.parse_unary())
        return node

    def parse_unary(self):
        if self.peek() == ("op", "-"):
            self.next()
            return ("neg", self.parse_unary())
        return self.parse_primary()

    def check_index(self, i, ctx):
        if i < 1:
            raise ExprError("byte index B%d out of range (1-based)" % i)
        if i > self.max_byte:
            raise ExprError("byte index B%d exceeds maximum B%d %s"
                            % (i, self.max_byte, ctx))

    def parse_primary(self):
        kind, val = self.next()
        if kind == "num":
            return ("num", val)
        if kind == "byte":
            self.check_index(val, "for this frame/payload")
            return ("byte", val)
        if kind == "range":
            a, b = val
            self.check_index(a, "for this frame/payload")
            self.check_index(b, "for this frame/payload")
            if a >= b:
                raise ExprError("range [B%d:B%d] must have first byte < second byte" % (a, b))
            return ("range", val)
        if kind == "op" and val == "(":
            node = self.parse_or()
            self.expect_op(")")
            return node
        raise ExprError("unexpected token %r" % (val,))


def validate_expr(text, max_byte, ctx):
    try:
        ExprParser(text, max_byte).parse()
    except ExprError as e:
        err(ctx, "invalid expr %r: %s" % (text, e))


# ---------------------------------------------------------------------------
# Low-level field validators
# ---------------------------------------------------------------------------

def validate_can_id(value, ctx):
    if not isinstance(value, str) or not HEX_CAN_ID_RE.match(value):
        err(ctx, "id %r must be a hex string like '0x1DB'" % (value,))
        return
    if int(value, 16) > MAX_CAN_ID:
        err(ctx, "id %s exceeds 0x%X" % (value, MAX_CAN_ID))


def validate_data8(value, ctx, field="data"):
    """8 bytes of uppercase/lowercase hex separated by single spaces."""
    if not isinstance(value, str):
        err(ctx, "%s must be a string of 8 hex bytes" % field)
        return
    parts = value.split(" ")
    if len(parts) != MAX_FRAME_BYTES or any(not HEX_BYTE_RE.match(p) for p in parts):
        err(ctx, "%s must be exactly 8 hex bytes 'AA BB CC DD EE FF GG HH', got %r"
            % (field, value))


def validate_interval(value, ctx):
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        err(ctx, "interval_ms must be a positive integer, got %r" % (value,))


def validate_counter(counter, ctx):
    if not isinstance(counter, dict):
        err(ctx, "counter must be an object")
        return
    byte = counter.get("byte")
    modulus = counter.get("modulus")
    if not isinstance(byte, int) or not 1 <= byte <= MAX_FRAME_BYTES:
        err(ctx, "counter.byte must be 1..%d, got %r" % (MAX_FRAME_BYTES, byte))
    if not isinstance(modulus, int) or not 2 <= modulus <= 255:
        err(ctx, "counter.modulus must be 2..255, got %r" % (modulus,))


# ---------------------------------------------------------------------------
# Signal / rule validators
# ---------------------------------------------------------------------------

def validate_signal(sig, isotp, ctx, seen_names):
    if not isinstance(sig, dict):
        err(ctx, "signal must be an object")
        return
    name = sig.get("name")
    if not isinstance(name, str) or not SIGNAL_NAME_RE.match(name):
        err(ctx, "signal name %r must be UPPER_SNAKE_CASE" % (name,))
    elif name in seen_names:
        err(ctx, "duplicate signal name %r (must be unique per profile)" % name)
    else:
        seen_names.add(name)

    if "unit" not in sig or not isinstance(sig.get("unit"), str):
        err(ctx, "signal %r must have a string 'unit' (may be empty)" % (name,))

    has_expr = "expr" in sig
    has_array = "array" in sig
    if has_expr == has_array:
        err(ctx, "signal %r must have exactly one of 'expr' or 'array'" % (name,))
        return

    max_expr_byte = MAX_ISOTP_EXPR_BYTES if isotp else MAX_FRAME_BYTES
    max_array_byte = MAX_ISOTP_ARRAY_BYTES if isotp else MAX_FRAME_BYTES

    if has_expr:
        if not isinstance(sig["expr"], str) or not sig["expr"].strip():
            err(ctx, "signal %r expr must be a non-empty string" % (name,))
        else:
            validate_expr(sig["expr"], max_expr_byte, ctx + " signal " + str(name))
    else:
        arr = sig["array"]
        if not isinstance(arr, dict):
            err(ctx, "signal %r array must be an object" % (name,))
            return
        first = arr.get("first_byte")
        count = arr.get("count")
        bytes_per = arr.get("bytes_per", 1)
        order = arr.get("order", "big")
        for key in ("factor", "offset"):
            if key in arr and not isinstance(arr[key], (int, float)):
                err(ctx, "signal %r array.%s must be numeric" % (name, key))
        if not isinstance(first, int) or first < 1:
            err(ctx, "signal %r array.first_byte must be >= 1" % (name,))
        elif not isinstance(count, int) or count < 1:
            err(ctx, "signal %r array.count must be >= 1" % (name,))
        elif bytes_per not in (1, 2):
            err(ctx, "signal %r array.bytes_per must be 1 or 2" % (name,))
        elif order not in ARRAY_ORDERS:
            err(ctx, "signal %r array.order must be 'big' or 'little'" % (name,))
        else:
            last = first + count * bytes_per - 1
            if last > max_array_byte:
                err(ctx, "signal %r array spans B%d..B%d, exceeding B%d %s"
                    % (name, first, last, max_array_byte, "for this frame/payload"))


def validate_rx_rule(rule, ctx, seen_names):
    if not isinstance(rule, dict):
        err(ctx, "rx rule must be an object")
        return
    validate_can_id(rule.get("id"), ctx)
    if "mask" in rule:
        validate_data8(rule["mask"], ctx, "mask")
    if "match" in rule:
        validate_data8(rule["match"], ctx, "match")
    if "mask" in rule and "match" not in rule:
        err(ctx, "mask present without match")
    if "checksum" in rule and rule["checksum"] not in RX_CHECKSUMS:
        err(ctx, "rx checksum %r must be one of %s" % (rule["checksum"], RX_CHECKSUMS))
    isotp = rule.get("isotp", False)
    if not isinstance(isotp, bool):
        err(ctx, "isotp must be a boolean")
        isotp = False
    signals = rule.get("signals", [])
    if not isinstance(signals, list):
        err(ctx, "signals must be a list")
        return
    for i, sig in enumerate(signals):
        validate_signal(sig, isotp, "%s rx[%d]" % (ctx, i), seen_names)


# ---------------------------------------------------------------------------
# Profile validator
# ---------------------------------------------------------------------------

def validate_profile(path):
    rel = os.path.relpath(path, REPO_ROOT)
    try:
        with open(path, "r", encoding="utf-8") as f:
            prof = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        err(rel, "cannot load/parse: %s" % e)
        return 0

    ctx = rel
    pid = prof.get("id")
    if not isinstance(pid, str) or not SLUG_RE.match(pid):
        err(ctx, "id %r must be a lowercase slug" % (pid,))
    if not isinstance(prof.get("car_model"), str) or not prof["car_model"]:
        err(ctx, "car_model must be a non-empty string")
    if "comment" in prof and not isinstance(prof["comment"], str):
        err(ctx, "comment must be a string")
    speed = prof.get("bus_speed", 500000)
    if not isinstance(speed, int) or speed <= 0:
        err(ctx, "bus_speed must be a positive integer")

    for i, tx in enumerate(prof.get("tx", [])):
        tctx = "%s tx[%d]" % (ctx, i)
        if not isinstance(tx, dict):
            err(tctx, "tx entry must be an object")
            continue
        validate_can_id(tx.get("id"), tctx)
        validate_data8(tx.get("data"), tctx, "data")
        validate_interval(tx.get("interval_ms"), tctx)
        if "counter" in tx:
            validate_counter(tx["counter"], tctx)
        if "checksum" in tx and tx["checksum"] not in TX_CHECKSUMS:
            err(tctx, "tx checksum %r must be one of %s" % (tx["checksum"], TX_CHECKSUMS))

    for i, poll in enumerate(prof.get("poll", [])):
        pctx = "%s poll[%d]" % (ctx, i)
        if not isinstance(poll, dict):
            err(pctx, "poll entry must be an object")
            continue
        validate_can_id(poll.get("id"), pctx)
        validate_data8(poll.get("data"), pctx, "data")
        validate_interval(poll.get("interval_ms"), pctx)
        if "flow_control" in poll:
            validate_data8(poll["flow_control"], pctx, "flow_control")
        if "flow_control_id" in poll:
            # optional: response CAN id this poll's flow control serves
            # (default request id + 8); old firmware ignores the key
            validate_can_id(poll["flow_control_id"], pctx)

    seen_names = set()
    signal_count = 0
    for i, rule in enumerate(prof.get("rx", [])):
        before = len(seen_names)
        validate_rx_rule(rule, ctx, seen_names)
        signal_count += len(rule.get("signals", [])) if isinstance(rule, dict) else 0
    return signal_count


# ---------------------------------------------------------------------------
# Index validator
# ---------------------------------------------------------------------------

def main():
    index_path = os.path.join(REPO_ROOT, "vehicle_profiles.json")
    try:
        with open(index_path, "r", encoding="utf-8") as f:
            index = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print("FATAL: cannot load vehicle_profiles.json: %s" % e)
        return 1

    if index.get("version") != 1:
        err("vehicle_profiles.json", "version must be 1, got %r" % (index.get("version"),))
    vehicles = index.get("vehicles")
    if not isinstance(vehicles, list) or not vehicles:
        err("vehicle_profiles.json", "vehicles must be a non-empty list")
        vehicles = []

    seen_ids = set()
    total_signals = 0
    for entry in vehicles:
        ectx = "vehicle_profiles.json entry %r" % (entry.get("id") if isinstance(entry, dict) else entry)
        if not isinstance(entry, dict):
            err(ectx, "entry must be an object")
            continue
        vid = entry.get("id")
        if not isinstance(vid, str) or not SLUG_RE.match(vid):
            err(ectx, "id %r must be a lowercase slug" % (vid,))
        elif vid in seen_ids:
            err(ectx, "duplicate vehicle id %r" % vid)
        else:
            seen_ids.add(vid)
        if not isinstance(entry.get("name"), str) or not entry["name"]:
            err(ectx, "name must be a non-empty string")
        rel_path = entry.get("profile")
        if not isinstance(rel_path, str) or not rel_path:
            err(ectx, "profile must be a repo-relative path")
            continue
        profile_path = os.path.join(REPO_ROOT, rel_path)
        if not os.path.isfile(profile_path):
            err(ectx, "profile file %r does not exist" % rel_path)
            continue
        n_signals = validate_profile(profile_path)
        total_signals += n_signals
        # profile's own id must match the index id
        try:
            with open(profile_path, "r", encoding="utf-8") as f:
                prof_id = json.load(f).get("id")
            if prof_id != vid:
                err(ectx, "profile id %r does not match index id %r" % (prof_id, vid))
        except (OSError, json.JSONDecodeError):
            pass  # already reported by validate_profile
        if not errors:
            print("OK  %-28s %s" % (vid, rel_path))

    print("-" * 60)
    if errors:
        for e in errors:
            print("ERROR: " + e)
        print("%d error(s), %d vehicle(s), %d signal(s) checked" % (len(errors), len(vehicles), total_signals))
        return 1
    print("All %d vehicle profile(s) valid, %d signal(s) total." % (len(vehicles), total_signals))
    return 0


if __name__ == "__main__":
    sys.exit(main())
