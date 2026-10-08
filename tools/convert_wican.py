#!/usr/bin/env python3
"""Convert meatpiHQ/wican-fw vehicle_profiles to the leaf-telematics schema.

Mechanical converter: reads every <make>/<profile>.json under the wican-fw
vehicle_profiles folder and writes converted profiles to wican/<make>/ in this
repo, updates vehicle_profiles.json and regenerates CONVERSION-NOTES.md.

WiCAN -> schema mapping
-----------------------
WiCAN profiles are ELM327 command scripts:

  ATSP6/7/8/9/A  -> bus_speed (6/7 = 500000, 8/9/A = 250000; other ATSP -> skip)
  ATSHxxxx       -> poll request CAN id (11- or 29-bit hex; last one wins,
                    profile init + pid_init are applied in order per pid)
  ATCRAxxxx      -> rx (response) CAN id; default tx+8 (11-bit only; a 29-bit
                    request without ATCRA is skipped - response id not derivable)
  ATFCSD<hex>    -> flow_control data for that pid (padded to 8 bytes with 00;
                    default 30 01 00 FF FF FF FF FF)
  "pid" string   -> poll data. WiCAN pid strings carry a trailing digit when
                    their length is odd: it is the EXPECTED RESPONSE FRAME
                    COUNT (ELM327 optimization, see wican-fw elm327.c) and is
                    stripped; the request bytes get a PCI length byte prepended
                    (ISO-TP single frame) and zero-padding to 8 bytes.
  per-pid "parameters" {name: expr} -> rx signals {name, expr, unit, class}

Expression re-basing (the important part)
-----------------------------------------
WiCAN expressions are 0-based over the ELM327 response buffer, which is the
concatenation of every received frame's data INCLUDING each frame's PCI byte.
Our schema (see README "ISO-TP assembly semantics"):
  * single-frame response (PCI 0x0n): B1..B8 are the raw frame bytes (PCI at B1)
  * multi-frame response: assembled payload = FF bytes 2..7 + CF bytes 1..7
Frame layout per pid comes from the trailing digit (1 = single frame, 2..9 =
multi-frame). Pid strings without a digit (even length) are inferred: if any
expression references a byte index > 7 the response must be multi-frame,
otherwise single-frame is assumed (see CONVERSION-NOTES.md).

Transforms applied to expressions:
  B<n>            -> B<mapped>            (index re-based per layout)
  [B<a>:B<b>]     -> [B<mapped>:B<mapped>]
  B<n>:<d>        -> ((B<mapped>>>d)&1)   (WiCAN bit-select, not in our grammar)
  S<n>            -> ((B<m>)-((B<m>&128)>>7)*256)  (WiCAN signed byte)
  A<<k            -> A*(2**k)             (our grammar has >> but not <<)

Skips (recorded in CONVERSION-NOTES.md): profiles using ATCEA/ATTAF/ATCERF
(ISO-TP extended addressing), ATCP (priority addressing), non-CAN ATSP,
non-hex ids, and any signal whose transformed expression the schema validator
rejects (e.g. byte refs beyond B64 on an isotp payload).

Usage:
    python tools/convert_wican.py ["<path to wican-fw>/vehicle_profiles"]
"""

import copy
import json
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SRC = (r"C:/Users/BILLY ENGINEERING/Documents/PlatformIO/Projects/"
               r"wican-fw-main (1)/wican-fw-main/vehicle_profiles")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import validate_profiles as validator  # noqa: E402  (reference expr parser)

PROTOCOL_SPEED = {"6": 500000, "7": 500000, "8": 250000, "9": 250000,
                  "A": 250000, "a": 250000}
HEX_RE = re.compile(r"^[0-9A-Fa-f]+$")
SKIP_CMD_RE = re.compile(r"^(ATCEA|ATTAF|ATCERF|ATCP|ATPB|STN|STPX)",
                         re.IGNORECASE)
SLUG_RE = re.compile(r"[^a-z0-9]+")
DEFAULT_FC = "30 01 00 FF FF FF FF FF"

# AT commands that are formatting/timeout/noise for our purposes and are
# silently ignored (noted in CONVERSION-NOTES.md only when unusual).
BENIGN_CMDS = {"ATE0", "ATE1", "ATL0", "ATL1", "ATH0", "ATH1", "ATS0", "ATS1",
               "ATM0", "ATM1", "ATAT0", "ATAT1", "ATAT2", "ATCAF0", "ATCAF1",
               "ATR0", "ATR1", "ATST32", "ATST64", "ATST96", "ATZ", "ATIB10",
               "ATDP", "ATFI", "ATFE", "ATWM", "ATCSM0", "ATCSM1"}


class SkipProfile(Exception):
    pass


class Ctx(object):
    """Effective ELM327 state after applying init + pid_init commands."""
    def __init__(self):
        self.speed = None
        self.tx = None
        self.rx = None
        self.fcsd = None
        self.fcsh = None
        self.ignored = []

    def apply(self, cmd):
        up = cmd.upper()
        if up.startswith("ATSP"):
            p = up[4:]
            if p in PROTOCOL_SPEED:
                self.speed = p
            else:
                raise SkipProfile("unsupported protocol ATSP%s "
                                  "(only ISO-15765 CAN 6/7/8/9/A convert)" % p)
            return
        if up.startswith("ATSH"):
            h = up[4:]
            if not HEX_RE.match(h):
                raise SkipProfile("non-hex ATSH%s" % h)
            v = int(h, 16)
            if v == 0 or v > 0x1FFFFFFF:
                raise SkipProfile("ATSH%s out of CAN id range" % h)
            self.tx = v
            return
        if up.startswith("ATCRA"):
            h = up[5:]
            if not HEX_RE.match(h):
                raise SkipProfile("non-hex ATCRA%s" % h)
            v = int(h, 16)
            if v == 0 or v > 0x1FFFFFFF:
                raise SkipProfile("ATCRA%s out of CAN id range" % h)
            self.rx = v
            return
        if up.startswith("ATFCSD"):
            h = up[6:]
            if not HEX_RE.match(h) or len(h) % 2 != 0:
                raise SkipProfile("malformed ATFCSD%s" % h)
            self.fcsd = h
            return
        if up.startswith("ATFCSH"):
            # FC transmit header; recorded so genuinely differing values can
            # be flagged (the schema has no separate FC-transmit-id key).
            h = up[6:]
            if HEX_RE.match(h):
                self.fcsh = int(h, 16)
            return
        if SKIP_CMD_RE.match(up):
            raise SkipProfile("unsupported init command %s" % cmd)
        if up in BENIGN_CMDS or up.startswith("STCAFCP"):
            self.ignored.append(cmd)
            return
        self.ignored.append(cmd)  # unknown AT command: ignore, but record


# ---------------------------------------------------------------------------
# wican extends-chain merge (mirrors .vehicle_profiles/src/cars.js):
# walk chain root-first; a parameter name defined at a deeper level replaces
# the same name everywhere in the accumulated set; empty formulas delete.
# ---------------------------------------------------------------------------

def load_json(path):
    with open(path, "r", encoding="utf-8-sig") as f:
        return json.load(f)


def resolve_extends(relpath, files):
    chain = []
    seen = set()
    nxt = relpath
    while nxt is not None:
        if nxt in seen or nxt not in files:
            break
        seen.add(nxt)
        d = copy.deepcopy(files[nxt])
        chain.append(d)
        nxt = d.get("extends")

    car = copy.deepcopy(chain[0])
    car["pids"] = []
    for new_car in reversed(chain):  # root first, child last
        pids = new_car.get("pids") or []
        new_names = set()
        for p in pids:
            new_names.update((p.get("parameters") or {}).keys())
        for p in car["pids"]:
            for k in [k for k in (p.get("parameters") or {}) if k in new_names]:
                del p["parameters"][k]
        by_pid = {p["pid"]: p for p in car["pids"]}
        for p in pids:
            p = copy.deepcopy(p)
            if p["pid"] in by_pid:
                by_pid[p["pid"]].setdefault("parameters", {}).update(
                    p.get("parameters") or {})
            else:
                p.setdefault("parameters", {})
                car["pids"].append(p)
    for p in car["pids"]:
        for k in [k for k, v in p["parameters"].items() if not v]:
            del p["parameters"][k]
    car["pids"] = [p for p in car["pids"] if p["parameters"]]
    if not car.get("init"):
        for c in chain:
            if c.get("init"):
                car["init"] = c["init"]
                break
    return car


# ---------------------------------------------------------------------------
# Expression re-basing
# ---------------------------------------------------------------------------

def map_index(i, layout):
    """WiCAN 0-based index -> our 1-based byte index. None = not expressible."""
    if layout == "single":
        return i + 1
    f, o = divmod(i, 8)
    if f == 0:
        if o < 2:
            return None          # PCI or FF length byte: never addressable
        return o - 1
    if o == 0:
        return None              # consecutive-frame PCI byte
    return 7 * f + o - 1


# Single-pass tokenizer: alternation order matters. re.sub never rescans
# replacement text, so byte refs generated by one substitution (e.g. the
# little-endian expansion of a descending range) are never re-processed.
TOKEN_RE = re.compile(r"""
    \[S(\d+):S(\d+)\]                    # 1,2  signed range (big-endian, sign-extended)
  | \[B(\d+):B(\d+)\]                    # 3,4  range
  | (?<![B0-9)])B(\d+):(\d)(?![0-9])     # 5,6  bit select Bn:d
  | (?<![A-Z0-9)])S(\d+)(?![0-9])        # 7    signed byte Sn
  | <<\s*(\d+)                          # 8    shift left <<k
  | (?<![B0-9)])B(\d+)(?![0-9:])         # 9    plain byte Bn
""", re.VERBOSE)


def transform_expr(expr, layout):
    """Returns (transformed_expr, first_bad_index_or_None)."""
    bad = [None]

    def midx(i):
        r = map_index(int(i), layout)
        if r is None:
            bad[0] = int(i)
            return int(i)
        if r > 64:
            bad[0] = int(i)
        return r

    def repl(m):
        if m.group(1) is not None:
            # wican [Sa:Sb]: big-endian value sign-extended across 8*w bits
            a, b = int(m.group(1)), int(m.group(2))
            if a >= b:
                bad[0] = a
                return m.group(0)
            w = b - a + 1
            r = "[B%d:B%d]" % (midx(a), midx(b))
            sign = 1 << (8 * w - 1)
            return "((%s)-((%s&%d)>>%d)*%d)" % (r, r, sign, 8 * w - 1,
                                                1 << (8 * w))
        if m.group(3) is not None:
            a, b = int(m.group(3)), int(m.group(4))
            if a < b:
                return "[B%d:B%d]" % (midx(a), midx(b))
            if a == b:
                return "(B%d)" % midx(a)  # wican [Bn:Bn] == single byte
            # descending range: wican profiles use it as a LITTLE-ENDIAN read
            # of bytes Bb..Ba (value = Bb + B(b+1)*256 + ... + Ba*256^(a-b))
            terms = ["B%d" % midx(b)]
            for i in range(1, a - b + 1):
                terms.append("B%d*%d" % (midx(b + i), 256 ** i))
            return "(" + "+".join(terms) + ")"
        if m.group(5) is not None:
            return "((B%d>>%d)&1)" % (midx(m.group(5)), int(m.group(6)))
        if m.group(7) is not None:
            b = midx(m.group(7))
            return "((B%d)-((B%d&128)>>7)*256)" % (b, b)
        if m.group(8) is not None:
            return "*%d" % (1 << int(m.group(8)))
        return "B%d" % midx(m.group(9))

    out = TOKEN_RE.sub(repl, expr).replace(" ", "")
    return out, bad[0]


def expr_ok(expr):
    before = len(validator.errors)
    validator.validate_expr(expr, 64, "convert")
    validator.errors[before:] = []
    return len(validator.errors) == before


# ---------------------------------------------------------------------------
# PID conversion
# ---------------------------------------------------------------------------

def max_wican_index(exprs):
    m = 0
    for e in exprs:
        for r in re.finditer(r"[BS](\d+)", e):
            m = max(m, int(r.group(1)))
    return m


def fmt_id(v):
    return "0x%03X" % v if v <= 0x7FF else "0x%08X" % v


def fmt_data(b):
    return " ".join("%02X" % x for x in b)


def convert_pid(pid_entry, ctx, params_db, notes, prof_label, used_names,
                fc_response_id=None):
    pstr = pid_entry.get("pid", "")
    label = "%s pid %s" % (prof_label, pstr)
    if not HEX_RE.match(pstr) or len(pstr) < 2:
        notes.append("SKIP PID %s: pid string not hex" % label)
        return None

    frames = None
    if len(pstr) % 2 == 1:
        d = pstr[-1]
        frames = int(d) if d.isdigit() and 1 <= int(d) <= 9 else None
        req = pstr[:-1]
    else:
        req = pstr
    if len(req) % 2 != 0 or not HEX_RE.match(req):
        notes.append("SKIP PID %s: request hex malformed after stripping "
                     "frame-count digit" % label)
        return None
    rbytes = bytes.fromhex(req)
    if not 1 <= len(rbytes) <= 7:
        notes.append("SKIP PID %s: request is %d bytes (max 7)"
                     % (label, len(rbytes)))
        return None

    params = pid_entry.get("parameters") or {}
    if frames == 1:
        layout = "single"
    elif frames is not None:
        layout = "multi"
    else:
        layout = "multi" if max_wican_index(params.values()) > 7 else "single"

    signals = []
    for name in sorted(params.keys()):
        expr = params[name]
        texpr, bad = transform_expr(expr, layout)
        if bad is not None:
            notes.append("DROP SIGNAL %s.%s: byte B%d (%s layout) not "
                         "addressable in schema" % (label, name, bad, layout))
            continue
        if not expr_ok(texpr):
            notes.append("DROP SIGNAL %s.%s: transformed expr %r rejected by "
                         "schema validator" % (label, name, texpr))
            continue
        # wican short_names are not always strict UPPER_SNAKE (e.g.
        # 'PCM_Oil_Life'); our schema requires it. params.json lookup uses the
        # original name.
        up_name = name.upper()
        sig_name = up_name
        n = 2
        while sig_name in used_names:
            sig_name = "%s_%d" % (up_name, n)
            n += 1
        used_names.add(sig_name)
        info = params_db.get(name, {}).get("settings", {})
        sig = {"name": sig_name, "expr": texpr, "unit": info.get("unit", "")}
        if info.get("class") and info["class"] != "none":
            sig["class"] = info["class"]
        signals.append(sig)

    if not signals:
        notes.append("SKIP PID %s: no convertible signals left" % label)
        return None

    svc_resp = (rbytes[0] + 0x40) & 0xFF
    L = len(rbytes)
    if layout == "single":
        mask = [0x00] + [0xFF] * L + [0x00] * (7 - L)
        match = [0x00, svc_resp] + list(rbytes[1:]) + [0x00] * (7 - L)
    else:
        n = min(L, 6)  # echo spans B3..B(2+L); mask is only 8 bytes wide
        mask = [0xF0, 0x00] + [0xFF] * n + [0x00] * (6 - n)
        match = [0x10, 0x00, svc_resp] + list(rbytes[1:1 + n]) + [0x00] * (6 - n)

    poll_data = bytes([L]) + rbytes
    poll_data = poll_data.ljust(8, b"\x00")[:8]

    if ctx.fcsd:
        fc = bytes.fromhex(ctx.fcsd).ljust(8, b"\x00")[:8]
        fc_str = fmt_data(fc)
    else:
        fc_str = DEFAULT_FC

    poll = {"id": fmt_id(ctx.tx), "data": fmt_data(poll_data),
            "interval_ms": 1000, "flow_control": fc_str,
            "comment": "request %s (WiCAN pid %s, %s, %d response frame%s)"
                       % (req.upper(), pstr, layout,
                          frames if frames is not None else -1,
                          "s" if frames != 1 else "")}
    if fc_response_id is not None:
        # response id differs from request id + 8: the firmware must key
        # flow-control delivery by this id (poll.flow_control_id)
        poll["flow_control_id"] = fmt_id(fc_response_id)
    rx = {"id": fmt_id(ctx.rx), "isotp": True,
          "mask": fmt_data(bytes(mask)), "match": fmt_data(bytes(match)),
          "signals": signals}
    return poll, rx


# ---------------------------------------------------------------------------
# Profile conversion
# ---------------------------------------------------------------------------

def slugify(car_model):
    return SLUG_RE.sub("-", car_model.lower()).strip("-")


def convert_profile(relpath, files, params_db, src_label, notes):
    label = "%s (%s)" % (relpath, src_label)
    prof = resolve_extends(relpath, files)

    base_ctx = Ctx()
    for cmd in [c.strip() for c in (prof.get("init") or "").split(";") if c.strip()]:
        base_ctx.apply(cmd)

    default_header = False
    if base_ctx.tx is None and not any(
            (p.get("pid_init") or "").upper().find("ATSH") >= 0
            for p in prof.get("pids", [])):
        if base_ctx.speed in ("7", "9", "A", "a"):
            raise SkipProfile("no ATSH request header and 29-bit protocol: "
                              "default header not derivable")
        # ELM327 default: functional broadcast header 0x7DF, responses on
        # 0x7E8. Some vehicles answer on 0x7E0 -> may need manual fix.
        base_ctx.tx = 0x7DF
        default_header = True

    # bus speed: ATSP may live in init or only in pid_init; profiles with no
    # ATSP at all rely on ELM327 auto-detection -> default 500000 with a note.
    speed = base_ctx.speed
    if speed is None:
        for p in prof.get("pids", []):
            c = Ctx()
            try:
                for cmd in [x.strip() for x in (p.get("pid_init") or "").split(";")
                            if x.strip()]:
                    c.apply(cmd)
            except SkipProfile:
                continue
            if c.speed:
                speed = c.speed
                break
    if speed is None:
        speed = "6"
        notes.append("NOTE %s: no ATSP protocol command; assuming ATSP6 "
                     "(500000) as ELM327 auto-detect equivalent" % label)
    bus_speed = PROTOCOL_SPEED[speed]
    if default_header:
        notes.append("NOTE %s: no ATSH request header; assumed ELM327 default "
                     "functional header 0x7DF (responses on 0x7E8); if the "
                     "vehicle answers on 0x7E0 change the poll/rx ids"
                     % label)

    polls, rxs = [], []
    used_names = set()
    fc_warned = False
    for pid_entry in prof.get("pids", []):
        ctx = copy.deepcopy(base_ctx)
        for cmd in [c.strip() for c in (pid_entry.get("pid_init") or "").split(";")
                    if c.strip()]:
            ctx.apply(cmd)
        if ctx.tx is None:
            notes.append("SKIP PID %s pid %s: no ATSH request id"
                         % (label, pid_entry.get("pid")))
            continue
        if ctx.rx is None:
            if ctx.tx == 0x7DF:      # functional broadcast -> responses on 0x7E8
                ctx.rx = 0x7E8
            elif ctx.tx <= 0x7FF:
                ctx.rx = ctx.tx + 8
            else:
                notes.append("SKIP PID %s pid %s: 29-bit request id without "
                             "ATCRA (response id not derivable)"
                             % (label, pid_entry.get("pid")))
                continue
        out = convert_pid(pid_entry, ctx, params_db, notes, label, used_names,
                          fc_response_id=(ctx.rx if ctx.rx != ctx.tx + 8
                                          else None))
        if out is None:
            continue
        poll, rx = out
        polls.append(poll)
        rxs.append(rx)
        if ctx.rx != ctx.tx + 8 and not fc_warned:
            notes.append("NOTE %s: responses arrive on %s (request id + 8 "
                         "would be %s); emitted flow_control_id on the poll "
                         "entries - requires firmware support for the key"
                         % (label, fmt_id(ctx.rx), fmt_id(ctx.tx + 8)))
            fc_warned = True
        if ctx.fcsh is not None and ctx.fcsh != ctx.tx:
            notes.append("NOTE %s pid %s: ATFCSH %s != request id %s; the "
                         "schema has no separate flow-control TRANSMIT id "
                         "(flow_control_id carries the response id) - "
                         "flow control would be sent on the wrong id"
                         % (label, pid_entry.get("pid"), fmt_id(ctx.fcsh),
                            fmt_id(ctx.tx)))

    if not polls:
        raise SkipProfile("no convertible pids left")

    ignored = sorted(set(base_ctx.ignored))
    unusual = [c for c in ignored if c.upper() not in BENIGN_CMDS
               and not c.upper().startswith("STCAFCP")]
    orig_note = prof.get("comment") or prof.get("note") or ""
    comment = ("Converted from meatpiHQ/wican-fw vehicle_profiles/%s by "
               "tools/convert_wican.py. WiCAN init: %s. Expression re-basing: "
               "WiCAN exprs are 0-based over ELM327 response data including "
               "PCI bytes; converted to this schema's isotp semantics "
               "(single-frame: raw frame, PCI at B1; multi-frame: assembled "
               "payload = FF bytes 2..7 + CF bytes 1..7). Frame layout per pid "
               "from the WiCAN pid string's trailing frame-count digit, "
               "inferred from byte refs when absent.%s"
               % (relpath, prof.get("init", "").strip(),
                  (" Original note: " + orig_note) if orig_note else ""))
    if unusual:
        comment += " Ignored init commands: " + ", ".join(unusual) + "."

    car = {
        "id": None,  # filled by caller
        "car_model": prof["car_model"],
        "comment": comment,
        "bus_speed": bus_speed,
        "tx": [],
        "poll": polls,
        "rx": rxs,
    }
    return car, unusual


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SRC
    src = os.path.abspath(src)
    params_path = os.path.join(os.path.dirname(src), ".vehicle_profiles",
                               "params.json")
    params_db = load_json(params_path)

    files = {}
    for root, _dirs, names in os.walk(src):
        for n in sorted(names):
            if n.endswith(".json"):
                full = os.path.join(root, n)
                files[os.path.relpath(full, src).replace(os.sep, "/")] = full

    notes = []
    converted = []   # (relpath, car_dict, out_rel)
    skipped = []     # (relpath, reason)

    for relpath in sorted(files):
        if relpath == "generic.json":
            skipped.append((relpath, "generic OBD-II already covered by our "
                                     "generic_obd2.json"))
            continue
        make = relpath.split("/")[0].lower()
        slug = os.path.splitext(os.path.basename(relpath))[0].lower()
        out_rel = "wican/%s/%s.json" % (make, slug)
        try:
            car, _unusual = convert_profile(
                relpath,
                {k: load_json(v) for k, v in files.items()},
                params_db, relpath, notes)
        except SkipProfile as e:
            skipped.append((relpath, str(e)))
            continue
        except Exception as e:  # defensive: one bad file must not kill the run
            skipped.append((relpath, "converter error: %s: %s"
                                    % (type(e).__name__, e)))
            continue
        converted.append((relpath, car, out_rel))

    # unique ids: slug from car_model, de-duplicated across the whole index
    index_path = os.path.join(REPO_ROOT, "vehicle_profiles.json")
    with open(index_path, "r", encoding="utf-8") as f:
        index = json.load(f)
    taken = {v["id"] for v in index["vehicles"]}

    entries = []
    for relpath, car, out_rel in converted:
        slug = slugify(car["car_model"])
        vid = slug
        n = 2
        while vid in taken:
            vid = "%s-%d" % (slug, n)
            n += 1
        taken.add(vid)
        car["id"] = vid
        entries.append({"id": vid, "name": car["car_model"], "profile": out_rel})

        out_path = os.path.join(REPO_ROOT, out_rel)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(car, f, indent=1, ensure_ascii=False)
            f.write("\n")

    # rewrite index: keep non-wican entries as-is, replace wican entries
    kept = [v for v in index["vehicles"]
            if not v["profile"].startswith("wican/")]
    index["vehicles"] = kept + sorted(entries, key=lambda e: e["name"].lower())
    with open(index_path, "w", encoding="utf-8") as f:
        json.dump(index, f, indent=2, ensure_ascii=False)
        f.write("\n")

    write_notes(src, converted, skipped, notes)

    nsig = sum(len(rx["signals"]) for _r, car, _o in converted for rx in car["rx"])
    print("converted: %d profiles (%d signals), skipped: %d, notes: %d"
          % (len(converted), nsig, len(skipped), len(notes)))
    return 0


def write_notes(src, converted, skipped, notes):
    lines = []
    lines.append("# Conversion notes — WiCAN (meatpiHQ/wican-fw) import")
    lines.append("")
    lines.append("Generated by `tools/convert_wican.py` from:")
    lines.append("`%s`" % src)
    lines.append("")
    lines.append("Source commit/paths as found on disk; re-run the converter "
                 "after updating the wican-fw checkout.")
    lines.append("")
    lines.append("## Converted profiles (%d)" % len(converted))
    lines.append("")
    lines.append("| WiCAN source | profile id | signals |")
    lines.append("|---|---|---|")
    for relpath, car, out_rel in sorted(converted, key=lambda c: c[2]):
        nsig = sum(len(rx["signals"]) for rx in car["rx"])
        lines.append("| %s | `%s` | %d |" % (relpath, car["id"], nsig))
    lines.append("")
    lines.append("## Skipped profiles (%d)" % len(skipped))
    lines.append("")
    for relpath, reason in skipped:
        lines.append("- `%s` — %s" % (relpath, reason))
    lines.append("")
    lines.append("## Per-pid / per-signal decisions")
    lines.append("")
    if notes:
        for n in notes:
            lines.append("- " + n)
    else:
        lines.append("(none)")
    lines.append("")
    lines.append("## General caveats")
    lines.append("")
    lines.append("- **ATSP mapping**: ELM327 `ATSP6/7` = ISO-15765-4 CAN 500 kbit"
                 " (11/29-bit), `ATSP8/9/A` = CAN 250 kbit. The schema only "
                 "carries the baud rate (`bus_speed`); 11- vs 29-bit is derived"
                 " by the firmware from the id value (`> 0x7FF` = extended).")
    lines.append("- **PCI length byte**: WiCAN request strings are pure request"
                 " bytes (e.g. `220101`); the ELM layer prepends the ISO-TP "
                 "single-frame PCI length byte. Converted `poll.data` includes"
                 " it (`02 22 01 01 00 00 00 00`), matching the leaf profile.")
    lines.append("- **Frame-count digit**: a WiCAN pid string of odd length "
                 "carries a trailing response-frame-count digit (ELM327 "
                 "optimization, stripped before sending). It decides the byte "
                 "re-basing: `1` = single-frame (+1 shift), `2..9` = multi-frame"
                 " (PCI-stripped assembly). Pid strings without the digit are "
                 "inferred: any byte reference > 7 forces multi-frame, else "
                 "single-frame is assumed — an ambiguous case can produce "
                 "wrong offsets if the ECU answers with the other layout.")
    lines.append("- **Flow control**: `ATFCSD` maps to the poll entry's "
                 "`flow_control` (zero-padded to 8 bytes; default "
                 "`30 01 00 FF FF FF FF FF`), sent on the poll's request id "
                 "(matches `ATFCSH` in every convertible profile; the few "
                 "pids with a genuinely different `ATFCSH` are noted above). "
                 "When the response id (from `ATCRA`) differs from "
                 "`request id + 8`, the poll entry carries an optional "
                 "`flow_control_id` = response id so the firmware can "
                 "deliver flow control to those responses; firmware without "
                 "support for that key falls back to `request id + 8`.")
    lines.append("- **B64 expression limit**: expressions addressing assembled-"
                 "payload bytes beyond B64 (large cell-voltage dumps) are "
                 "dropped per-signal above. Use an `array` signal to reach "
                 "payload bytes up to B256.")
    lines.append("- **float32**: the firmware evaluates expressions in 32-bit "
                 "float; 32-bit integer compositions (e.g. odometer "
                 "`B<<24|...`) lose integer precision above 2^24, same as any "
                 "converted profile relying on them.")
    lines.append("- **29-bit requests without ATCRA**: response id of a 29-bit "
                 "request is not derivable mechanically; such pids are "
                 "skipped.")
    lines.append("- **`class` signal key**: WiCAN Home-Assistant device class,"
                 " kept as an extra signal key (firmware ignores unknown keys).")
    with open(os.path.join(REPO_ROOT, "CONVERSION-NOTES.md"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
