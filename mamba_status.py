#!/usr/bin/env python3
"""Mamba2 status monitor: poll the sensors listed in SENSORS over RS-485
(binary protocol, 19200 8N1) and call a callback on every change of the
Alarm (bit 0) and Warning (bit 10) bits of the live status word.

Per sensor, with command 0x01 FUNC_GET_STATUS:
    param 0 -> live status word (polled every POLL_INTERVAL)
    param 1 -> latched word (only read if CLEAR_STICKY)
    param 3 -> clear the latch (only if CLEAR_STICKY and something latched)

Frame (8 bytes): AA | addr (answer: addr|0x80) | sum(bytes 3..7)&0xFF | cmd | UINT32 LE
NAK answer:      AA | addr|0x80 | FE | FE | 00 00 00 00

Usage:  python mamba_status.py COM3        (or /dev/ttyUSB0)
Ctrl+C stops. With VERBOSE = False nothing is printed while running.
Settings: edit the constants below. Needs pyserial (pip install pyserial).
"""
import struct
import sys
import time

# ---- settings: edit these ----
SENSORS = (1, 2, 3, 4, 5)   # sensor addresses to poll (BCD switch, 0..30)
BAUD = 19200                # 8N1
TIMEOUT = 0.1               # answer timeout per request, s
RETRIES = 2                 # extra attempts per request (timeout / NAK)
GAP = 0.05                  # min gap between requests, s
CLEAR_STICKY = False        # True: clear the latch after reading it (writes EEPROM)
VERBOSE = False             # True: print frames (hex), no-answer and edges on stderr
POLL_INTERVAL = 0.5         # pause between polling rounds, s

PREAMBLE = 0xAA
ANSWER = 0x80
STREAM = 0x40
ADDR_MASK = 0x1F
FUNC_NAK = 0xFE
FRAME_LEN = 8

FUNC_GET_STATUS = 0x01
STATUS_LIVE = 0
STATUS_LATCHED = 1
STATUS_CLEAR = 3

ALARM_BIT = 0
WARNING_BIT = 10


# -------------------------------------------------------------- callbacks ----
def on_alarm_set(addr):
    """Alarm bit of sensor addr went 0 -> 1. Put your action here."""
    pass


def on_alarm_cleared(addr):
    """Alarm bit of sensor addr went 1 -> 0."""
    pass


def on_warning_set(addr):
    """Warning bit of sensor addr went 0 -> 1."""
    pass


def on_warning_cleared(addr):
    """Warning bit of sensor addr went 1 -> 0."""
    pass


# (bit, name, set callback, cleared callback); looked up at call time
EDGES = (
    (ALARM_BIT, "alarm", "on_alarm_set", "on_alarm_cleared"),
    (WARNING_BIT, "warning", "on_warning_set", "on_warning_cleared"),
)


# ---------------------------------------------------------------- framing ----
def checksum(body):
    return sum(body) & 0xFF


def make_frame(addr, cmd, value=0, answer=False):
    body = struct.pack("<BI", cmd & 0xFF, value & 0xFFFFFFFF)
    adr = (addr & ADDR_MASK) | (ANSWER if answer else 0)
    return bytes([PREAMBLE, adr, checksum(body)]) + body


def find_answer(buf, addr, cmd):
    """Return ("ok", value) / ("nak", None) for the first valid answer from
    addr in buf, else None. Our own echo (bit 7 clear) and noise are skipped."""
    for i in range(len(buf) - FRAME_LEN + 1):
        f = buf[i:i + FRAME_LEN]
        if f[0] != PREAMBLE or not (f[1] & ANSWER) or (f[1] & STREAM):
            continue
        if (f[1] & ADDR_MASK) != addr or checksum(f[3:8]) != f[2]:
            continue
        if f[3] == cmd:
            return "ok", struct.unpack("<I", f[4:8])[0]
        if f[3] == FUNC_NAK:
            return "nak", None
    return None


def hexdump(data):
    return " ".join("%02X" % b for b in data)


# ------------------------------------------------------------------- link ----
# A link is a dict: {"write": f(bytes), "read": f() -> bytes, "flush_in": f(),
#                    "timeout": s, "retries": n, "gap": s, "last": t, "verbose": bool}
def transact(link, addr, cmd, value=0):
    """One request, one answer. Returns ("ok", value), ("nak", None) or ("none", None)."""
    req = make_frame(addr, cmd, value)
    result = ("none", None)
    for attempt in range(link["retries"] + 1):
        wait = link["last"] + link["gap"] - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        link["flush_in"]()
        if link["verbose"]:
            print("-> %s (addr %d try %d)" % (hexdump(req), addr, attempt + 1), file=sys.stderr)
        link["write"](req)
        buf = b""
        found = None
        deadline = time.monotonic() + link["timeout"]
        while found is None and time.monotonic() < deadline:
            chunk = link["read"]()
            if chunk:
                buf += chunk
                found = find_answer(buf, addr, cmd)
        link["last"] = time.monotonic()
        if link["verbose"] and buf:
            print("<- %s" % hexdump(buf), file=sys.stderr)
        if found and found[0] == "ok":
            return found
        if found:
            result = found          # NAK: sensor is there, frame got corrupted; retry
    return result


def serial_link(port, baud, timeout, retries, gap, verbose):
    import serial  # pyserial
    ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0.005)

    def write(data):
        ser.write(data)
        ser.flush()                 # half duplex: wait until sent

    return {"write": write, "read": lambda: ser.read(ser.in_waiting or 1),
            "flush_in": ser.reset_input_buffer, "close": ser.close,
            "timeout": timeout, "retries": retries, "gap": gap, "last": 0.0,
            "verbose": verbose}


# ------------------------------------------------------------------- poll ----
def log(link, msg):
    if link["verbose"]:
        print(msg, file=sys.stderr)


def read_live(link, addr, clear=False):
    """Return the live status word of addr, or None if it did not answer.
    If clear: read the latched word and clear the latch only when something
    is latched (latched != live, or ReadOut bit 15 set)."""
    kind, live = transact(link, addr, FUNC_GET_STATUS, STATUS_LIVE)
    if kind != "ok":
        log(link, "addr %d: %s" % (addr, "no answer" if kind == "none" else "NAK"))
        return None
    live &= 0xFFFF
    if clear:
        kind, latched = transact(link, addr, FUNC_GET_STATUS, STATUS_LATCHED)
        if kind == "ok":        # never clear what we could not read
            latched &= 0xFFFF
            if latched != live or latched & 0x8000:
                transact(link, addr, FUNC_GET_STATUS, STATUS_CLEAR)
    return live


def check_edges(link, addr, live, state):
    """Compare live with the previous word of addr in state; call callbacks.

    First successful read: the previous word is taken as 0, so a bit that is
    already 1 counts as a 0 -> 1 edge and its 'set' callback is called (an
    alarm active at start-up is not missed). A bit that is 0 gives no call."""
    prev = state.get(addr, 0)
    state[addr] = live
    for bit, name, set_cb, clr_cb in EDGES:
        old, new = (prev >> bit) & 1, (live >> bit) & 1
        if old == new:
            continue
        log(link, "addr %d: %s %s" % (addr, name, "set" if new else "cleared"))
        globals()[set_cb if new else clr_cb](addr)


def poll_once(link, sensors, state, clear=False):
    for addr in sensors:
        live = read_live(link, addr, clear)
        if live is not None:
            check_edges(link, addr, live, state)


def poll_forever(link, sensors, clear=False, interval=POLL_INTERVAL):
    state = {}                  # addr -> last live status word
    while True:
        poll_once(link, sensors, state, clear)
        time.sleep(interval)


# ------------------------------------------------------------------- main ----
def main():
    if len(sys.argv) != 2:
        print("usage: python mamba_status.py PORT   (e.g. COM3 or /dev/ttyUSB0)", file=sys.stderr)
        return 2
    try:
        link = serial_link(sys.argv[1], BAUD, TIMEOUT, RETRIES, GAP, VERBOSE)
    except ImportError:
        print("pyserial is needed: pip install pyserial", file=sys.stderr)
        return 2
    except Exception as e:  # serial.SerialException etc.
        print("cannot open %s: %s" % (sys.argv[1], e), file=sys.stderr)
        return 2
    try:
        poll_forever(link, SENSORS, CLEAR_STICKY)
    except KeyboardInterrupt:
        pass                    # Ctrl+C: quiet exit
    finally:
        link["close"]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
