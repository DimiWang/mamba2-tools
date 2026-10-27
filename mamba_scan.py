#!/usr/bin/env python3
"""Mamba2 bus scanner: find the sensors on an RS-485 line (binary protocol,
19200 8N1). Read-only: nothing is written to the sensors.

For every address FIRST_ADDR..LAST_ADDR send 0x0E FUNC_GET_FWVER. A sensor
that answers is then asked (if DETAILS) for:
    0x08 FUNC_GET_DEVID, 0x01 FUNC_GET_STATUS param 0, 0x09 FUNC_GET_LASTERROR

Frame (8 bytes): AA | addr (answer: addr|0x80) | sum(bytes 3..7)&0xFF | cmd | UINT32 LE
NAK answer:      AA | addr|0x80 | FE | FE | 00 00 00 00

Usage:  python mamba_scan.py COM5        (or /dev/ttyUSB0)
Exit code 0 if at least one sensor was found, 1 if none, 2 on usage/port error.
Settings: edit the constants below. Needs pyserial (pip install pyserial).
"""
import struct
import sys
import time

# ---- settings: edit these ----
FIRST_ADDR = 1              # first address to scan (firmware accepts 0..30)
LAST_ADDR = 30              # last address to scan
BAUD = 19200                # 8N1
TIMEOUT = 0.1               # answer timeout per request, s (sensor answers ~20 ms)
RETRIES = 1                 # extra attempts per request (timeout / NAK)
GAP = 0.05                  # min gap between requests, s
ECHO = "auto"               # adapter echo: "auto" skip if present, "on" always (checked), "off" none
DETAILS = True              # False: only probe with 0x0E
VERBOSE = False             # True: hex dump of every frame on stderr

PREAMBLE = 0xAA
ANSWER = 0x80
STREAM = 0x40
ADDR_MASK = 0x1F
FUNC_NAK = 0xFE
FRAME_LEN = 8

FUNC_GET_STATUS = 0x01      # param 0 = current status (3 would clear the latch!)
FUNC_GET_DEVID = 0x08
FUNC_GET_LASTERROR = 0x09
FUNC_GET_FWVER = 0x0E

STATUS_BITS = ((0, "Alarm"), (3, "Tamper1"), (4, "Tamper2"), (6, "AccMove"),
               (7, "PowerProblem"), (8, "TesterEnabled"), (9, "DigitalIn"),
               (10, "Warning"), (11, "Warning2"), (12, "AlarmReq"),
               (13, "NoisyLine"), (14, "lidOpened"), (15, "ReadOut"))
CABLE_STATE = {0: "normal", 1: "short", 2: "no end resistor", 3: "?"}
SYSTEM_ERRORS = {0: "none", -5: "ERROR_EEPROM"}


# ---------------------------------------------------------------- framing ----
def checksum(body):
    return sum(body) & 0xFF


def make_frame(addr, cmd, value=0, answer=False):
    body = struct.pack("<BI", cmd & 0xFF, value & 0xFFFFFFFF)
    adr = (addr & ADDR_MASK) | (ANSWER if answer else 0)
    return bytes([PREAMBLE, adr, checksum(body)]) + body


def valid_answers(buf):
    """Yield every checksum-valid answer frame (bit 7 set, not stream) in buf."""
    for i in range(len(buf) - FRAME_LEN + 1):
        f = buf[i:i + FRAME_LEN]
        if f[0] == PREAMBLE and (f[1] & ANSWER) and not (f[1] & STREAM) \
                and checksum(f[3:8]) == f[2]:
            yield f


def find_answer(buf, addr, cmd):
    """("ok", value) / ("nak", None) for the first valid answer from addr, else None.
    Our own echo (bit 7 clear) and noise are skipped."""
    for f in valid_answers(buf):
        if (f[1] & ADDR_MASK) != addr:
            continue
        if f[3] == cmd:
            return "ok", struct.unpack("<I", f[4:8])[0]
        if f[3] == FUNC_NAK:
            return "nak", None
    return None


def hexdump(data):
    return " ".join("%02X" % b for b in data)


def decode_status(v):
    v &= 0xFFFF
    flags = [name for bit, name in STATUS_BITS if v & (1 << bit)]
    return "0x%04X cable=%s%s" % (v, CABLE_STATE[(v >> 1) & 3],
                                  (" " + ",".join(flags)) if flags else "")


def decode_error(v):
    s = v - (1 << 32) if v & 0x80000000 else v
    return "%d%s" % (s, (" (%s)" % SYSTEM_ERRORS[s]) if s in SYSTEM_ERRORS else "")


# ------------------------------------------------------------------- link ----
def log(msg):
    if VERBOSE:
        print(msg, file=sys.stderr)


def open_port(port):
    import serial  # pyserial
    return serial.Serial(port, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.005)


def read_answer(ser, addr, cmd, req, warnings):
    buf = b""
    echo_left = ECHO == "on"
    deadline = time.monotonic() + TIMEOUT
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        if echo_left:
            if len(buf) < FRAME_LEN:
                continue
            if buf[:FRAME_LEN] == req:
                buf = buf[FRAME_LEN:]
            else:
                warnings.append("addr %d: expected echo %s, got %s"
                                % (addr, hexdump(req), hexdump(buf[:FRAME_LEN])))
            echo_left = False
        found = find_answer(buf, addr, cmd)
        if found:
            return found, buf
    return None, buf


_last = [0.0]


def transact(ser, addr, cmd, value, warnings):
    """Returns ("ok", value), ("nak", None), ("garbled", raw) or ("none", None)."""
    req = make_frame(addr, cmd, value)
    result = ("none", None)
    for attempt in range(RETRIES + 1):
        wait = _last[0] + GAP - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        ser.reset_input_buffer()
        log("-> %s   (addr %d cmd 0x%02X try %d)" % (hexdump(req), addr, cmd, attempt + 1))
        ser.write(req)
        ser.flush()                 # half duplex: wait until sent
        found, raw = read_answer(ser, addr, cmd, req, warnings)
        _last[0] = time.monotonic()
        if raw:
            log("<- %s" % hexdump(raw))
        for f in valid_answers(raw):
            if (f[1] & ADDR_MASK) != addr:
                warnings.append("addr %d: answer from address %d on the bus" % (addr, f[1] & ADDR_MASK))
        if found and found[0] == "ok":
            return found
        if found:
            result = found          # NAK: sensor present, frame corrupted; retry
            continue
        body = raw[FRAME_LEN:] if (ECHO != "off" and raw[:FRAME_LEN] == req) else raw
        if body and result[0] != "nak":
            result = ("garbled", raw)
    return result


# ------------------------------------------------------------------- scan ----
def scan(ser, warnings):
    """Returns (found, problems): found = [(addr, fw, devid, status, error)]."""
    found, problems = [], []
    for addr in range(FIRST_ADDR, LAST_ADDR + 1):
        kind, fw = transact(ser, addr, FUNC_GET_FWVER, 0, warnings)
        if kind == "ok":
            info = [None, None, None]
            if DETAILS:
                for i, cmd in enumerate((FUNC_GET_DEVID, FUNC_GET_STATUS, FUNC_GET_LASTERROR)):
                    k, v = transact(ser, addr, cmd, 0, warnings)
                    info[i] = v if k == "ok" else None
            found.append((addr, fw) + tuple(info))
        elif kind == "nak":
            problems.append("Address %d: NAK only (sensor present, frames corrupted)" % addr)
        elif kind == "garbled":
            problems.append("Address %d: no valid answer, received %s (address conflict or noise?)"
                            % (addr, hexdump(fw)))
    return found, problems


def print_results(found, problems, warnings):
    for addr, fw, devid, status, error in found:
        line = "addr %2d  fw 0x%08X" % (addr, fw)
        if DETAILS:
            line += "  id %s  status %s  error %s" % (
                "-" if devid is None else "%d (0x%08X)" % (devid, devid),
                "-" if status is None else decode_status(status),
                "-" if error is None else decode_error(error))
        print(line)
    for p in problems:
        print(p)
    for w in warnings:
        print("Warning: %s" % w)
    print("Scanned addresses %d..%d: %d sensor(s) found%s%s" % (
        FIRST_ADDR, LAST_ADDR, len(found),
        (" at " + ", ".join(str(d[0]) for d in found)) if found else "",
        (", %d address(es) with errors" % len(problems)) if problems else ""))


# ------------------------------------------------------------------- main ----
def main():
    if len(sys.argv) != 2:
        print("usage: python mamba_scan.py PORT   (e.g. COM5 or /dev/ttyUSB0)", file=sys.stderr)
        return 2
    try:
        ser = open_port(sys.argv[1])
    except ImportError:
        print("pyserial is needed: pip install pyserial", file=sys.stderr)
        return 2
    except Exception as e:  # serial.SerialException etc.
        print("cannot open %s: %s" % (sys.argv[1], e), file=sys.stderr)
        return 2
    warnings = []
    try:
        found, problems = scan(ser, warnings)
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130
    finally:
        ser.close()
    print_results(found, problems, warnings)
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
