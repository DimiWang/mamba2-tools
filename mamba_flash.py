#!/usr/bin/env python3
"""Mamba2 flash tool: uses the binary-protocol flash commands 0xA0..0xA7.

Region "boot" = bootloader (boot flash), region "app" = application
(program flash). See doc/binary_protocol.md, section "Flash commands".

Examples:
  mamba_flash.py --port /dev/ttyUSB0 --addr 1 read app 0x9D026000 16
  mamba_flash.py --port /dev/ttyUSB0 erase app 0x9D026000
  mamba_flash.py --port /dev/ttyUSB0 write app image.bin --base 0x9D026000
  mamba_flash.py --port /dev/ttyUSB0 write boot bootloader.hex
  mamba_flash.py --dry-run write app image.bin --base 0x9D026000   (prints frames)
  mamba_flash.py selftest

Needs pyserial (pip install pyserial) except for --dry-run and selftest.
"""
import argparse
import struct
import sys
import time

PREAMBLE = 0xAA
ANSWER = 0x80
FUNC_NAK = 0xFE
PAGE = 0x1000
WORD = 4

# command code = base + operation
REGION_BASE = {"boot": 0xA0, "app": 0xA4}
OP_ERASE, OP_READ, OP_SET, OP_WRITE = 0, 1, 2, 3

# physical limits (same as flash_regions.h); app end assumes a 256 KB part
REGIONS = {
    "boot": {"start": 0x1FC00000, "wr_end": 0x1FC02000, "rd_end": 0x1FC03000},
    "app": {"start": 0x1D006000, "wr_end": 0x1D040000, "rd_end": 0x1D040000},
}

STATUS = {
    0x00: "OK",
    0x01: "RANGE (address outside region / pointer at end)",
    0x02: "ALIGN (not page / word aligned)",
    0x03: "PROTECTED (page holds the running firmware)",
    0x04: "WRITE_PROTECTED (BWP / PWP configuration bits)",
    0x05: "NO_POINTER (write address not set)",
    0x06: "NOT_ERASED (target word not blank)",
    0x07: "NVM_ERROR (WRERR / LVDERR)",
    0x08: "VERIFY (read back differs)",
    0x09: "LOCKED (configuration locked)",
}


class FlashError(Exception):
    pass


def to_phys(addr):
    seg = addr >> 29
    if seg in (4, 5):
        return addr & 0x1FFFFFFF
    if seg == 0:
        return addr
    raise ValueError("unsupported address 0x%08X" % addr)


def frame(dev, cmd, value, answer=False):
    body = struct.pack("<BI", cmd, value & 0xFFFFFFFF)
    adr = (dev & 0x1F) | (ANSWER if answer else 0)
    return bytes([PREAMBLE, adr, sum(body) & 0xFF]) + body


def parse(raw):
    if len(raw) != 8 or raw[0] != PREAMBLE:
        raise FlashError("bad answer %s" % raw.hex(" "))
    if (sum(raw[3:8]) & 0xFF) != raw[2]:
        raise FlashError("answer checksum error %s" % raw.hex(" "))
    return raw[1] & 0x1F, raw[3], struct.unpack("<I", raw[4:8])[0]


def status_pointer(region, value):
    st = value >> 24
    low = value & 0xFFFFFF
    if low == 0xFFFFFF:
        ptr = None
    else:
        ptr = (0x1F000000 if low >= 0xC00000 else 0x1D000000) | low
    return st, ptr


def hexfmt(b):
    return " ".join("%02X" % x for x in b)


class Link:
    def __init__(self, port, baud, dev, timeout, dry_run=False, verbose=False):
        self.dev = dev
        self.dry_run = dry_run
        self.verbose = verbose
        self.ser = None
        self.sim_ptr = {}
        if not dry_run:
            import serial  # pyserial
            self.ser = serial.Serial(port, baud, bytesize=8, parity="N",
                                     stopbits=1, timeout=timeout)

    def transact(self, cmd, value):
        req = frame(self.dev, cmd, value)
        if self.verbose or self.dry_run:
            print("-> " + hexfmt(req))
        if self.dry_run:
            # pretend success: reads return erased flash, the pointer is simulated
            op = cmd & 3
            if op == OP_READ:
                return cmd, 0xFFFFFFFF
            ptr = self.sim_ptr.get(cmd & ~3, 0xFFFFFF)
            if op in (OP_ERASE, OP_SET):
                ptr = to_phys(value) & 0xFFFFFF
            else:
                ptr = (ptr + WORD) & 0xFFFFFF
            self.sim_ptr[cmd & ~3] = ptr
            return cmd, ptr
        self.ser.reset_input_buffer()
        self.ser.write(req)
        deadline = time.time() + self.ser.timeout + 0.5
        buf = b""
        while time.time() < deadline:
            buf += self.ser.read(8 - len(buf) if len(buf) < 8 else 1)
            # skip local echo / noise until an answer frame from our device
            while len(buf) >= 8:
                if buf[0] == PREAMBLE and (buf[1] & ANSWER) and (buf[1] & 0x1F) == self.dev \
                        and (sum(buf[3:8]) & 0xFF) == buf[2]:
                    if self.verbose:
                        print("<- " + hexfmt(buf[:8]))
                    _, acmd, val = parse(buf[:8])
                    time.sleep(0.02)
                    return acmd, val
                buf = buf[1:]
        raise FlashError("no answer to %s" % hexfmt(req))

    def status_cmd(self, region, op, value):
        cmd = REGION_BASE[region] + op
        acmd, val = self.transact(cmd, value)
        if acmd == FUNC_NAK:
            raise FlashError("NAK (checksum error) for cmd 0x%02X" % cmd)
        if acmd != cmd:
            raise FlashError("unexpected answer cmd 0x%02X" % acmd)
        st, ptr = status_pointer(region, val)
        if st != 0:
            raise FlashError("cmd 0x%02X param 0x%08X: status 0x%02X %s" %
                             (cmd, value, st, STATUS.get(st, "?")))
        return ptr

    def erase(self, region, addr):
        return self.status_cmd(region, OP_ERASE, addr)

    def set_ptr(self, region, addr):
        return self.status_cmd(region, OP_SET, addr)

    def write(self, region, word):
        return self.status_cmd(region, OP_WRITE, word)

    def read(self, region, addr):
        cmd = REGION_BASE[region] + OP_READ
        acmd, val = self.transact(cmd, addr)
        if acmd == FUNC_NAK:
            st = val >> 24
            raise FlashError("read 0x%08X: %s" % (addr, STATUS.get(st, "NAK / checksum error")
                                                    if st else "NAK (checksum error)"))
        return val


def load_image(path, base):
    """Return {phys_addr: byte}. Intel HEX (.hex) or raw binary at base."""
    data = {}
    if path.lower().endswith(".hex"):
        upper = 0
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line.startswith(":"):
                    continue
                rec = bytes.fromhex(line[1:])
                if sum(rec) & 0xFF:
                    raise FlashError("HEX checksum error: " + line)
                n, off, typ = rec[0], (rec[1] << 8) | rec[2], rec[3]
                payload = rec[4:4 + n]
                if typ == 0:
                    for i, b in enumerate(payload):
                        data[to_phys(upper + off + i)] = b
                elif typ == 1:
                    break
                elif typ == 4:
                    upper = ((payload[0] << 8) | payload[1]) << 16
                elif typ == 2:
                    upper = ((payload[0] << 8) | payload[1]) << 4
    else:
        if base is None:
            raise FlashError("--base is needed for a binary image")
        start = to_phys(base)
        with open(path, "rb") as f:
            for i, b in enumerate(f.read()):
                data[start + i] = b
    return data


def plan_words(region, data):
    r = REGIONS[region]
    inside = {a: b for a, b in data.items() if r["start"] <= a < r["wr_end"]}
    skipped = len(data) - len(inside)
    words = {}
    for a, b in inside.items():
        w = a & ~3
        words.setdefault(w, bytearray(b"\xFF" * 4))[a - w] = b
    return {a: struct.unpack("<I", bytes(v))[0] for a, v in sorted(words.items())}, skipped


def do_write(link, region, path, base, verify):
    words, skipped = plan_words(region, load_image(path, base))
    if skipped:
        print("note: %d bytes outside the %s write range are skipped" % (skipped, region))
    if not words:
        raise FlashError("nothing to write in region %s" % region)
    pages = sorted({a & ~(PAGE - 1) for a in words})
    for p in pages:
        print("erase 0x%08X" % (p | 0xA0000000))
        link.erase(region, p | 0xA0000000)
    expected = None
    n = 0
    for a, w in words.items():
        if w == 0xFFFFFFFF:
            continue  # erased already
        if expected != a:
            link.set_ptr(region, a | 0xA0000000)
        expected = link.write(region, w)
        n += 1
        if n % 256 == 0:
            print("  %d words" % n)
    print("wrote %d words in %d pages" % (n, len(pages)))
    if verify and not link.dry_run:
        for a, w in words.items():
            got = link.read(region, a | 0xA0000000)
            if got != w:
                raise FlashError("verify 0x%08X: 0x%08X != 0x%08X" % (a, got, w))
        print("verify OK")


def selftest():
    # examples used in doc/binary_protocol.md
    assert hexfmt(frame(1, 0xA4, 0x9D026000)) == "AA 01 A3 A4 00 60 02 9D"
    assert hexfmt(frame(1, 0xA4, 0x00026000, True)) == "AA 81 06 A4 00 60 02 00"
    assert hexfmt(frame(1, 0xA7, 0x12345678)) == "AA 01 BB A7 78 56 34 12"
    assert hexfmt(frame(1, 0xA7, 0x00026004, True)) == "AA 81 0D A7 04 60 02 00"
    assert status_pointer("app", 0x00026004) == (0, 0x1D026004)
    assert status_pointer("boot", 0x00C02000) == (0, 0x1FC02000)
    assert status_pointer("app", 0x05FFFFFF) == (5, None)
    assert to_phys(0x9D026000) == to_phys(0xBD026000) == to_phys(0x1D026000) == 0x1D026000
    w, s = plan_words("app", {0x1D026000: 0x78, 0x1D026001: 0x56, 0x1D026006: 0x11, 0x1D000000: 1})
    assert w == {0x1D026000: 0xFFFF5678, 0x1D026004: 0xFF11FFFF} and s == 1
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--addr", type=int, default=1, help="device address 0..30")
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--dry-run", action="store_true", help="print request frames, no serial port")
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("read"); p.add_argument("region", choices=REGION_BASE)
    p.add_argument("address", type=lambda s: int(s, 0)); p.add_argument("count", type=int, nargs="?", default=1)
    p = sub.add_parser("erase"); p.add_argument("region", choices=REGION_BASE)
    p.add_argument("address", type=lambda s: int(s, 0))
    p = sub.add_parser("write"); p.add_argument("region", choices=REGION_BASE); p.add_argument("image")
    p.add_argument("--base", type=lambda s: int(s, 0), help="load address of a .bin image")
    p.add_argument("--no-verify", action="store_true")
    sub.add_parser("selftest")
    a = ap.parse_args()

    if a.cmd == "selftest":
        selftest()
        return 0
    link = Link(a.port, a.baud, a.addr, a.timeout, a.dry_run, a.verbose)
    try:
        if a.cmd == "read":
            for i in range(a.count):
                addr = a.address + 4 * i
                print("0x%08X: 0x%08X" % (addr, link.read(a.region, addr)))
        elif a.cmd == "erase":
            link.erase(a.region, a.address)
            print("erased")
        elif a.cmd == "write":
            do_write(link, a.region, a.image, a.base, not a.no_verify)
    except FlashError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
