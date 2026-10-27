# Host tools

All tools are Python 3 and talk to the sensors over the RS-485 bus (UART,
19200 8N1) with the binary protocol described in
[doc/binary_protocol.md](../doc/binary_protocol.md). A real port needs
pyserial (`pip install pyserial`).

| Tool | Purpose |
|---|---|
| `mamba_scan.py` | Scan the bus for sensors (addresses 1-30). Read-only. Only parameter: port. |
| `mamba_flash.py` | Erase, read and write the bootloader and application flash (commands `0xA0`-`0xA7`) |
| `mamba_status.py` | Poll the sensors in `SENSORS` and call callbacks on Alarm / Warning changes. Silent unless `VERBOSE`. |

## mamba_scan.py: find the sensors on the bus

```
python3 tools/mamba_scan.py /dev/ttyUSB0      # Linux
python3 tools/mamba_scan.py COM5              # Windows
```

The only parameter is the serial port. Baud rate is fixed at 19200 8N1.

For each address, the scanner sends `FUNC_GET_FWVER` (`0x0E`). A sensor
that answers is then asked for:

* its device ID (`0x08` `FUNC_GET_DEVID`);
* its status word (`0x01` `FUNC_GET_STATUS`, parameter 0 = current status);
* its last error (`0x09` `FUNC_GET_LASTERROR`).

Only these read commands are sent. Nothing is written. Parameter 3 of
`0x01`, which would clear the readout latch, is never used.

Each answer is checked for the `0xAA` start byte, the address with bit 7
set, the checksum and the echoed command. A NAK (`FE`) is retried; bytes
without a valid answer are reported as a problem (for example two sensors
with the same address); a valid frame from another address is a warning.

Settings are constants at the top of the script (edit them there):

| Constant | Default | Meaning |
|---|---|---|
| `FIRST_ADDR`, `LAST_ADDR` | 1, 30 | address range. The firmware accepts 0-30. |
| `BAUD` | 19200 | baud rate (8N1), fixed |
| `TIMEOUT` | 0.1 s | answer timeout per request. A sensor answers about 20 ms after the request. |
| `RETRIES` | 1 | extra attempts per request |
| `GAP` | 0.05 s | minimum gap between requests. The firmware asks for 50 ms. |
| `ECHO` | `"auto"` | RS-485 adapter echo. `"auto"` skips an echo if there is one, `"on"` = always echoes (checked), `"off"` = no echo. |
| `DETAILS` | True | False: only probe with `0x0E` |
| `VERBOSE` | False | hex dump of every frame (on stderr) |

Exit code: 0 if at least one sensor was found, 1 if none, 2 on usage or port error.

Sample output:

```
addr  1  fw 0x23210157  id 1001 (0x000003E9)  status 0x0000 cable=normal  error -5 (ERROR_EEPROM)
addr  4  fw 0x23210157  id 1004 (0x000003EC)  status 0x0101 cable=normal Alarm,TesterEnabled  error 0 (none)
Scanned addresses 1..30: 2 sensor(s) found at 1, 4
```

A full scan of 30 addresses with no sensors takes about 30 × 2 × (0.1 + 0.05) s,
which is about 9 s. Every found sensor adds 3 requests.

## mamba_flash.py

See the docstring (`python3 tools/mamba_flash.py -h`) and
[doc/binary_protocol.md](../doc/binary_protocol.md), section 4.7.

## mamba_status.py: watch Alarm / Warning, call callbacks on change

```
python tools/mamba_status.py COM3    # or /dev/ttyUSB0
```

The only argument is the serial port. All settings are constants at the top
of the script:

| Constant | Default | Meaning |
|---|---|---|
| `SENSORS` | `(1, 2, 3, 4, 5)` | sensor addresses to poll (0-30) |
| `BAUD` | 19200 | baud rate (8N1) |
| `TIMEOUT` | 0.1 s | answer timeout per request |
| `RETRIES` | 2 | extra attempts per request (timeout or NAK) |
| `GAP` | 0.05 s | minimum gap between requests |
| `CLEAR_STICKY` | `False` | read and clear the latched word (writes EEPROM) |
| `VERBOSE` | `False` | print frames, no-answer and edges on stderr |
| `POLL_INTERVAL` | 0.5 s | pause between polling rounds |

The script polls until Ctrl+C, which exits quietly with code 0.

Each round, every sensor's live status word (`0x01` `FUNC_GET_STATUS`,
parameter 0) is read and compared with the previous one. Changes of Alarm
(bit 0) and Warning (bit 10) call the empty functions at the top of the
script, each with the sensor address: `on_alarm_set`, `on_alarm_cleared`,
`on_warning_set`, `on_warning_cleared`. Fill them in. The first successful read
is compared with 0; a bit that is already 1 then counts as a 0 -> 1 edge, so an
alarm active at start-up is not missed.

With `CLEAR_STICKY = True`, the latched word (parameter 1) is read after the
live word, and the latch is cleared (parameter 3, writes EEPROM) only when
something is latched: the latched word differs from the live word, or its
ReadOut bit 15 is set. If the latched word cannot be read, nothing is cleared.
With `False`, the latched word is not read. See
[doc/binary_protocol.md, section 5.3](../doc/binary_protocol.md#53-sticky-latched-status).

With `VERBOSE = False` nothing is printed while running; only fatal start-up
errors (usage, port cannot be opened, pyserial missing) are printed, with exit code 2.
A sensor that does not answer is skipped for that round. With
`True`, frames (hex), sensors that do not answer, and every edge go to stderr.
Answers are checked (start byte, address bit 7, checksum, echoed command), our
own echo is skipped, and a NAK is retried. Needs pyserial.
