# ps2

PS/2 host.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. The host receives scan codes from a keyboard or mouse and, with `tx` on, sends it commands. The protocol follows Adam Chapweske's *The PS/2 Mouse/Keyboard Protocol*. A frame is a start bit, eight data bits least significant first, odd parity and a stop bit, and the device drives the clock in both directions. The host reads on the falling edge. To send, it inhibits the clock for at least 100 microseconds, pulls data low as request-to-send, releases the clock, changes data on each falling edge and checks for the device's acknowledge on the eleventh clock. A send ends in `txerr` if the device does not start clocking within 15 ms, does not finish within 2 ms, or does not acknowledge. A received frame that stops halfway is dropped after 2 ms without a clock.

The IP is one rule in Bluespec SystemVerilog: two synchronisers, edge detection, an 11-bit shifter and the send state machine.

The testbench drives a keyboard model that clocks at 12.5 kHz. It checks that a scan code is received and raises the interrupt only with `ien`, that a frame cut off halfway is dropped even though `tick` was lowered while its counter was running, and that a frame with even parity sets `rxerr`. With `tx` on it also checks that a command goes out after at least 100 microseconds of clock inhibit with odd parity and a stop bit, that the device reply is received, that a send without acknowledge sets `txerr`, and that a device that never clocks sets `txerr` and leaves both lines released.

| `tx` | off | on |
| :--: | --: | --: |
| Area, um2 | 1108 | 1765 |

## Registers

| Offset | Register | Fields |
| :--: | :-- | :-- |
| 0x00 | `ctrl` | `en`, `ien` |
| 0x04 | `tick` | clock cycles per microsecond, minus one (99 at reset, for 100 MHz) |
| 0x08 | `rxd` | last byte received |
| 0x0C | `txd` | write a byte to send it (with `tx`) |
| 0x10 | `status` | `rxv`, `rxerr`, `txdone`, `txerr` (write 1 to clear; the last two with `tx`), `busy` |

`clk_pull` and `data_pull` high mean pull the line low, and `clk_i` and `data_i` are the line levels. Both lines are open drain with pull-ups on the board, and PS/2 is a 5 V interface, so the pads need level shifting. The host inhibits the clock only to send; inhibiting at any other time is not implemented.

## License

Mulan PSL v2.
