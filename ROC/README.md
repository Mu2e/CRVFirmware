# CRV ROC firmware

Firmware of the Mu2e CRV readout controller (ROC). Written from the sources on
branch `sc/ewt-match`, commit `d7399cd` (FPGA1 version `a9`).

## 1. What this is

| Part | Directory | Device | What it does |
|---|---|---|---|
| FPGA1 | `FPGA1/` | Spartan-6 `xc6slx25t-fgg484-3` | Fibre link to the DTC, event building, clock and marker fan-out. One per ROC. |
| FPGA2 ("data FPGA") | `FPGA2/` | Spartan-6 `xc6slx25-fgg484-3` | Eight FEB ports each, port buffers, DDR event store, link to FPGA1. Three per ROC, same bitfile. |
| uC | `uC/` | TI RM48 (IAR project) | Slow control, forwards data requests from FPGA1 to the FEBs. |

Both FPGA designs are ISE 14.7 projects (`Controller_FPGA1.xise`,
`Controller_FPGA2.xise`).

The uC reaches all four FPGAs on one 16-bit bus. Address bits 11..10 select the
FPGA (0 = FPGA1, 1..3 = data FPGAs, i.e. offsets `0x000`, `0x400`, `0x800`,
`0xC00`), bits 9..0 the register. Addresses `0x300`..`0x302` are broadcasts to
the three data FPGAs.

**FPGA2:** this document describes the FPGA2 source on this branch (version
register `0x0013`; `0x0010` before the port match). Build products are not tracked
in git.

## 2. Data path

1. The DTC sends a data request for one event window tag (EWT, also "uB").
2. FPGA1 stores the request for the uC (`DReqBuff`) and its EWT (`TStmpBuff`,
   3 words per request).
3. The uC reads the request and broadcasts a uB request to the FEBs through the
   transmit buffers of the three data FPGAs.
4. Each FEB answers into its port buffer in a data FPGA (`PhyRxBuff`, one per
   port, 8 ports per data FPGA).
5. When every active port holds a complete answer, the data FPGA copies the
   event part to its DDR memory. With the port match on (section 5) the part
   is built per request instead: ports that did not answer this request are
   left out and flagged.
6. From the DDR the part goes over a two-lane serial link into one of the three
   FPGA1 link FIFOs (`LinkBuff`, 8192 x 16).
7. `EventBuilder` waits until every link with active ports holds a complete
   part, then builds the event into `EventBuff` and writes the event's tag to
   `EvUbBuff`.
8. `DRHandler` compares that tag with the oldest request in `TStmpBuff` and
   tells `PacketFormer` to send or to discard the event.
9. `PacketFormer` sends the data header and the data packets to the DTC.

**Permission line.** FPGA1 drives one line to all three data FPGAs (`TrigFM` in
FPGA1, `DReqFM` in FPGA2). With CSR bit 6 = 0 it is high when all three link
FIFOs are empty and no link FIFO reset is in progress. With CSR bit 6 = 1 the
line carries FM-encoded requests instead. The data FPGAs follow the same bit:
they take it from any write to bus address `0x000`.

**Credit counter.** A data FPGA starts reading an event from the DDR only while
`TxBlkCount` is not 0. The counter is loaded with 4 when it is 0 and the
permission line is high, and decrements when a DDR read starts. The reload is
level based, so it does not limit a data FPGA to one event per permission.

## 3. Clocks

Board clock `ClkB` is 160 MHz (as configured in the clock cores and in the
FPGA2 constraints; not checked against the schematic).

### FPGA1

| Clock | Frequency | Source | Logic |
|---|---|---|---|
| `UsrClk2(0)` | 156.25 MHz | GTP reference clock | fibre receive and transmit, `EventBuilder`, `DRHandler`, `PacketFormer`, `LinkFlushCtrl`, read side of the link FIFOs |
| `UsrClk2(1)` | 156.25 MHz | GTP | second fibre channel |
| `RxOutClk(0..2)` | 100 MHz | recovered from each data FPGA link | link deserialisers, write side of the link FIFOs |
| `SysClk` | 100 MHz | `ClkB` x 5/8 | uC bus and registers, DCS buffers, request FM transmitter, time bases, PLL and LED serial ports |
| `EthClk` | 160 MHz | `ClkB`, 90 degrees | marker input sampling, Orange Tree port |
| `Clk80MHz` | 80 MHz | `EthClk` divided by two in `Clk80MHzGen`, phase aligned to the marker | heartbeat and trigger FM transmitters, heartbeat generator |

### FPGA2

| Clock | Frequency | Source | Logic |
|---|---|---|---|
| `SysClk` | 100 MHz | `ClkB` PLL | uC bus and registers, DDR write and read sequencers, read side of the port buffers |
| `RxFMClk` | 200 MHz | `ClkB` PLL | FEB receive (PHY data, CRC, FM receivers), write side of the port buffers. The PHY data itself comes with the 25 MHz receive clock of each PHY (4 bits per cycle); `RxFMClk` samples that clock and takes one nibble per edge |
| `BitClk` | 500 MHz | `ClkB` PLL | serialiser of the link to FPGA1 |
| `i50MHz` | 50 MHz | `ClkB` PLL | PHY management (MDIO) and SPI |
| LPDDR controller clocks | from `VXO` | memory controller core | DDR memory interface |

## 4. Register map FPGA1

Bus offset `0x000`. 16-bit registers. Addresses that are defined in
`Project_Defs.vhd` but neither decoded nor in the read mux are not listed.
"Pops" means a read removes one word from that buffer.

### Control and status

| Addr | R/W | Meaning |
|---|---|---|
| 0x00 | R/W | CSR, see below |
| 0x01 | R/W | GTP: W [0] disable transmitter A, [8] disable transmitter B. R channel 0: [0] transmitter disabled, [1] reframe, [3:2] comma, [5:4] invalid character, [7:6] control character; channel 1 the same in [15:8] |
| 0x02 | R/W | W [0] = 1: `GTPRxRst`, resets the receive FIFOs, the data path and the counters. R [5:4] GTP receive buffer full, [3:2] empty |
| 0x08 | R/W | active ports, bits 23..16. Written by the uC only with CSR bit 6 = 1; otherwise copied from the data FPGAs |
| 0x09 | R/W | active ports, bits 15..0 |
| 0x0A | R/W | [3:0] ROC ID, sent in the packets |
| 0x19 | R/W | [0] PLL chip power down; R also [4] PLL status |
| 0x27 | R/W | link FIFOs: W [8] reset deserialisers, [9] reset link FIFOs. R [6:4] full, [2:0] empty |
| 0x6C / 0x6D | R | uptime in seconds, high / low |
| 0x99 | R | firmware version (`0x00A9`) |
| 0x96 / 0x97 | R | git hash, bits 31..16 / 15..0, see section 7 |

CSR 0x00:

| Bit | R/W | Meaning |
|---|---|---|
| 0 | R | `IntTmgEn`, never set in this version |
| 1 | R | `TmgCntEn`, never set in this version |
| 2 | R/W | `FormHold`: the event builder does not start |
| 3 | W | 1: reset of the GTP transceiver |
| 4 | R/W | `ExtTmg` |
| 5 | R/W | `MarkerSyncEn` |
| 6 | R/W | `TrigTx_Sel`: 0 = `TrigFM` is the permission line, 1 = FM-encoded requests |
| 7 | W | 1: `FormRst`, resets event builder and packet former state |
| 8 | R/W | `TstTrigEn`, test trigger enable |
| 9 | R/W | `TstTrigCE`, finite burst of test triggers (count in 0x0F) |
| 10 | R/W | `HrtBtTxInh`, inhibit heartbeat transmission |
| 11 | R/W | `DRHold`: the data request handler does not start |

### Data requests and event building

| Addr | R/W | Meaning |
|---|---|---|
| 0x0D | R | request buffer for the uC (`DReqBuff`), pops |
| 0x0E | R | words in `DReqBuff` |
| 0x3C | R | [15] `DReqBuff` empty, [9:0] words |
| 0x86 | R | trace of the request words, pops |
| 0x04..0x06 | R | words in link FIFO 0..2 |
| 0x24..0x26 | R | first word of link FIFO 0..2, pops (debugging on a stopped ROC only) |
| 0x07 | R | [1] `EventBuff` full, [0] empty |
| 0x5C | R | copy of what was written to `EventBuff`, pops |
| 0x2C | R | event word count buffer, pops |
| 0x2D | R | [15] word count buffer full, [14] empty, low bits words |
| 0x5D | R/W | ROC timeout for a request without event, units of 6.4 ns. `0xFFFF` = off. Reset value `0xFFFE` (419 us) |
| 0x94 | R | timeout counter of the request being handled |
| 0x58 | R/W | [0] `sendGR`: build fake events on markers instead of FEB data, [15:8] W extra length; R [15:8] fake events built |
| 0x60 / 0x61 | R/W | internal data request generator: EWT low / mid word. A write to 0x60 generates one request |
| 0x79 | R/W | [0] `uBinHeader`, [1] `uBwrt`, [2] `uBdebug`, [3] `uBdebug2`. Purpose unclear; bits 0 and 1 reach the event builder and packet former but are not used there |
| 0x88..0x8E | R | uB numbers of the three links at the last event: link 2, link 1, link 0 (two words each), 0x8E [0] mismatch flag |

### EWT match (`DRHandler`)

Counters stop at `0xFFFF` and are cleared by `GTPRxRst`.

| Addr | R/W | Meaning |
|---|---|---|
| 0x82 / 0x83 | R | data headers sent to the DTC, low / high |
| 0x9C | R | data requests received from the DTC |
| 0x9D | R | events built |
| 0x8F | R | requests dropped (no event, or superseded by a newer request) |
| 0x90 | R | events dropped (late, old, or bad tag) |
| 0x91 | R | [15:8] built events waiting for a decision, [7:0] words in `TStmpBuff`; both stop at 255 |
| 0x92 | R | state machines, see below |
| 0x93 | R/W | control: [0] drop late, [1] bypass matching, [2] link FIFO flush, [3] bad-tag guard. Reads `0x000D` after reset |
| 0x9A | R | event tag, low 16 bits, at the last drop that involved an event |
| 0x9B | R | request tag, low 16 bits, at the last drop |

0x92:

| Bits | Meaning |
|---|---|
| 13 | `EventBuff` empty |
| 12 | `EvUbBuff` empty |
| 11:8 | event builder: 0 idle, 1 waiting for complete parts, 2..D building, E fake event, F resetting link FIFOs |
| 7 | packet former idle |
| 6:4 | packet former: 0 idle, 1 packet count, 2 data header, 3 controller header, 4 data, 5 discarding, 7 DCS |
| 3:0 | handler: 0 idle, 1 start, 2 waiting, 3 compare, 4 send, 5 timeout, 6 drop request, 7 drop event, 8 settle, 9 drop superseded request |

### Link FIFO flush (`LinkFlushCtrl`)

| Addr | R/W | Meaning |
|---|---|---|
| 0x9E | R | resets of the link FIFOs, any cause |
| 0x9F | R | bad tags: event newer than the request with no newer request queued |
| 0xA0 | R | stale data: requests that found data in the link FIFOs with the builder idle or waiting |
| 0xA1 | R | truncated events |
| 0xA2 | R | at the last bad tag or stale trigger: [0] bad tag, [1] stale, [2] flush was enabled, [7:4] builder state, [10:8] link FIFO empty flags, [15:12] words in `TStmpBuff` (stops at 15) |
| 0xA3..0xA5 | R | first word of link FIFO 0..2 at that moment |
| 0xA6..0xA8 | R | words in link FIFO 0..2 at that moment |

### Heartbeat, marker and timing

| Addr | R/W | Meaning |
|---|---|---|
| 0x62 | R/W | [0] heartbeat generator enable |
| 0x63 / 0x64 | R/W | generator period in 80 MHz clocks, low / high. One heartbeat every period + 1 clocks; write the period before enabling |
| 0x3D | R | [15] heartbeat buffer empty, low bits words |
| 0x3E | R | heartbeat buffer, pops |
| 0x81 | R/W | offset added to the microbunch number (`ExtuBunchOffset`) |
| 0x70 | R | low 16 bits of the microbunch number last sent |
| 0x41 | R | delayed markers counted |
| 0x42 | R | `HeartBeatCnt` (source comment: markers sent out) |
| 0x4F | R | `HeartBtCnt` (source comment: EWT from the fibre) |
| 0x43 | R | `LastWindow`: timer value at the last heartbeat |
| 0x44 / 0x54 | R | `InjectionTs` / first `InjectionTs` |
| 0x59 | R | [7:0] injection counter |
| 0x5A | R | `InjectionWindow` |
| 0x5B | R | [15:12] last four samples of the NIM trigger input, [9] GPI input, [8] NIM trigger input, [7:0] injection duty (never assigned, reads 0). Purpose unclear |
| 0x78 | R/W | [7:0] marker delay |
| 0x45 | R/W | [0] align the 80 MHz clock to the marker; R [15:8] alignment shift counter |
| 0x47 | R/W | [2:0] loopback mode |
| 0x4A | R | [7:0] loopback marker counter |
| 0x80 | R | [15:8] CRC error counter (constant 0 in this version), [3:0] loss-of-signal counter |
| 0x98 | R/W | automatic GTP reset on loss of signal: W [0] enable (reset value 1). R [15] reset active, [13] enable, [12] armed, [6:0] counter |
| 0x0F | W | number of test triggers in a burst |
| 0x39 | W | [0] `TstPlsEn`, [1] `TrgSrc`. Purpose unclear |
| 0x3A / 0x3B | W | `FreqReg` high / low (reset value `0x00000237`). Purpose unclear |
| 0x40 | R/W | [8:0] data request prescale |
| 0x34 / 0x35 | R/W | test counter high / low |

### DCS and traces

| Addr | R/W | Meaning |
|---|---|---|
| 0x50 | R | DCS request buffer from the DTC, pops |
| 0x51 | R | words in the DCS request buffer |
| 0x53 | R/W | W: one word into the DCS reply buffer. R: last word written |
| 0x52 | R | DCS reply buffer: [15] write strobe, [14] full, [13] empty, low bits words |
| 0x20 | R | trace of received fibre words, channel 0, pops |
| 0x0C | R | status and word counts of the two fibre receive buffers |
| 0x85 | R | trace of transmitted fibre words, pops |
| 0x87 | R | reads 0 in this version (trace of link FIFO input is commented out) |
| 0x4C | W | [8] `DDRSel`, [12] debug buffer reset. Purpose unclear |

### Serial ports

| Addr | R/W | Meaning |
|---|---|---|
| 0x10..0x15 | W | front panel LED data |
| 0x16 | W | [0] LED reset |
| 0x17 / 0x18 | W | PLL chip data, upper 8 bits / lower 16 bits |

## 5. Register map FPGA2

Bus offsets `0x400`, `0x800`, `0xC00`. Source on this branch, see section 1.

### Control and status

| Addr | R/W | Meaning |
|---|---|---|
| 0x00 | R/W | CSR, see below. Also written by broadcast `0x300` |
| 0x17 | R/W | [7:0] input mask: ports allowed to become active |
| 0x2F | R | [7:0] active ports: masked ports on which link activity is seen |
| 0x0E | R/W | [7:0] transmit enable mask per port |
| 0x15 | R | [7:0] PHY carrier sense per port |
| 0x14 | R | [7:0] PHY receive error per port |
| 0x16 | R | [7:0] port buffer not empty |
| 0x60 | R/W | [7:0] CRC error per port; write 1 to clear |
| 0x70..0x77 | R | PHY activity counter, port 0..7 |
| 0x61 | R | [15:12] request word counter, [11] request source (CSR bit 6 of FPGA1), [9:0] debug pins |
| 0x6C / 0x6D | R | uptime in seconds, high / low |
| 0x43 / 0x44 | R/W | test counter high / low |
| 0x99 | R | firmware version (`0x0013`; `0x0012` first version with the port match, `0x0010` before it) |

CSR 0x00:

| Bit | R/W | Meaning |
|---|---|---|
| 0 | R/W | 1: reset of the eight port buffers, and a reset pulse to the PHY chips |
| 1 | W | reset of the link to FPGA1 |
| 2 | R/W | PHY chips powered (1 = on) |
| 3 | R/W | FM receivers enabled |
| 4 | W | reset of the DDR controller |
| 5 | R/W | copy events from the port buffers to the DDR |
| 6 | R/W | `PhyDatSel` |
| 7 | R/W | read events from the DDR and send them to FPGA1 |
| 8 | W | `InitReq` |
| 9 | W | reset of the request word counter |
| 10 | R | request buffer empty |

### FEB ports

| Addr | R/W | Meaning |
|---|---|---|
| 0x11 | W | one word into the transmit buffer to the FEBs. Also written by broadcast `0x301` |
| 0x12 | R/W | W [0] = 1: transmit the buffer. Also by broadcast `0x302`. R [1] transmit buffer empty, [0] transmission in progress |
| 0x13 | R | words in the transmit buffer |
| 0x18..0x1F | R | words in port buffer 0..7 |
| 0x20..0x27 | R | first word of port buffer 0..7, pops |
| 0x50..0x5F | R | CRC of the last frame, port 0..7, high / low word |
| 0x30..0x37 | R | FM receive buffer 0..7, pops |
| 0x38..0x3F | R | words in FM receive buffer 0..7 |
| 0x40 | R | FM receive buffers: [15:8] full, [7:0] empty |
| 0x41 | R/W | FM parity errors: R [9:8] request and heartbeat receiver, [7:0] per port. W [7:0] clear per port, [8] clear all and reset the FM buffers, [9], [10] clear request / heartbeat receiver |

### DDR memory and link to FPGA1

| Addr | R/W | Meaning |
|---|---|---|
| 0x02 / 0x03 | R/W | DDR write pointer, high / low |
| 0x04 / 0x05 | R/W | DDR read pointer, high / low. A write to 0x05 starts a read |
| 0x07 | R/W | DDR data port |
| 0x06 | R | DDR data port with the two bytes swapped |
| 0x08 | R | DDR controller status: FIFO flags, [1] calibration done, [0] reset |
| 0x09 | R | DDR read and write FIFO counts |
| 0x0C | R | [15:12] write sequencer status, [2:0] read sequencer status |
| 0x46 / 0x47 | R | DDR read pointer of the event read out, high / low |
| 0x01 | R/W | link to FPGA1: W [0] `LinkStatEn`. R [15:8] link-full counter, [5] transmit FIFO full, [4] empty, [0] `LinkStatEn` |
| 0x10 | W | one word into the transmit FIFO of the link to FPGA1 |
| 0x48 | R | trace of the words sent to FPGA1, pops |
| 0x80 | R | transmit overflow counter |

### Port match (`PortMatch`, from version `0x0012`)

Aim: for every request sent to the FEBs one event part goes to FPGA1, in the
order of the requests. `PortMatch` reads the four header words of each FEB
answer into registers and keeps a list of the four newest open requests, taken
from the request packets the uC loads (first word low byte `0x82`). The match
is on after reset. With the match off an event is built when all active ports
hold a complete answer, as before; the event then empties the list. With it on,
for the oldest open request E:

| Header of an active port | What happens |
|---|---|
| EWT of E | the port is in the event |
| EWT of another open request | the port is missing in this event and keeps its answer |
| EWT not in the list | the port buffer is emptied (read until it stays empty) |
| none | wait; with a time out the port is missing when it runs out |

The event is built when every active port is in or missing. A missing port sets
its port bit, bit 9 and bit 13 in the status word of the part (bit 9 becomes
bit 25 of the event status in FPGA1). If all ports are missing a part without
data is sent. The list drops its oldest entry when a fifth request is pushed.

The list is emptied by the reset of the port buffers (CSR bit 0), while the
write to the DDR is off (CSR bit 5 = 0), and by the run start reset that the uC
writes to FPGA1 (bit 0 to bus address `0x002`, `GTPRxRst`). The run start reset
also clears 0x83..0x8B. It does not change the enable bit or the time out.
Requests loaded while the write to the DDR is off are not kept; with the match
on, their answers are emptied from the port buffers as "not in the list".

A data FPGA without active port puts nothing in the list and builds nothing.

| Addr | R/W | Meaning |
|---|---|---|
| 0x81 | R/W | W [0] port match on (reset: on; 0 gives the old rule), [15] clear the status registers 0x83..0x8B. R [0] on, [3:2] state (0 deciding, 1 event offered, 2 event being written), [6:4] open requests, [15:8] ports with a header |
| 0x82 | R/W | time out in units of 10 us, 0 = none. Reset: 300 (3 ms, an estimate) |
| 0x83 / 0x84 | R | time from the oldest open request becoming the oldest to the event, last / maximum, in 10 us. With the match off: from the first request after the previous event |
| 0x85 | R | events built |
| 0x86 | R | requests seen (also counted without active port) |
| 0x87 | R | port buffers emptied |
| 0x88 | R | events with missing ports |
| 0x89 | R | events built by the time out |
| 0x8A | R | requests dropped from a full list |
| 0x8B | R | [7:0] missing ports of the last event with missing ports, [15:8] ports emptied since the last clear |
| 0x8C / 0x8D | R | EWT of the last header that was not in the list, high / low |
| 0x8E | R | [10:8] port of that header, [7:0] ports being emptied now |
| 0x8F / 0x90 | R | oldest open request, high / low. Valid if 0x81 [6:4] is not 0, otherwise an old value |
| 0x91 | R | first word of the last packet loaded for the FEBs |

Counters stop at `0xFFFF`. The enable bit and the time out return to their
reset values with the reset from the CPLD (`CpldRst`).

Only the short request packet (command `0x82`, section 6) is recognised. If
0x86 does not count or 0x91 does not read `0x0182` for one request per packet,
the list stays empty and the match must be switched off.

**Write strobe.** The write strobe of the uC is asynchronous to `SysClk`. In
version `0x0012` the synthesis had made two copies of the flip-flop that samples
it (`WRDL(0)`); the copies could see the edge in different clocks and the port
match then took one write twice (list entries like `0xA5870182`). From `0x0013`
`PortMatch` has its own two flip-flops for the strobe, and neither these nor
`WRDL`/`RDDL` of the top level may be duplicated (attributes in the source).
The strobe of `PortMatch` comes one clock later than `WRDL = 1`, so the write
strobe must be low for at least 30 ns (the uC is set to 5 bus clocks).

With the time out at 0 a part can be late: if a FEB missed request E, the part
for E is completed only when the answers to the next request arrive, and is
then sent directly before that next part. A time out shorter than the DTC time
out avoids this.

### PHY management

| Addr | R/W | Meaning |
|---|---|---|
| 0xFF | R/W | [1:0] PHY chain select, [2] read |
| 0x100..0x17F | W | MDIO write, register address in the low address bits |
| 0xFD / 0xFE | R | MDIO read data |
| 0x42 | W | SPI write to the clock fan-out chip |

A write of bit 0 = 1 to bus address `0x002` (the FPGA1 `GTPRxRst` register) is
also decoded by the data FPGAs as their `GTPRxRst`.

## 6. Data formats

### Event part on the link, data FPGA to FPGA1

| Word | Content |
|---|---|
| 0 | word count of this part, including these 4 words (bits 12..0 are used) |
| 1 | status: [7:0] ports with an error or missing, [10:8] error kind ([9] is also set for a missing port), [11] uB mismatch between ports, [12] overflow in the data FPGA (counted in FPGA2 0x80), [13] ports missing (port match, not used by FPGA1) |
| 2 | EWT bits 31..16 |
| 3 | EWT bits 15..0 |
| 4.. | data |

### FEB answer in a port buffer

| Word | Content |
|---|---|
| 0 | word count, including these 4 words |
| 1 | EWT bits 31..16 |
| 2 | EWT bits 15..0 |
| 3 | status |
| 4.. | data |

The data FPGA takes the EWT of the part from the first active port and compares
the other ports against it (status bit 11). With the port match on the EWT of
the part is that of the request, and only ports with that EWT are in the part.

### uB request from the uC to the FEBs

Written by `PHY_LOAD_DAQ_K28SEND_BCAST_MINI` (`uC/source/Mu2e_Ctrl_DAQ_PHY.c`) to
`0x301`, then sent with a 1 to `0x302`.

| Word | Content |
|---|---|
| 0 | command `0x82` and the number of requests n (one byte each) |
| 1, 2 | request 1: EWT bits 15..0, then bits 31..16 |
| 3, 4 | request 2 if n > 1, and so on |

### Event in `EventBuff`

| Word | Content |
|---|---|
| 0 | number of data words (sum of the three parts, without their headers) |
| 1 | status bits 31..16 |
| 2 | status bits 15..0 |
| 3 | EWT bits 31..16 |
| 4 | EWT bits 15..0 |
| 5.. | data of link 0, link 1, link 2 |

Status: [23:0] ports with an error (8 per link), [26:24] error kinds, [27]..[29]
error in link 0..2, [30] uB mismatch (between links, or between ports of a data
FPGA), [31] truncated in FPGA1 or overflow in a data FPGA. FPGA1 cuts an event
at 16357 data words.

Bits [27]..[29] are set when the status word of that link has bit 11 or bit 12
set, so `0x48000000` means that data FPGA 0 saw different EWTs on its own
ports; a mismatch only between the links gives `0x40000000`. Bit [25] comes
from bit 9 of a link status word: an error of that kind reported by a FEB, or,
with the port match on, a port that did not answer the request (the port bit in
[23:0] says which).

**Swapped halves.** The builder reads link word 2 (EWT high) in state
`RdUbLow` into bits 15..0 of `uB0/uB1/uB2`, and word 3 (EWT low) in `RdUbHigh`
into bits 31..16. So `uBcheck(31 downto 16)` is the low half of the EWT. The
tag used by the EWT match is `uBcheck(31 downto 16)`. Registers 0x88..0x8D show
the uB numbers with the halves in that swapped order.

## 7. Build and version

- **Compile:** open the `.xise` in ISE 14.7 Project Navigator and run "Generate
  Programming File". Results: `FPGA1/ControllerFPGA_1.bit`,
  `FPGA2/Controller_FPGA2.bit` (not tracked in git). Check "Timing Score: 0" in
  the `.par` report.
- **Version:** register 0x99, set by hand in the read mux of each top level.
- **Git hash (FPGA1):** registers 0x96/0x97 show bits 31..0 of the constant in
  `git_hash_pkg.vhd`. `python updateVersion.py` in `FPGA1/` writes the current
  `HEAD` into that file, so run it after committing and compile again. The
  resulting change of `git_hash_pkg.vhd` is itself uncommitted.
- **Simulation (FPGA1):** `bash FPGA1/sim/run_ghdl.sh` runs `EwtMatch_tb.vhd`
  with GHDL (local, or the `ghdl/ghdl` container via podman or docker). It
  covers `EventBuilder`, `PacketFormer`, `DRHandler` and `LinkFlushCtrl` with
  FIFO models, and compares the transmitted words for normal, DCS and fake
  events with the firmware of commit `513c2ce` (before the EWT match). `run_ghdl.sh full` runs only the
  test cases. Not covered: the Xilinx FIFO cores, the top level.
- **Simulation (FPGA2):** `bash FPGA2/sim/run_ghdl.sh` runs `PortMatch_tb.vhd`
  on the complete FPGA2 top level with behavioural models of the Xilinx cores
  (`FPGA2/sim/core_models.vhd`): the port match cases, then a comparison of the
  events sent to FPGA1 with the firmware of commit `513c2ce` (before the port match). Not covered: the
  real cores, the PHY transmit side, the link serializer.
- **Timing (FPGA2, build of `0x0013`):** all constraints met. Smallest setup
  slack 0.31 ns on `SysClk` and 0.18 ns on `RxFMClk`, in the crossings between
  these two clocks (port buffer pointers), which are timed as related clocks.
  Hold slack on `SysClk` is 0.00 ns, in the FM receiver of a port (0.01 ns in
  the build of `0x0012`). One path in the memory controller is relaxed to
  13.6 ns in the constraint file (`TS_MIG_DRP_FIX`).
