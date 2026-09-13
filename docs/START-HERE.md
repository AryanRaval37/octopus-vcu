# Start here — building a VCU with no hardware

*For: first VCU, S32K344 + S32DS/RTD/FreeRTOS eventually, i.MX 95 for the ML
side, on a Mac today with a Windows laptop arriving later.*

---

## Part 1 — What a VCU actually is

Strip away the jargon and a VCU is **a state machine with CAN I/O and a lot
of paranoia**. It is not algorithmically hard. Almost none of it is clever.
The difficulty is entirely in two places: integrating with other people's
under-documented CAN devices, and getting the failure paths right.

It is the box that decides **how much torque the motor makes, right now, and
whether the car is allowed to move at all.** Everything else supports that.

### What it does, concretely

**1. Read the driver's intent.**
The accelerator pedal has **two independent sensors** with different slopes.
Not for redundancy in the backup sense — so that a *disagreement is
detectable*. One sensor cannot tell you it's lying; two can. You sample both
at 1 kHz, convert to 0–100 %, and check them against each other constantly.
Same idea for the brake.

**2. Refuse to move when something is wrong.**
This is the part that makes it a safety system rather than a throttle map:

- **APPS plausibility** — if the two pedal sensors disagree by more than 10 %
  of travel for more than 100 ms, torque goes to zero and *stays* zero until
  the driver physically releases the pedal. (FSAE rule T.4; the same logic
  exists in every production EV under a different name.)
- **Brake pedal plausibility (BPPC)** — throttle above 25 % while the brakes
  are applied means something is stuck. Torque to zero until the pedal drops
  below 5 %. Releasing the brake is *not* enough.
- **Range checks** — a sensor reading 0.0 V or 5.0 V is a broken wire, which
  is a different fault from "the two disagree", and you want to know which.
- **BSPD** — this one is **analogue hardware, not code**. A circuit that
  opens the shutdown loop when hard braking and high HV power happen at once.
  Implementing it in the MCU doesn't satisfy the rule and doesn't make the car
  safe.

**3. Sequence the high voltage.**
You cannot just close a contactor onto a 400 V pack — the inverter's DC-link
capacitors look like a dead short, and you weld the contactor shut. So:

1. Close the negative contactor (AIR−).
2. Close a **precharge relay** that feeds the DC link through a resistor.
3. Watch the DC-link voltage climb (you read it from the inverter over CAN).
4. When it reaches ~95 % of pack voltage, close AIR+ and open the precharge
   relay.
5. If it doesn't get there in time, or isn't *rising* at all — fault, open
   everything. A precharge resistor that has gone open-circuit is a real
   failure and the timeout alone won't tell you which failure it was.

**4. Run the vehicle state machine.**

```
INIT → LV_READY → PRECHARGE → TS_ACTIVE → RTD_WAIT → DRIVE
                      ↓            ↓          ↓        ↓
                      └────────────┴──────────┴────────┴──→ FAULT
```

`RTD_WAIT` is the "ready to drive" gate: the driver must press the brake
**and** the button, and the car must make a noise for at least a second
before it can move. That's a rule, and it's a good one — an EV is silent and
a car that can lurch without warning is dangerous in a pit lane.

**5. Turn pedal position into a torque command.**
The order is a safety property, not a style choice:

```
pedal → torque map → BMS current limit → thermal derate → cell derate
      → overspeed → regen blend → rate limit → STATE GATE
      → PLAUSIBILITY GATE → CAN
```

(In the scaffold every stage is there except the BMS current limit, which
needs a motor torque constant to convert amps to newton-metres. The slot is
left in the code with a comment, rather than filled with a guess.)

The two gates are **last**, and they're assignments (`= 0`), not clamps. Put
them anywhere else and some future derate calculation can hand torque back
after a safety check has taken it away. Every other step may only ever reduce
magnitude.

**6. Talk to the BMS and the inverter.**
Read state of charge, pack voltage, minimum cell voltage, current limits,
temperatures, fault flags. Send a torque command at a fixed rate — typically
100 Hz. Two non-obvious things:

- **Every received signal needs a timeout.** A stale reading that still looks
  plausible is more dangerous than a missing one.
- **The torque frame must be sent unconditionally, even in FAULT** — with zero
  torque and the enable bit clear. Most inverters disable output if frames
  stop, but silence is not a *defined* safe state. Explicit zero is.

**7. Handle faults with a policy, not with scattered `if`s.**
Every fault has a severity (warn / derate / limp / critical), a debounce
time, a heal time, and whether it latches. One table. When someone asks
"what does the car do if the BMS drops out?", the answer is one row, not a
grep across the codebase.

**8. Log everything.**
Not optional. You cannot debug a moving vehicle without it.

### How much work is that?

Roughly **2,500–4,500 lines of application C**, and **7–10 person-weeks**
after the platform works. For scale, a real FSAE team's VCU
(`sfuphantom/vcu-fw`, TMS570 + FreeRTOS) is 561 commits. And the arXiv
write-up of a four-wheel-drive Formula Student car — a team that machines
its own hub motors — runs its VCU on a bought ETAS ES910.

Not to discourage you. To set the bar: **a bench demo where a pedal
commands torque and the safety checks demonstrably cut it is a genuinely
good outcome.** A car that drives is a different project.

---

## Part 2 — How to organise the code

There is a scaffold in this repo. It builds, and the test suite passes:

```sh
cmake -S . -B build && cmake --build build
./build/run_scenarios tests/scenarios/*.scn
./build/vcu_demo
```

### The one rule

**The VCU is a pure function.**

```c
vcu_step(&vcu, &in, &out);
```

Inputs in, outputs out. No clocks, no pins, no CAN, no RTOS anywhere inside
it. The caller gathers inputs, calls the function, writes the outputs.

Everything good follows from that:

- The identical code runs on your Mac and on the S32K344. **You can start
  today.**
- A test can inject a sensor failure at exactly t=3500 ms, forty times, in a
  second. You cannot do that in a car, and those are the paths that matter.
- When the Windows laptop and the board arrive, you write *one* new file —
  the port — and the VCU is already tested.
- If the S32K344 disappoints you, switching silicon is a weekend.

### Layers

```
app/   state machine · torque arbitration · fault manager    ← pure logic
svc/   pedal acquisition · plausibility · precharge          ← pure logic
sig/   CAN encode/decode (generated from a DBC)              ← generated
hal/   ~15 functions: can, adc, gpio, time, nvm, wdt, log    ← headers only
port/  posix/ · s32k3/                                       ← one dir per target
```

**Dependency rule:** `src/` includes only `include/vcu/` and the C standard
library. If a file under `src/` ever includes `Flexcan_Ip.h` or
`FreeRTOS.h`, the design has broken. That rule is the whole architecture.

### Things worth copying from the scaffold

**Fixed point, with the units in the type name.** `nm_x10_t`, `mv_t`, `dv_t`,
`pct_x10_t`. No floats in the control path. And a separate type for
high voltage: a 400 V pack does not fit in a `uint16_t` of millivolts, and
making that a *compile error* rather than a code review item is worth the
five minutes.

**Warnings as errors from line one**, including `-Wconversion`. Cheap now,
impossible to retrofit at 4,000 lines. It caught the voltage bug above
before the code ever ran.

**Modules report, the fault manager decides.** `fault_report(f, id,
present)` every tick — including the false ones, which is how healing works.
Severity, debounce and latching live in the table.

**Calibration in a struct, not `#define`s.** You will want to change the
torque map in a car park without recompiling, and tests want to construct
odd configurations deliberately.

**Time is an input, never read.** Every module takes `dt_ms`. That's what
makes deterministic replay possible, and it means a scheduler that runs long
can't silently shorten the 100 ms plausibility window.

### Scenario tests

A test is a text file that reads like a description of a drive:

```
t=3400   pedal=450
t=3500   apps2_mv=2000                       # channel 2 drifts: 17 % off
t=3550   expect_no_fault=APPS_IMPLAUSIBLE    # 100 ms hasn't elapsed
t=3620   expect_fault=APPS_IMPLAUSIBLE expect_torque_max=0
```

**Keep every scenario you ever write.** By the end you should have 30–50.
That corpus is worth more than the code — it's what lets you change the
torque map on a Friday and know you didn't break the shutdown path. Run it
in CI (GitHub Actions is free for public repos).

Four bugs this scaffold's own tests and compiler caught, left in the README
because they're the point: the millivolt unit overflow; a **double-latching**
bug where `APPS_IMPLAUSIBLE` was latched both in the pedal module and again
in the fault table, meaning the driver could never recover by releasing the
pedal, which is exactly what the rule says should happen; a narrowing
conversion that a newer compiler refused to build at all; and a **dead code
path** — the plant's thermal model truncated to zero at a 1 ms tick, so the
temperatures never moved and the whole thermal derate had never once run.
That last one is the argument for a plant that pushes back: nothing else
would have told me.

---

## Part 3 — What to do now, with no hardware

Nothing below needs a board, a Windows laptop, or a debug probe.

### Week 1 — get the loop closed

- Build the scaffold, read it in the order the README suggests, run the demo.
- Write **three new scenarios of your own**. Suggestions: inverter drops out
  mid-drive; motor overtemperature ramping into a derate; driver requests
  reverse while rolling forward. Writing tests is how you'll discover what
  you actually think the car should do.
- Push it to GitHub with CI running the suite.

### Week 2 — the CAN layer

This is the one that decides whether the project works, and it's paperwork,
not programming.

- **Find out what inverter and BMS you'll have, and get their CAN protocol
  documents in writing.** If you can't, the VCU is unbuildable on any
  silicon. Design against a simulated inverter and say so.
- Write a `.dbc` for your vehicle — even if you're inventing it. Use
  **`cantools`** (`pip install cantools`) to generate the C encode/decode:
  `python -m cantools generate_c_source vehicle.dbc`. It produces
  dependency-free C that compiles on your Mac *and* on the S32K344. Never
  hand-write bit packing; that's where the bugs live.
- Fill in `sig/`: the signal database with a `last_rx_ms` and `valid` flag
  per signal, and a timeout for every one.

### Week 3 — a real virtual CAN bus

Move the simulated inverter and BMS out of your test harness and onto an
actual bus, so you can point real tools at them.

On Apple Silicon, `vcan` needs a Linux kernel. Two options:

- **`python-can`'s `virtual` bus** — pure Python, no kernel module, works
  natively on macOS. Start here; it's enough to script an inverter node.
- **An arm64 Linux VM** (UTM, Lima, or Docker) for real SocketCAN:
  `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link
  set up vcan0`. Then `candump vcan0` and `cansend` behave exactly as they
  will on the bench. Docker Desktop's kernel may or may not have the `vcan`
  module; a full VM definitely does.

Either way you end up able to run `candump` against your own VCU, which is
the same skill you'll use on the real bus.

### Week 4 — learn the platform on paper

- Read the S32K3 reference manual's **Boot** chapter (the IVT is mandatory
  and is a classic first-week wall) and **AN14893** on linker files and
  startup code.
- Read NXP's RTD product brief and skim the **`*_Ip`** example list — that
  suffix is the non-AUTOSAR marker and those are the only examples you care
  about.
- Read the Zephyr `mr_canhubk3` board page even though you're going the RTD
  route. It documents the FS26 jumper dance and the connector pinouts better
  than anything else, and those are hardware facts that apply either way.
- Order the J-Link, the 12.0 V bench supply, and the 3.3 V USB-TTL adapter.

### What to cut

AUTOSAR, MCAL, tresos, any ASIL claim. Secure boot, HSE, lifecycle
transitions, UDS, bootloader/OTA. A custom PCB. Traction control, sensor
fusion, path planning. High voltage — do 48 V on the bench.

---

## Part 4 — Resources worth your time

Curated, not exhaustive. Check links before relying on them.

### Read these first — open-source VCU code

- **[GEVCU](https://github.com/collin80/GEVCU)** — old, and the hardware is
  dated, but it's the clearest textbook example of exactly the state machine
  described above. Read it in an afternoon.
- **[eVCU](https://github.com/marlinarnz/eVCU)** — ESP32/FreeRTOS, small,
  well-structured. Good model of device abstraction and task decomposition.
- **[ZombieVerter](https://github.com/damienmaguire/Stm32-vcu)** — the most
  *used* open VCU; drives Tesla, Leaf, Prius and Outlander inverters out of
  the box. Read it for the inverter protocol implementations, which are hard
  to find documented anywhere else. Sold assembled by EVBMW.
- **[sfuphantom/vcu-fw](https://github.com/sfuphantom/vcu-fw)** — a real FSAE
  team's VCU (TMS570 + FreeRTOS). Read it for scale calibration and for how
  they structure the APPS/BSE tasks.
- **[openinverter wiki VCU comparison](https://openinverter.org/wiki/VCU_Comparison)**
  — the map of this ecosystem.

### Rules — read the actual document, not a summary

- **[Formula Student Germany rules](https://www.formulastudent.de/fsg/rules/)**
  — the EV and T sections. Even if you're not competing, these are the best
  free written specification of EV safety requirements in existence, and
  they explain *why*. Sections T.4 (APPS), EV.4 (BPPC), EV.5/EV.6 (shutdown,
  AIRs, precharge), EV.10 (RTD).
- **Formula Bharat** publishes its own rules, largely derived from FSG.
- **[fswiki.us](https://fswiki.us/)** — student-written explainers of
  electronic throttle control and the shutdown circuit.

### CAN — the one thing to actually study

- **[CSS Electronics' CAN bus intro](https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial)**
  — the best free "zero to competent" material anywhere, including their
  CAN-FD and DBC explainers. Start here; it's a few hours.
- **[Kvaser's CAN protocol tutorial](https://kvaser.com/can-protocol-tutorial/)**
  — deeper on the wire protocol, error frames and bus-off. Read it when you
  hit your first error storm.
- **[cantools](https://github.com/cantools/cantools)** and
  **[python-can](https://python-can.readthedocs.io/)** — your actual tools.
  Learn `cantools` properly; it generates C, decodes logs, and plots signals.
- **[SavvyCAN](https://savvycan.com/)** — free cross-platform CAN analysis
  GUI, works on macOS. A reasonable free alternative to CANalyzer for
  reverse-engineering an undocumented bus.

### Books — three that are actually worth it

- **Elecia White, *Making Embedded Systems*** (2nd ed., O'Reilly). The best
  single book for exactly your position. Chapters on architecture and on
  managing state are directly applicable to this project.
- **James Grenning, *Test-Driven Development for Embedded C***. This is the
  book that argues for the architecture in this scaffold. If you read one
  thing about *how* to structure embedded code, make it this.
- **Barr Group's Embedded C Coding Standard** (free PDF). Short, opinionated,
  and mostly right. Adopt a subset and put it in `CONTRIBUTING.md`.

Skip, for now: the MISRA C standard (buy it if a customer demands it, not
before), and anything with AUTOSAR in the title — though it's worth learning
the vocabulary (BSW, RTE, MCAL, SWC) so NXP's documentation makes sense.

### NXP-specific

- **NXP Community S32K knowledge base** — the KB articles are better than the
  official docs for anything practical. Particularly the RTD install guide
  and the non-AUTOSAR `Siul2` example.
- **[Application Code Hub](https://mcuxpresso.nxp.com/appcodehub)** — filter
  for S32K3. Working example projects built on S32DS + RTD.
- **[MCU on Eclipse](https://mcuoneclipse.com/)** (Erich Styger) — the best
  independent embedded blog going. Less S32K3 than i.MX/Kinetis, but the
  posts on debugging, linker files, FreeRTOS and probes are excellent and
  broadly applicable.
- **[SEGGER's S32K3xx wiki page](https://kb.segger.com/NXP_S32K3xx)** — read
  it before you connect the probe, not after. ECC RAM init and debug
  authentication are covered there and nowhere else as clearly.

### Testing and simulation

- Grenning's book above covers host-based testing of embedded C properly.
- **Unity + CMock** (ThrowTheSwitch) if you want a fuller unit-test framework
  than the scenario runner. **Ceedling** wraps them; it's Ruby-based and
  somewhat dated but widely used in this space.
- For the ML side later: **comma2k19** on HuggingFace is MIT-licensed, 33
  hours of highway driving **with raw CAN** — real ego speed and
  lead-vehicle radar. It's the best free ground truth for validating
  distance and time-to-collision estimates.

---

## The short version

You are not blocked on hardware. The VCU's value is in logic that runs
anywhere, and the failure paths — the part that actually matters — can *only*
be tested off the car. Build the logic and the test corpus now; when the
board and the Windows laptop arrive, you write one port file and you're
already tested.

And before you spend money: **confirm you can get the CAN protocol document
for your specific inverter and BMS.** That's the real critical path.
