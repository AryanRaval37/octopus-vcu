# Aeolus VCU

Vehicle Control Unit firmware for an electric Formula Student car. Targets the
NXP S32K344; runs and is fully tested on a development machine with no
hardware at all.

```sh
cmake -S . -B build && cmake --build build
./build/run_scenarios tests/scenarios/*.scn    # the safety suite
./build/vcu_demo                                # watch a drive cycle
./build/vcu_demo --csv drive.csv                # ...and keep the log
```

Requires CMake and a C11 compiler. On macOS: `brew install cmake`.

**New here? Read [`docs/START-HERE.md`](docs/START-HERE.md).** It is twenty
minutes and it puts the files in the right order.

---

## The one idea

The controller is a **pure function**:

```c
vcu_step(&vcu, &in, &out);
```

No clocks, no pins, no CAN, no RTOS anywhere inside it. The caller gathers
inputs, calls `vcu_step`, writes the outputs. That is the whole contract, and
everything good here falls out of it: the same code runs on a laptop and on
the S32K344, a test can inject a sensor failure at exactly t=3500 ms, and
porting means rewriting `board/` rather than the controller.

---

## Layout

There are two kinds of code here: code that runs on your laptop, and code
that can't.

```
core/              the controller. pure C. no hardware, no clock, no globals.
  types.h          the vocabulary
  vcu.c            one tick: sense, decide, record
  config.c         every tunable number in the car
  datalog.c        the flight recorder
  sense/           "can I trust this, and what does it mean?"
    signals.c        CAN message health: corruption, loss, delay
    pedals.c         four sensors, two pedals, plausibility
  decide/          "given that, what does the car do?"
    faults.c         the fault table: severity, debounce, healing, latching
    state.c          vehicle state machine + precharge sequencer
    torque.c         pedal -> torque command, and every ceiling on the way
board/             the machine. one directory per machine.
  host/            this machine: reads the simulator instead of hardware
  s32k3/           the real one, when the hardware arrives
sim/               a pretend car, so the controller has something to argue with
tests/scenarios/   the corpus
docs/rules/        the governing rulebook
```

**Dependency rule, enforced by review:** `core/` includes only `core/` and
the C standard library. If a file under `core/` ever includes `zephyr/…`,
`Flexcan_Ip.h`, or `FreeRTOS.h`, the design has broken.

---

## Status

12 scenarios, 142 checks, zero warnings under `-Werror -Wconversion` plus
ASan and UBSan.

Implemented: the state machine and precharge sequencer, dual-redundant
accelerator and brake sensing with the plausibility rules, CAN message health
(corruption, loss and delay per T11.9.2.d), the fault table, the torque
pipeline including the 80 kW / 500 A ceilings and the BMS discharge limit,
thermal and cell derating, and datalogging.

Not yet: CAN bit packing (`sig/` — generate it from a DBC with `cantools`),
NVM for calibration, a torque map beyond linear, and the S32K3 port.

Deliberately absent: **the BSPD.** Per T11.6 it is a standalone
non-programmable circuit that opens the shutdown circuit on simultaneous hard
braking and ≥5 kW to the motors. Implementing it in the MCU satisfies neither
the rule nor physics — one of the failures it guards against is this MCU
having hung. `core/config.h` has a note on keeping the software thresholds
consistent with that board once its varistor is trimmed.

---

## Four things the compiler and the tests caught

Left here because they are the point.

- **A dead code path nobody noticed.** The simulator integrated temperature
  with integer division at a 1 ms tick, so every increment truncated to zero
  and both temperatures sat at exactly 25 °C for the life of the project. The
  entire thermal derate had therefore never executed once. `plant.h` already
  carried a comment warning about this exact trap for `rpm` — the same bug,
  twelve lines apart, caught once. Fixed with a micro-degree accumulator.

- **A safety hole in the shutdown path.** `shutdown_ok` was only checked on
  the way into precharge, so once the car was driving, the shutdown circuit
  could open and the state machine would not notice — against EV4.11.8. The
  inertia switch, the cockpit buttons and the brake over-travel switch all
  live on that loop. Now `08_shutdown_circuit.scn`.

- **A severity that was subtly wrong.** `APPS_IMPLAUSIBLE` opened the
  contactors, but T11.8.8 says explicitly that deactivating the tractive
  system is not necessary — cutting motor power is enough. Meanwhile a
  channel going open *is* an SCS failure and T11.9.5 does want the AIRs
  open. Two failures of one sensor, two different responses.

- **A false claim in a comment.** `types.h` asserted that mixing `mv_t` and
  `dv_t` was a compile error. It was not: a C `typedef` is an alias, so both
  are `uint16_t` and the compiler is perfectly happy. `-Wconversion` had
  caught the original *overflow*, never the unit confusion. The claim is gone
  and the caveat is written down instead.

---

## Rules

Governing rulebook: **Formula Bharat 2027 v1.2**, in `docs/rules/`. It is an
adaptation of FS-Rules 2026 v1.1 and the numbering matches. Rule numbers
appear in comments wherever they drive a decision; `docs/START-HERE.md` has
the table of the ones worth knowing up front.

Note that the numbering changed from older rulebooks — what used to be `T.4`
is now `T11.8`, and `EV.4` no longer exists.

---

## Porting to the S32K344

Write `board/s32k3/main.c`. It is the only new file, and
`board/s32k3/README.md` has the sketch, the task rates, and the list of
things that will bite you. Build `core/` unchanged with `arm-none-eabi-gcc`.
No `#ifdef` in the controller.
