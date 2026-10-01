# Aeolus VCU

VCU firmware for our Formula Bharat 2027 electric car. Target is the NXP
S32K344, but everything except the board layer builds and runs on a laptop
against a simulated car, which is where all the testing happens for now.

```sh
cmake -S . -B build && cmake --build build
(cd build && ctest)                           # everything
./build/run_scenarios tests/scenarios/*.scn   # just the scenarios, with output
./build/vcu_demo                              # print a simulated drive
./build/vcu_demo --csv drive.csv              # ...and save the log
```

Needs CMake and a C11 compiler (`brew install cmake` on a Mac). If
`arm-none-eabi-gcc` is installed, `ctest` also checks that `core/` still
builds for the target.

## How it's put together

The controller is one function with no hardware in it:

```c
vcu_step(&vcu, &in, &out);
```

The board code reads the pins and CAN into `in`, calls that, and writes
`out` back to the pins and CAN. `core/` never touches a clock, a register or
an RTOS call, so the exact same code runs in the tests and on the car.
[docs/design.md](docs/design.md) explains why and goes through the rest of
the design.

```
core/            the controller (no hardware, no mutable globals, no malloc)
  types.h          units, states, fault IDs, in/out structs
  config.c         every calibration number
  vcu.c            one tick, calls everything below in order
  sense/           turning raw inputs into things we can trust
    signals.c        CAN message health: timeout, frozen counter, checksum
    pedals.c         APPS and brake pairs, plausibility, BPPC
  decide/          deciding what to do about them
    faults.c         the fault table (severity, debounce, latching)
    state.c          state machine and precharge
    torque.c         pedal -> torque, with every limit and gate
  datalog.c        ring-buffer logger
board/host/      runs core/ against the simulator
board/s32k3/     the real board (not written yet, see its README)
sim/             plant model and the scenario runner
tests/           scenarios (.scn), unit tests
docs/            design notes, testing, rulebook, open questions
```

Rule: files in `core/` only include other `core/` files and the C standard
library.

## Where it's at

Done and tested in simulation: state machine and precharge, APPS and brake
plausibility, BPPC, CAN message health, fault handling, torque limits (power,
current, BMS limits, thermal and cell derating), regen, brake light, R2D
sound, datalogging.

Not done:
- CAN encode/decode. Blocked on getting the BMS and inverter protocol docs.
- The S32K3 board layer (`board/s32k3/main.c`).
- Storing calibration in flash.
- A real torque map (it's linear right now).

A lot of numbers in `core/config.c` are placeholders.
[docs/need-from-team.md](docs/need-from-team.md) lists what we still need to
find out from the rest of the team.

The BSPD is deliberately not in here. T11.6 requires it to be a separate
non-programmable circuit.

## Rules

Everything is written against the Formula Bharat 2027 rulebook v1.2
(`docs/rules/`), which is an adaptation of FS-Rules 2026 v1.1 with the same
numbering. Rule numbers are cited in comments wherever a rule drives a
decision.
