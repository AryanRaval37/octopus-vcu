# Start here

You are about to read a Vehicle Control Unit. This page gets you oriented in
about twenty minutes, in the right order, and then gets out of the way.

```sh
cmake -S . -B build && cmake --build build
./build/run_scenarios tests/scenarios/*.scn   # the safety suite
./build/vcu_demo                              # watch a drive
```

Needs CMake and a C11 compiler, nothing else. On macOS: `brew install cmake`.

---

## What this thing actually does

A VCU decides **how much torque the motor makes, right now, and whether the
car is allowed to move at all.** That is the entire job. Everything else in
here supports one of those two sentences.

It is not algorithmically hard. Almost none of it is clever. The difficulty
is in the failure paths — what happens when a sensor lies, when the battery
stops talking, when someone hits a shutdown button at 60 km/h. That is where
the code spends its complexity, and it is why the test suite matters more
than the code does.

---

## The one idea

**The controller is a pure function.**

```c
vcu_step(&vcu, &in, &out);
```

Inputs in, outputs out. No clocks, no pins, no CAN, no RTOS anywhere inside
it. The caller gathers the inputs, calls the function, applies the outputs.

Everything good about this repo falls out of that one property:

- The identical code runs on your laptop and on the S32K344.
- A test can kill a sensor at exactly t=3500 ms, forty times, in a second.
  You cannot do that in a car, and those are the paths that matter.
- Changing silicon means rewriting `board/`, not the controller.
- `git diff` on a behaviour change shows logic, not driver noise.

If you only remember one thing from this page, remember that.

---

## How the code is laid out

There are exactly two kinds of code here: **code that runs on your laptop,
and code that can't.** That is the whole directory structure.

```
core/       the controller. pure C. no hardware, no clock, no globals.
board/      the machine. one directory per machine. the ONLY place hardware exists.
  host/       your laptop -- senses the simulator
  s32k3/      the car -- senses actual pins and CAN
sim/        a pretend car, so the controller has something to argue with
tests/      the scenario corpus
```

Inside `core/`, one more split, which is just the shape of a tick:

```
core/sense/    "can I trust this, and what does it mean?"
core/decide/   "given that, what does the car do?"
```

`sense/` turns raw millivolts and CAN frames into trustworthy facts.
`decide/` turns trustworthy facts into contactor states and a torque number.
`core/vcu.c` runs sense, then decide, then writes it down.

**The dependency rule, and it is the whole architecture:** `core/` includes
only `core/` and the C standard library. If a file under `core/` ever
includes `Flexcan_Ip.h` or `FreeRTOS.h`, the design has broken and the test
suite dies with it.

---

## Read it in this order

Each file ends by naming the next one, so you can also just open `types.h`
and keep going until you run out.

| # | File | Why |
|---|------|-----|
| 1 | `core/types.h` | The vocabulary. Units, states, faults. Everything else is written in these words. |
| 2 | `core/sense/signals.c` | Smallest file here. Is anything on this bus still worth believing? |
| 3 | `core/sense/pedals.c` | The highest-consequence file in the repo. Four sensors, two pedals, and whether to trust them. |
| 4 | `core/decide/faults.c` | Scroll to the table. That is the entire fault policy of the vehicle on one screen. |
| 5 | `core/decide/state.c` | The state machine and the precharge sequencer. |
| 6 | `core/decide/torque.c` | The torque pipeline. Read the comment at the top before changing anything. |
| 7 | `core/datalog.c` | The flight recorder. You cannot debug a moving vehicle. |
| 8 | `core/vcu.c` | One tick of the whole car. Makes more sense once you have met the parts. |
| 9 | `tests/scenarios/02_apps_disagreement.scn` | What a test looks like, and the most important one here. |

Then, when you need them: `core/config.c` (every tunable number),
`board/host/main.c` (what a board actually has to do), and
`board/s32k3/README.md` (what to write when the hardware lands).

---

## Design philosophy

Six ideas. Everything in `core/` follows from them, and if you are about to
write code that breaks one, that is worth a conversation first.

**1. Time is an input, never something you fetch.**
Every module takes `dt_ms`. Nothing calls a clock. That is what makes
deterministic replay possible, and it means a scheduler that runs long cannot
quietly shorten a 100 ms safety window.

**2. Modules report, the fault manager decides.**
`fault_report(f, id, present)` every tick — including the false ones, which
is how healing works. Severity, debounce and latching live in one table. When
someone asks "what does the car do if the BMS drops out?", the answer is one
row you can point at, not a grep across the tree.

**3. One latch, one owner.**
If two pieces of code both latch the same condition, neither is in charge and
the driver can never clear it. This has bitten this repo once already; the
comment on `FAULT_APPS_IMPLAUSIBLE` in `decide/faults.c` is the scar.

**4. Safety gates go last, and they are assignments.**
The torque pipeline may only ever make the number smaller — until the final
two gates, which set it to zero outright. Put a gate anywhere else and some
future derate step gets to hand torque back after a safety check took it
away, and it will not look like a bug when someone writes it.

**5. Fail in the boring direction.**
Two accelerator sensors disagree? Believe the lower one — the car goes slower
than asked, never faster. Two brake sensors disagree? Believe the higher one.
Same principle, pointed whichever way is dull.

**6. Calibration is data, not code.**
Every tunable number lives in `vcu_cfg_t`, because you will want to change
the torque map in a paddock without a toolchain, and because tests want to
build deliberately silly configurations to see what breaks.

---

## Scenario tests

A scenario is a timeline that reads like a description of a drive:

```
t=3400   pedal=450
t=3500   apps2_mv=2000                     # channel 2 drifts: 17 % off
t=3550   expect_no_fault=APPS_IMPLAUSIBLE  # 100 ms has not elapsed yet
t=3620   expect_fault=APPS_IMPLAUSIBLE expect_torque_max=0
```

Percentages are tenths — `pedal=450` is 45.0 %. An unknown key is a hard
failure, because a typo in an expectation that silently passes is worse than
no test: you believe it.

**Inputs:** `pedal` `brake` `apps1_mv` `apps2_mv` `brake1_mv` `brake2_mv`
`ts` `rtd` `sdc` `dir` `bms_alive` `inv_alive` `bms_fault` `inv_fault`
`bms_stuck` `inv_stuck` `bms_crc_bad` `inv_crc_bad` `soc` `cell_mv`
`amp_limit` `rpm` `temp_motor` `temp_inv` `pc_open`

**Assertions:** `expect_state` `expect_torque_max` `expect_torque_min`
`expect_brake_min` `expect_brake_max` `expect_fault` `expect_no_fault`
`expect_air_pos` `expect_air_neg` `expect_enable` `expect_log_min`
`expect_log_dropped`

`run_scenarios -v <file>` prints a state trace every 100 ms, which is the
fastest way to find out why a scenario you just wrote does not do what you
expected.

**Keep every scenario you ever write**, including the boring ones. The corpus
ends up worth more than the code — it is what lets you change the torque map
on a Friday afternoon and still know the shutdown path works. Aim for 30–50.
There are 12 today.

---

## The rules

The governing rulebook is **Formula Bharat 2027 v1.2**, a copy of which is in
`docs/rules/`. It declares itself an adaptation of FS-Rules 2026 v1.1, and
the numbering and values match, so the FSG document works as a cross
reference.

Rule numbers appear in comments wherever they drive a decision. The ones
worth knowing before you read the code:

| Rule | What it says |
|------|--------------|
| **T11.8.5/.6** | At least two accelerator sensors, on non-intersecting transfer functions |
| **T11.8.8/.9** | >10 percentage points disagreement for >100 ms → motor power off. TS need not deactivate |
| **T11.8.12** | Fully released pedal → wheel torque ≤ 0 Nm |
| **T11.9** | System Critical Signals: open circuit, short, out of range, corruption, loss, delay. Safe state is opened SDC and opened AIRs |
| **T11.9.4** | Tolerated message delay must not exceed 500 ms |
| **T11.6** | BSPD — standalone, non-programmable **hardware**. Not this code, and must not be |
| **EV2.2.1/.2** | TS power ≤ 80 kW, TS current ≤ 500 A |
| **EV2.2.4** | Wheels must not be spun in reverse |
| **EV4.11.7/.8** | R2D needs brake + a dedicated action; leave R2D immediately when the SDC opens |
| **EV4.12.1** | Ready-to-drive sound for 1–3 s |
| **EV5.7.1** | Pre-charge to ≥95 % of pack before closing the second AIR |
| **A6.4.4** | A working APPS/brake plausibility check is required to operate the car |

Two of these are worth internalising because they are easy to get backwards:

- **T11.8.8 vs T11.9.5.** Two pedal channels disagreeing means cut the
  torque and leave HV up. A pedal channel being electrically broken means
  open the contactors. Same sensor, two failures, two different cars
  afterwards.
- **T11.6 is hardware.** Implementing a BSPD in software satisfies neither
  the rule nor physics — one of the failures it guards against is this MCU
  having hung.

---

## What is not here yet

- **`sig/` — CAN encode/decode.** The *health* layer exists
  (`core/sense/signals.c`); the bit packing does not. Generate it from a DBC
  with `cantools generate_c_source`. Never hand-write bit packing.
- **NVM** for calibration.
- **A torque map** beyond linear.

And one thing that is deliberately absent: **the BSPD**, for the reasons
above.

The full plan, the survey of other open-source VCUs, and the C-versus-C++
argument are in `docs/IMPLEMENTATION-PLAN.md`.

---

## When the hardware arrives

You write one file: `board/s32k3/main.c`. `core/` is already tested.
`board/s32k3/README.md` has the sketch and the list of things that will bite
you — the IVT, ECC RAM init, and which half of RTD to use.

Before you spend money: **confirm you can get the CAN protocol document for
your specific inverter and BMS.** That is the real critical path, and no
amount of good architecture substitutes for it.
