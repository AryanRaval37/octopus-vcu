# Aeolus VCU

Vehicle Control Unit firmware for an electric car. Targets the NXP S32K344;
runs and is fully tested on a development machine with no hardware at all.

```sh
cmake -S . -B build && cmake --build build
./build/run_scenarios tests/scenarios/*.scn    # the safety test suite
./build/vcu_demo                                # watch a drive cycle
```

Requires only CMake and a C11 compiler. On macOS: `brew install cmake`.

---

## The one idea

The controller is a **pure function**:

```c
vcu_step(&vcu, &in, &out);
```

No clocks, no pins, no CAN, no RTOS anywhere inside it. The caller gathers
inputs, calls `vcu_step`, and writes the outputs. That is the whole contract.

Everything good about this repo falls out of that one property:

- The identical code runs on your laptop and on the S32K344.
- A test can inject a sensor failure at exactly t=3500 ms, forty times, in a
  second. You cannot do that in a car, and those are the paths that matter.
- Porting to different silicon means rewriting `port/`, not the VCU.
- `git diff` on a behaviour change shows logic, not driver noise.

---

## Layout

```
include/vcu/     public headers — the vocabulary
src/
  vcu.c          orchestrator: dt, fault reporting, calls the modules in order
  app/
    state.c      vehicle state machine + precharge sequencer
    torque.c     pedal -> torque command, all the clamps
    fault.c      the fault table: severity, debounce, healing, latching
  svc/
    apps.c       pedal acquisition + FSAE plausibility checks
sim/
  plant.c        crude vehicle model: DC link, rpm, temperatures
  scenario.c     the .scn file runner
port/
  posix/main.c   host port — becomes the S32K344 port later
tests/scenarios/ the test corpus
```

**Dependency rule, enforced by review:** `src/` includes only `include/vcu/`
and the C standard library. If a file under `src/` ever includes
`zephyr/…`, `Flexcan_Ip.h`, or `FreeRTOS.h`, the design has broken.

---

## Reading order

1. `include/vcu/types.h` — the vocabulary. Units, states, faults.
2. `src/svc/apps.c` — smallest and highest-consequence file here.
3. `src/app/state.c` — the state machine.
4. `src/app/torque.c` — read the comment at the top before changing anything.
5. `src/app/fault.c` — the whole fault policy of the vehicle, in one table.
6. `tests/scenarios/02_apps_disagreement.scn` — what a test looks like.

Each of those files ends by naming the next one, so you can also just open
`types.h` and keep going. `src/vcu.c` is the spine — one tick of the whole
car — and is worth reading once you have seen the parts it calls.

---

## Scenario tests

A scenario is a timeline of "set this" and "assert that":

```
t=3400   pedal=450
t=3500   apps2_mv=2000          # channel 2 drifts: 17 % disagreement
t=3550   expect_no_fault=APPS_IMPLAUSIBLE   # 100 ms not elapsed yet
t=3620   expect_fault=APPS_IMPLAUSIBLE expect_torque_max=0
```

Inputs: `pedal` `brake` `apps1_mv` `apps2_mv` `brake_mv` `ts` `rtd` `mode`
`dir` `bms_alive` `inv_alive` `bms_fault` `inv_fault` `soc` `cell_mv` `rpm`
`temp_motor` `temp_inv` `pc_open`

Assertions: `expect_state` `expect_torque_max` `expect_torque_min`
`expect_fault` `expect_no_fault` `expect_air_pos` `expect_enable`

Percentages are tenths (`pedal=450` is 45.0 %). An unknown key is a hard
failure — a typo in an expectation that silently passes is worse than no
test.

**Keep every scenario you ever write.** The corpus is worth more than the
code: it is what lets you change the torque map on a Friday and know you did
not break the shutdown path.

The suite is currently seven files and 80 checks:

```
01_normal_startup      LV -> precharge -> RTD -> drive, the happy path
02_apps_disagreement   FSAE T.4, and the most important test here
03_brake_plausibility  FSAE EV.4 (BPPC)
04_precharge_faults    open resistor, and the timeout
05_bms_dropout         a CAN partner goes silent mid-drive
06_thermal_derate      motor overtemp fades torque rather than cutting it
07_cell_sag            low cell voltage does the same, on the way down
```

---

## Porting to the S32K344

Write `port/s32k3/main.c`. It is the only new file:

```c
for (;;) {
    vcu_in_t in = {0};
    in.now_ms   = OsIf_GetCounter(...);
    in.apps1_mv = adc_read_mv(APPS1_CH);      /* Adc_Sar_Ip_*  */
    in.apps2_mv = adc_read_mv(APPS2_CH);
    sig_decode(&in);                           /* Flexcan_Ip_*  */

    vcu_out_t out;
    vcu_step(&vcu, &in, &out);

    gpio_write(AIR_POS, out.air_pos);          /* Siul2_Dio_Ip_* */
    can_send_torque(out.torque_cmd, out.inverter_enable);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
}
```

Build `src/` and `include/` unchanged with `arm-none-eabi-gcc`. No `#ifdef`
in the VCU.

Task rates on target: **1 kHz** safety (ADC + plausibility + watchdog),
**100 Hz** control (torque + inverter TX), 50 Hz state machine, 10 Hz
housekeeping. The safety task owns the shutdown GPIO directly so it can open
the contactors even if everything else deadlocks.

---

## Four things the compiler and the tests already caught

Left here because they are the point:

- **`-Wconversion` caught a unit bug.** A 400 V pack does not fit in a
  `uint16_t` of millivolts. That is why HV has its own type (`dv_t`,
  decivolts) — mixing the two is now a compile error. Warnings are errors in
  this project from day one; it is cheap now and expensive at 4,000 lines.

- **A scenario test caught double-latching.** `APPS_IMPLAUSIBLE` was latched
  both in `apps.c` (per the rule) and again in the fault table, so the
  driver could never recover by releasing the pedal — which is exactly what
  FSAE T.4 says should happen. One latch, one owner. See the comment in
  `src/app/fault.c`.

- **`-Wconversion` again, and this time it stopped the build.** `return
  -max_mag` in `torque.c` promotes to `int` and narrows back to `int16_t`,
  which a newer clang rejects outright. Worth knowing that the warning set
  in this repo is strict enough that a compiler upgrade can fail it — that
  is the deal, and it is still the right deal.

- **A dead code path nobody noticed.** The plant integrated temperature with
  integer division at a 1 ms tick, so every increment truncated to zero and
  both temperatures sat at exactly 25 °C for the life of the project. The
  entire thermal derate in `torque.c` had therefore never executed once.
  `plant.h` already carried a comment warning about this exact trap for
  `rpm` — the same bug, twelve lines apart, caught once. Fixed with a
  micro-degree accumulator; `06_thermal_derate.scn` now covers the path.

---

## Not implemented yet

- **The BMS current limit stage of the torque pipeline.** The slot is left
  in the pipeline in `src/app/torque.c`, between the map and the thermal
  derate, with a comment saying so. It needs a motor torque constant to get
  from amps to newton-metres, and inventing that number would give you a
  calibration you cannot check against anything.
- `sig/` — CAN encode/decode. Generate it from a DBC with `cantools`; do not
  hand-write bit packing.
- Datalogging. Not optional in practice — you cannot debug a moving vehicle
  without it.
- NVM for calibration.
- Drive modes beyond a linear torque map.

## Deliberately not implemented

**The BSPD is not here and must not be.** Per FSAE rules it is an analogue
hardware circuit that opens the shutdown circuit on simultaneous hard
braking and high HV power, latching for at least one second. Implementing it
in the MCU does not satisfy the rule and does not make the car safe.
