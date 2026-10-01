# Design notes

How the VCU code is organised and why, plus the places where we had to
interpret the rules. If you're new, read this, then `core/types.h`, then
`core/vcu.c`, and follow the calls from there.

## The core doesn't know about hardware

Everything in `core/` is called through one function:

```c
vcu_step(&vcu, &in, &out);
```

`in` is a snapshot of everything the VCU is allowed to know this tick (pedal
readings, buttons, what arrived on CAN, the time). `out` is everything it
can do (torque command, relays, buzzer, brake light, SDC switch). The board
layer fills one and applies the other.

Nothing in `core/` reads a clock, touches a register, or calls the RTOS.
That's what makes the rest possible:

- The same code runs in the scenario tests on a laptop and on the car.
- A test can break a sensor at exactly t=3500 ms and get the same result
  every run. We can't do that on the car, and failure handling is most of
  what a VCU does.
- Moving to different hardware means rewriting `board/`, not the logic.

On the target, one task owns the `vcu_t` and calls `vcu_step` at 1 kHz.
CAN RX and ADC can run in interrupts or other tasks and write into a staging
buffer, and the VCU task copies that into `vcu_in_t` once per tick (with
interrupts off for the copy, or a double buffer). Nothing inside `core/` is
shared between tasks, so there are no locks in there. Calling `vcu_step`
from two tasks would break that, so don't.

## Layout

```
core/        the controller
  sense/     raw inputs -> things we can trust (message health, pedals)
  decide/    what to do about them (faults, state machine, torque)
board/       one directory per target; the only place hardware exists
sim/         plant model + scenario runner (host only)
tests/       scenarios and unit tests
```

`core/` includes only `core/` and the C standard library. `ctest` builds
every `core/` file with `arm-none-eabi-gcc` to keep that honest.

## One tick

In `core/vcu.c`, in this order:

1. Work out `dt` from `now_ms` (clamped to 1 s).
2. If the driver just released the TS request, acknowledge latched faults.
3. Sense: CAN message health for the BMS and inverter, then both pedal
   pairs.
4. Report every fault condition to the fault manager.
5. State machine. Any CRITICAL fault goes straight to FAULT.
6. Torque pipeline.
7. Outputs: relays from the state, torque, brake light, SDC switch,
   inverter enable.
8. Datalog.

Faults come before the state machine so a critical fault takes effect in
the same tick. State comes before torque because torque is zero outside
DRIVE.

## Conventions

**Fixed point, unit in the type name.** `nm_x10_t` is 0.1 Nm, `pct_x10_t` is
0.1 %, `dv_t` is 0.1 V, and so on. No floats in the control path, because
a NaN torque command gets past every comparison. The catch: these are
typedefs, so `mv_t` and `dv_t` are the same type to the compiler and mixing
them up won't error. We looked at wrapping each unit in a struct (C) or a
template (C++) to make that a compile error, and decided the extra syntax
everywhere wasn't worth it at this size. If we ever get bitten by a unit
mix-up, revisit that.

**Time is an input.** Every module gets `dt_ms`. Rates are per ms, not per
tick (the torque slew limit, for example), so changing the task rate
doesn't change behaviour.

**No malloc.** The caller allocates every struct (statically on the
target), and `*_init()` just fills in memory that already exists.

**Modules report, the fault manager decides.** Every condition is reported
every tick, including when it's false (that's how healing works). Severity,
debounce, heal time and latching all come from the one table in
`decide/faults.c`, so "what happens if the BMS drops out?" is one row.

**One owner per latch.** If pedals.c latches the BPPC, the fault table
doesn't latch it again. Otherwise it's unclear which one is holding the car
and the driver can end up unable to clear it.

**Safety gates go last and set zero.** In `torque.c` every step can only
make the command smaller, and the state/fault gates at the end assign 0
rather than clamp. A derate added later can't give torque back after a
safety check took it away.

**Fail in the boring direction.** Two APPS channels disagree: believe the
lower. Brake channels disagree: the higher one is used for the BPPC and the
brake light, but R2D needs both to say the brake is on.

**Calibration is data.** Every number is in `vcu_cfg_t` (`core/config.c`).
`vcu_cfg_valid()` rejects calibrations that break a rule or the code's
assumptions, and the VCU sits in FAULT with `BAD_CONFIG` if given one.

## Our reading of the rules

These are decisions where the rulebook left room. Each one should be run past
a scrutineer (or at least the team) before the event.

**APPS disagreement vs APPS wiring fault.** T11.8.8 says for an APPS
implausibility, shutting motor power is enough and the TS can stay active.
T11.9.5 says the safe state for a failed SCS is SDC and AIRs open. Both
rules call their case an "implausibility". We split them:

- channels more than 10 pp apart for 100 ms: LIMP (torque off, HV stays up,
  driver lifts to recover)
- a channel outside its valid window: CRITICAL (SDC and AIRs open)

A channel out of range also counts toward the same 100 ms timer, and the
pedal reads zero immediately, so one noisy sample doesn't latch anything but
a real break stops torque at once.

**BPPC thresholds.** A6.4.4 requires the brake/APPS plausibility check to
work but FB2027 doesn't give numbers. We use 25 % to trip and 5 % to reset,
from the older FSAE/FSG rules, since that's what scrutineers will expect.

**R2D.** EV4.11.7 says the transition may only happen while the brakes are
on, together with a dedicated action. EV4.11.6 says R2D is the moment the
motors respond to the APPS. So we go to DRIVE (motors live) on the exact
tick where all of these are true:

- rising edge of the R2D button (so a stuck or held button doesn't count)
- both brake channels say the brake is on
- throttle below 5 %, so the car can't pull the instant it goes live
- no LIMP or worse fault holding torque off, since that clearing later would
  be the real start of R2D, possibly with no brake

The sound (EV4.12.1, "while entering R2D mode") plays over the first 1.5 s
of DRIVE. We looked at playing the sound first and enabling torque after
it, but then R2D really starts when the sound ends, and the driver may be
off the brake by then.

**The VCU's own SDC switch.** The VCU opens its switch in the shutdown loop
for its own CRITICAL faults (T11.9.5). It doesn't hold it open just because
the loop is open. If our sense point turns out to be after our own switch,
holding it would mean we see our own open loop forever and never leave
FAULT.

**Power limit.** EV2.2.1 and EV2.2.2 (80 kW, 500 A) are measured at the
accumulator outlet, so they're electrical. We control shaft torque, so the
limit is multiplied by an assumed drivetrain efficiency before converting to
torque. This is open loop and relies on that efficiency number being
conservative. Closing the loop on measured TS current would be better once
we know what current signal we get. Regen isn't limited by the rules
(EV2.2.3), only by the BMS charge limit.

**Brake light.** T6.3.1 says on if and only if the hydraulic brake or the
electric brake is actuated, so it comes on during regen too.

**Brake sensors are SCSs.** T6.1.13 allows regen on brake travel, which
makes the brake signals torque-influencing (T11.9.1). They get range checks
and a plausibility check like the APPS.

**No reverse.** EV2.2.4. Neutral is still there as a direction.

**Boot.** A device we've never heard from isn't a timeout until
`boot_grace_ms` after power-on, because the VCU boots faster than the BMS.
Once a device has been seen, the normal timeout applies.

**The BSPD isn't here.** T11.6 wants it standalone and non-programmable.
The only software link is making sure our BPPC trips before it does (see
the note at the bottom of `core/config.h`).

## CAN, when we get to it

`core/` doesn't know about bits and bytes. It gets, per device per tick,
whether a frame arrived, its rolling counter, whether the checksum passed,
and the decoded values. The plan:

1. Write a `.dbc` for the whole car from the BMS and inverter protocol docs.
2. Generate pack/unpack code with `cantools generate_c_source`. Don't
   hand-write bit packing.
3. A small layer in the board code turns received frames into `vcu_in_t`
   fields and `vcu_out_t` into transmitted frames.

Checksum checking happens in that layer (it depends on the device's
algorithm), but deciding whether the device is still trustworthy stays in
`core/sense/signals.c`, where the tests can reach it.

## Other projects worth reading

- [evfirmware-vcu](https://github.com/Talluri-Ram/evfirmware-vcu): FSAE VCU
  on STM32 with host tests. Closest thing to what we're doing. GPL.
- [concordia-fsae/firmware](https://github.com/concordia-fsae/firmware):
  good CAN tooling (YAML to C and DBC).
- [ZombieVerter](https://github.com/damienmaguire/Stm32-vcu): inverter CAN
  protocols that aren't documented anywhere else. Likely GPL.
- [GEVCU](https://github.com/collin80/GEVCU): old, but a clear state machine.
- [eVCU](https://github.com/marlinarnz/eVCU): MIT, ESP32 and event-driven.
  Worth reading, not worth porting, because the event model loses the
  repeatable tests.

Reading GPL code for ideas is fine. Copying from it makes our code GPL too.
