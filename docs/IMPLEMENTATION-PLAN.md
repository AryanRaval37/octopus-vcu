# Research findings and implementation plan

Written 2026-09-17 as research. **Phases 0-5 have since been implemented** —
see the status note at the end of Part 5 for what landed and what did not.
The research and the reasoning are left as written.

Sources are linked inline. Every rule citation is tagged with the rulebook and
version it came from, because **the numbering in your code is from an older
rulebook and no longer matches** — see Part 3.

---

## The short version

1. **eVCU is not the answer.** It is good code, but it is ESP32/Arduino-only and
   its event-driven threading model would cost you the thing that makes this
   repo valuable — deterministic replay. Read it for ideas, don't port it.
2. **There is no single standard way, but there is a convergent one**, and you
   already have it. Two serious independent FSAE codebases arrived at the same
   shape you have: pure logic core + HAL boundary + thin per-target port +
   host-based tests. That is the validation you were looking for.
3. **C++ is fine on the S32K3 and S32DS ships the C++ compiler.** But classes
   are not the reason to switch, and "classes instead of structs" would buy you
   almost nothing. There is exactly one compelling reason, and it is that a
   safety claim currently written in `types.h` **is false today and C++ makes it
   true at zero runtime cost.** Proof in Part 2.
4. **Your rule citations are stale** — your rulebook is **Formula Bharat 2027
   v1.2** (now in the repo), which renumbered everything: `T.4` is now `T11.8`,
   `EV.4` no longer exists. There are **seven** findings in Part 3, one of which
   (`shutdown_ok`) is a genuine safety hole.

---

# Part 1 — What already exists

## eVCU — the repo you found

[marlinarnz/eVCU](https://github.com/marlinarnz/eVCU). MIT licence, C++, ~57
commits.

**Architecture:** a `VehicleController` owns `Parameter` objects (thread-safe
boxes holding a bool/int/double, each with an ID) and `Device` objects (Pedal,
Contactors, IgnitionSwitch, DeviceCAN, DeviceSPI…). Devices subscribe to
parameter IDs; when a parameter changes, subscribers get notified and run. Each
device runs in its own FreeRTOS task.

**Verdict: read it, don't adopt it.** Three reasons:

- **It is ESP32-locked.** Hardware drivers are the ESP32 Arduino API and the
  threading is ESP32 FreeRTOS. On an S32K344 you would be rewriting all of it,
  at which point you have kept only the idea.
- **The pub/sub + thread-per-device model trades away determinism**, which is
  precisely what you currently have and should protect. Your `vcu_step()` is a
  pure function: same inputs, same outputs, every time. That is why a scenario
  file can inject a sensor failure at exactly t=3500 ms and get the same answer
  on every run. In an event-driven design where seven tasks react to value
  changes, "what happened at t=3500 ms" depends on the scheduler. You cannot
  replay it, so you cannot regression-test the failure paths — and the failure
  paths are the entire point of a VCU.
- **The thing that attracts you to it is available without it.** Its appeal is
  that it looks organised — classes, subscriptions, separation. You can have all
  of that structure with a synchronous core. See Part 2.

**What is genuinely worth stealing:** the `Device` abstraction as a *HAL shape*
— `DevicePin`, `DeviceCAN`, `DeviceSPI` as a small set of interfaces with one
implementation per target. That maps cleanly onto the `hal/` directory your
`START-HERE.md` already plans.

## The ones that matter more

| Project | MCU | Lang | Host tests | Licence | Why you care |
|---|---|---|---|---|---|
| [evfirmware-vcu](https://github.com/Talluri-Ram/evfirmware-vcu) | STM32F7 | C + FreeRTOS | **Yes** — Unity + lcov | **GPL** | Explicitly FSAE-tailored. Closest thing to a reference implementation of what you are building. |
| [concordia-fsae/firmware](https://github.com/concordia-fsae/firmware) | STM32 | C/C++/Rust | **Yes** — "Rig" sim framework | confirm | 1535 commits, active. Best CAN tooling of anyone: YAML → C + Rust + DBC codegen. |
| [sfuphantom/vcu-fw](https://github.com/sfuphantom/vcu-fw) | TMS570 | C + FreeRTOS | No | confirm | A real team's task decomposition and CLI debug tooling. Last release Mar 2023. |
| [ZombieVerter](https://github.com/damienmaguire/Stm32-vcu) | STM32 | C++ | No | likely GPL | **The inverter protocol implementations.** Tesla/Leaf/Prius/Outlander CAN, documented nowhere else. |
| [GEVCU](https://github.com/collin80/GEVCU) | Due | C++ | No | GPL | Old. Clean textbook state machine, readable in an afternoon. |

### Licence warning

`evfirmware-vcu` is **GPL**, and ZombieVerter and GEVCU almost certainly are
too. Reading them for structure is fine and normal. **Copy-pasting code from
them makes your VCU GPL**, which you would have to publish. eVCU is MIT, which
is permissive. Confirm Concordia's licence before borrowing. Make this a
conscious choice rather than an accident.

### So is there a standard way?

No formal one. But note what `evfirmware-vcu` and Concordia independently
converged on, neither knowing about the other:

- a **vehicle-logic layer** that does not touch hardware
- a **vehicle-interface / HAL layer** in between
- a **hierarchical state machine**: power-up → LV ready → HV startup → drive,
  with fault reachable from anywhere
- **dual-sensor plausibility** with range checks and disagreement timers
- **host-based tests** (Unity for one, a simulation rig for the other)

That is your architecture, point for point. You are not starting from scratch
and you are not off in the weeds — you independently landed on the shape the
serious teams use. The useful conclusion is not "adopt someone's repo", it is
**"steal their CAN tooling and their test corpus ideas, keep your core"**.

---

# Part 2 — C or C++?

## Short answer

C++ is supported and is a defensible choice, **but not for the reason you gave.**
Classes would buy you very little. One specific C++ feature would buy you
something real.

## Is it even possible? Yes.

- S32DS ships `arm-none-eabi-g++` alongside `gcc` in its `build_tools` (the
  NXP community threads about it are all people fixing PATH settings, not people
  being told it is unsupported) —
  [NXP community](https://community.nxp.com/t5/S32-Design-Studio/S32DS-3-4-Program-quot-arm-none-eabi-g-quot-not-found-in-PATH/m-p/1490673).
- **RTD itself is C**, developed to MISRA C:2012 and ISO 26262 up to ASIL D,
  with a non-AUTOSAR API —
  [RTD product brief](https://www.nxp.com/docs/en/product-brief/RTD-S32K3-PB.pdf).
  You call it from C++ through `extern "C"`. Most vendor headers carry the guard
  already; some NXP ones historically do not, and then you wrap them yourself.
- Automotive C++ is a real, standardised thing: **AUTOSAR C++14** (free to
  download) and **MISRA C++:2023**. Both ban exceptions, RTTI, `new`/`delete`,
  and recursion —
  [AUTOSAR C++14 guidelines](https://www.autosar.org/fileadmin/standards/R22-11/AP/AUTOSAR_RS_CPP14Guidelines.pdf).

## Why "classes instead of structs" is the wrong reason

Your core is a pure function:

```c
vcu_step(&vcu, &in, &out);
```

That is already the right design, and it is the reason you can test any of this.
Virtual dispatch buys you nothing here — you have exactly one implementation of
each module and you never swap one at runtime. Inheritance would add vtables,
indirection, and a layer of "where does this actually go?" to a codebase whose
main virtue is that you can follow it top to bottom.

Put differently: `apps_step(&a, cfg, in, dt)` becoming `a.step(cfg, in, dt)`
changes nothing about correctness, testability, or clarity. It is the same code
with the first argument moved.

## The one real reason

**There is a safety claim in your `types.h` right now that is not true.**

```c
typedef uint16_t mv_t;   // millivolts -- LV and single cells
typedef uint16_t dv_t;   // decivolts  -- HV; 4000 = 400.0 V
```

The comment next to it says mixing the two units "is now a compile error rather
than something a reviewer has to spot". `README.md` and `START-HERE.md` repeat
the claim.

It is false. A C `typedef` is an *alias*, not a new type. `mv_t` and `dv_t` are
both `uint16_t` and therefore the same type. Verified:

```c
dv_t pack = 4000;
mv_t cell;
cell = pack;      // decivolts into millivolts -- a unit bug
```

```
cc -std=c11 -Wall -Wextra -Werror -Wconversion -Wsign-conversion
=> COMPILED CLEAN -- the unit mix was NOT caught
```

What `-Wconversion` actually caught, back when this was written, was the
**overflow** (400,000 does not fit in a `uint16_t`). It never had any ability to
catch unit confusion. The lesson in the README is real; the mechanism described
is not.

In C++ you can make it true:

```cpp
template <typename Tag, typename Rep>
struct Unit {
    Rep v;
    constexpr explicit Unit(Rep x) : v(x) {}
    constexpr Rep raw() const { return v; }
};
struct MilliVoltTag; struct DeciVoltTag;
using mv_t = Unit<MilliVoltTag, uint16_t>;
using dv_t = Unit<DeciVoltTag,  uint16_t>;
```

```
error: no viable overloaded '='
    cell = pack;
    ~~~~ ^ ~~~~
note: no known conversion from 'Unit<DeciVoltTag, ...>'
      to 'const Unit<MilliVoltTag, ...>'
```

**And it costs nothing.** Compiled at `-O2`, a function taking the wrapped type
and one taking the raw `uint16_t` produce byte-identical arm64 assembly:

```
_scale_wrapped:              _scale_raw:
    add  w8, w1, w0              add  w8, w1, w0
    and  w0, w8, #0xffff         and  w0, w8, #0xffff
    ret                          ret
```

That is the argument. Not classes — **types**.

Two smaller C++ wins in the same spirit:

- `enum class` for `fault_id_t` and `vcu_state_t`, so they stop implicitly
  converting to `int`. Today `fault_report(f, 7, true, dt)` compiles.
- `constexpr` validation of `vcu_cfg_t` — the default calibration could be
  checked at compile time (`derate_start < derate_stop`, percentages ≤ 1000,
  `cell_derate_mv > cell_min_mv`), turning a class of calibration mistakes into
  build failures.

## What C++ costs you

Be honest about this list before deciding:

- `-fno-exceptions -fno-rtti -fno-threadsafe-statics`, no heap, no allocating
  STL (`std::string`, `std::map`, `std::function`, `iostream` are all out).
- `extern "C"` wrapping for RTD headers that lack guards.
- **Static initialisation order.** Objects with constructors in different
  translation units initialise in unspecified order, and on bare metal they run
  from `.init_array` before `main`. The standard mitigation is: no non-trivial
  global constructors, `constexpr` or explicit `init()` instead.
- The RTD examples, NXP community answers, and every S32K3 tutorial are in C.
  You will be translating when you get stuck.
- If safety argumentation ever matters, you inherit MISRA C++:2023 /
  AUTOSAR C++14 rather than the simpler MISRA C:2012 that RTD already follows.

## The fair C alternative

You can get distinct types in C today:

```c
typedef struct { uint16_t v; } mv_t;
typedef struct { uint16_t v; } dv_t;
```

These *are* distinct types and C will reject mixing them. The cost is `.v`
everywhere and no operators — `a.v + b.v` instead of `a + b`, and every
arithmetic helper written by hand. That ergonomic gap is the honest trade, and
for a codebase this size it is a real but survivable annoyance.

## Recommendation

**Move to C++17, restricted subset, and do it now — but adopt it for types, not
for objects.** Keep the pure-function architecture exactly as it is. Change
`vcu_step` into a method only if you later find a reason; there isn't one today.

Timing is the main argument: you are at ~2,300 lines and the decision is nearly
free. Once the CAN layer lands — generated C from `cantools`, plus signal
plumbing — the switch gets substantially more annoying.

**If you would rather not**, the fallback is: stay in C, use the
`struct`-wrapper trick for the two voltage units that actually matter, and fix
the false claim. That is a good outcome too. What is *not* acceptable is leaving
the claim in place, because someone will trust it.

**Either way: the false unit claim appears in three files** — `include/vcu/types.h`,
`README.md`, and `docs/START-HERE.md`. All three need correcting.

---

# Part 3 — Your rule citations are stale

**Governing rulebook: Formula Bharat 2027, version 1.2 (28.06.2026)** — the PDF
you dropped in the repo root. I read it directly.

It declares itself an *"Adaptation of FS-Rules 2026 v1.1"* on every page footer,
and I verified the rules below against both documents: **the numbering and every
numeric value are identical.** The only cosmetic difference is spacing —
Formula Bharat writes `T11.8`, FSG writes `T 11.8`. Use the Formula Bharat
form in code comments.

That means the FSG 2026 rulebook is a usable cross-reference when you want more
context, and anything I say below holds for both.

## Renumbering

| Your code says | FB 2027 / FSG 2026 says | Topic |
|---|---|---|
| T.4 | **T11.8** | Accelerator Pedal Position Sensor |
| — (missing) | **T11.9** | System Critical Signal (SCS) |
| BSPD "FSAE rules" | **T11.6** | Brake System Plausibility Device |
| EV.4 (BPPC) | **no numbered rule** — but still required, see below | — |
| EV.5/EV.6 | **EV5.6** / **EV5.7** | AIRs / Pre-Charge Circuit |
| EV.10 | **EV4.11** / **EV4.12** | Activating TS / Ready-to-Drive Sound |

## What the rules actually say

- **T11.8.9** — implausibility is "a deviation of more than ten percentage
  points pedal travel between any of the used APPSs **or any failure according
  to T11.9**". Your 10.0 pp threshold is correct.
- **T11.8.8** — implausibility persisting >100 ms means motor power shut down
  immediately. Your 100 ms is correct.
- **T11.8.6** — analog sensors need different, non-intersecting transfer
  functions. Your inverted channel 2 satisfies this. Note it says *analog* — see
  your PWM question in Part 4.
- **T11.8.12** — fully released pedal must give wheel torque **≤ 0 Nm**.
- **EV5.7.1** — pre-charge to **≥95 %** before closing the second AIR. Your
  `precharge_target_pct = 950` is correct.
- **EV4.12.1** — R2D sound for **at least 1 s and at most 3 s**. Your 1500 ms
  is compliant; note there is an upper bound, which your code does not enforce
  as such (it happens to comply because the value is a constant).
- **EV2.2.1 / EV2.2.2** — TS power **≤ 80 kW**, TS current **≤ 500 A**.
- **EV2.2.4** — "Wheels must not be spun in reverse."

## Gaps found in the current code

**1. `shutdown_ok` is only checked in `LV_READY`. (Safety hole.)**
`EV4.11.8`: "The R2D mode must be left immediately when the SDC is opened." In
`state.c` the shutdown circuit is only consulted on the `LV_READY → PRECHARGE`
transition. If the SDC opens while the car is in `DRIVE`, nothing in the state
machine notices. This is the most important item in this document.

**2. No power or current limit.** `EV2.2.1/2.2.2` cap TS power at 80 kW and
current at 500 A, and enforcing that is the VCU's job. This is the same slot as
the "BMS current limit" hole already marked in `torque.c` — but the rules make
it a *power* limit first and foremost, which reframes the stage: you need
`torque × rpm` against a kW ceiling, not only amps.

**3. Reverse is forbidden.** `EV2.2.4` says wheels must not be spun in reverse,
but `torque.c` step 7 happily negates torque for `DIR_REVERSE`. Either drop
reverse entirely or gate it behind something that cannot be reached on track.

**4. No CAN message integrity.** `T11.9.2(d)` requires digitally transmitted
signals to be protected against **data corruption (e.g. checksum)** and **loss
and delay of messages (e.g. timeouts)**. You have the timeout half via `valid`;
you have no rolling counter or CRC. `T11.9.4` caps allowed delay at **500 ms**
— your 200 ms / 100 ms timeouts are comfortably inside.

**5. The APPS severity is arguably wrong — and this one is an interpretation,
not a finding.**

`FAULT_APPS_IMPLAUSIBLE` is `FAULT_SEV_CRITICAL`, which sends the state machine
to `FAULT`, which opens both AIRs. But **T11.8.8** says explicitly: "It is not
necessary to completely deactivate the TS, the motor controller(s) shutting down
the power to the motor(s) is sufficient."

Meanwhile **T11.9.5** says that for signals influencing wheel torque, the safe
state *is* "opened SDC and opened AIRs" — and **T11.8.9** folds SCS failures
into the same word "implausibility".

So the two rules use one word for two things, and my reading is that they should
be split in the code:

| Condition | Rule | Response |
|---|---|---|
| Channels deviate >10 pp for >100 ms | T11.8.8 | Motor power off. TS **may stay active**. |
| Channel open / shorted / out of range | T11.9.2, T11.9.5 | Open SDC **and** AIRs. |

Your `FAULT_APPS_RANGE` already maps to the second row. `FAULT_APPS_IMPLAUSIBLE`
currently gets the second row's treatment when it should arguably get the first.

**Present this to your scrutineers before implementing it.** Being stricter than
the rules is not a rules violation, so the current behaviour is safe — it just
makes the car annoying to recover in a way the rules do not require. Do not
change it on my reading alone.

**6. The BPPC is still required — but the rulebook no longer defines its numbers.**

I checked this carefully because it decides the fate of
`03_brake_plausibility.scn`. The answer is more interesting than "it was removed".

- There is **no numbered technical rule** in FB2027 (or FSG 2026) defining a
  brake/throttle plausibility check with thresholds. `T11.8` has exactly twelve
  sub-rules and none of them is one. Searching the flattened text, no occurrence
  of `plausibilit*` falls within 300 characters of `brak` anywhere except the
  BSPD.
- **But `A6.4.4` explicitly lists it as a system that must be working**, in the
  minimum requirements for a safe testing environment: *"Working TSAL, IMD, AMS,
  ASSI, RES, EBS, **APPS/brake pedal plausibility check**, APPS, and ETC
  plausibility check if applicable"*.
- The `25 %` figure that does still appear is `T11.6.1`, and it is **CV only**
  (throttle >25 % over idle, for the BSPD). `CV1.6.5`'s ±5 % is also CV only.
  Neither applies to your car.

So: the check is still expected to exist and function, but its thresholds are
now **yours to choose and justify**. The historical 25 % / 5 % values come from
older FSAE/FSG rulebooks and remain the sane default — they are what scrutineers
will recognise.

**Keep the test and keep the implementation.** Change only the citation: cite
`A6.4.4` for *why it exists*, and note in the comment that the thresholds are
carried over from earlier rulebooks rather than mandated by this one. Right now
`apps.c` and the scenario both cite "FSAE EV.4", which is a rule number that no
longer exists in your rulebook.

**7. The brake sensors are System Critical Signals.**
`T6.1.13` permits the first 90 % of brake pedal travel to be used for
regeneration. The moment brake position influences torque, `T11.9.1` makes it
an SCS ("all electrical signals which … influence the wheel torque"), which
drags in the full open-circuit / short / range / timeout treatment. This matters
for your two-brake-sensor question below.

---

# Part 4 — Your open questions, answered

These are the TODOs you left in the code.

### "One big car-state struct, or smaller per-subsystem structs?"

**Neither extreme. Keep `vcu_in_t` as the input boundary, and split the
*internal* state by owner.**

Your instinct that one giant struct is "basically global variables" is right —
but note what makes globals bad is *unrestricted mutation*, not size. `vcu_in_t`
is passed as `const` everywhere below `vcu_step`, so nobody can write to it. A
large read-only input snapshot is a fundamentally different thing from a large
mutable global, and it is what makes the tick reproducible.

Concretely:

- `vcu_in_t` stays as it is: one immutable snapshot of the world per tick. This
  is the thing that makes replay work; don't break it up.
- Each module keeps owning its own mutable state (`apps_t`, `state_mgr_t`,
  `torque_t`, `fault_mgr_t`). That is already correct.
- Pass `const vcu_in_t*` down. Modules read the fields they need. The
  "redundancy" you are worried about costs nothing — it is one pointer.

The one change worth making: several modules take `const vcu_in_t*` but only
touch two or three fields. Where that is true, **pass the fields instead** — it
documents the real dependency and makes the unit test obvious. `apps_step` only
needs `apps1_mv`, `apps2_mv`, `brake_mv`. That is a much clearer signature than
"the whole world".

### "Where should sensors be read? Who calls the HAL?"

**The port calls the HAL. Never `vcu_step`.** You already answered this
correctly in your own note.

```
port/s32k3/main.c        <- the ONLY file that calls HAL
  ├─ hal_adc_read()  ─┐
  ├─ hal_can_rx()     ├─> fills vcu_in_t
  └─ hal_time_ms()   ─┘
          │
          v
     vcu_step(&vcu, &in, &out)     <- pure, no HAL, no clock
          │
          v
  ├─ hal_gpio_write(AIR_POS, out.air_pos)
  └─ hal_can_tx(out.torque_cmd)
```

If `vcu_step` ever calls a HAL function, the scenario runner cannot run it on
your Mac, and the whole test corpus dies. That boundary is the design.

### "Should the VCU see raw mV, or already-converted percentages?"

**Raw, and this is worth defending** — you flagged it as suspicious, but it is
deliberate.

The conversion is not arithmetic, it is *policy*: range checking, slope
handling, and the plausibility comparison all depend on the raw value. If the
port converts to a percentage first, then:

- a channel reading 0 mV (cut wire) and a channel reading exactly 0 % become
  indistinguishable — you have thrown away the fault before the VCU sees it;
- the calibration (`apps1_lo_mv` etc.) moves into the port, so it is no longer
  in the NVM-tunable config struct;
- your scenario files lose the ability to say `apps2_mv=2000`, which is how
  `02_apps_disagreement.scn` works.

The rule of thumb: **the port does unit conversion that is a fact about the
hardware (ADC counts → millivolts). The VCU does conversion that is a decision
(millivolts → pedal percent).**

### "APPS: one analog, one PWM. Different types?"

This is a real design change and the rules touch it.

- **T11.8.6** requires different non-intersecting transfer functions *if analog
  sensors are used*. With one analog and one PWM you are not in that clause for
  the pair — but you must still satisfy **T11.8.5** (two separate sensors) and
  **T11.9** for both signals.
- A PWM sensor fails differently. "Out of range" for analog is voltage outside a
  window; for PWM it is **duty cycle out of range, frequency wrong, or no edges
  at all**. The third is the important one — a stalled PWM line holds its last
  duty reading forever and looks perfectly healthy. You need an edge timeout.

**Suggested shape:** keep the *arbitration* logic unit-agnostic by having the
port present both channels as already-validated percentages **plus a validity
flag each**, and keep the raw value alongside for logging:

```c
struct { pct_x10_t pos; bool ok; mv_t raw_mv; }  apps1;   // analog
struct { pct_x10_t pos; bool ok; uint16_t duty; } apps2;  // PWM
```

This is the one place I would bend the "raw in" rule, because the two channels
genuinely have no common raw unit. The plausibility comparison then operates on
two percentages, which is what the rule is written about anyway ("deviation of
more than ten percentage points **pedal travel**").

### "Brake has two sensors too."

Correct, and the code is currently wrong about this — `apps.c` says "one sensor,
nothing to arbitrate", which was my error.

Given `T6.1.13` (regen on brake travel) makes brake position torque-influencing
and therefore an SCS under `T11.9.1`, the brake pair needs the same treatment
as APPS: range check each channel, compare them, and define what happens when
they disagree. It is close to a copy of the APPS logic, which suggests factoring
out a shared "dual redundant sensor" helper rather than writing it twice.

Note the asymmetry: for APPS you take the **lower** of the two channels (fail
toward less torque). For brake you should take the **higher** (fail toward more
braking / more caution about BPPC).

### "Multithreading — do I need mutexes around the state?"

**Almost certainly not, if you keep the current design.** This is a significant
benefit of the architecture you already have.

The standard structure is: one task owns the `vcu_t` and calls `vcu_step`. Other
tasks (CAN RX, ADC sampling) write into a *staging* buffer. Once per control
period the VCU task takes a consistent snapshot into `vcu_in_t` and runs. You
need protection only at that one handoff, and the cheapest correct options are:

- do the snapshot with interrupts briefly disabled (it is a `memcpy` of a small
  struct), or
- a double-buffer with an atomic index flip, or
- a FreeRTOS queue per signal group.

No mutex inside the VCU, because nothing inside the VCU is shared. `vcu_t` is
touched by exactly one task. That is worth protecting as an invariant — write it
down, because the first person to call `vcu_step` from two tasks will silently
break it.

The `START-HERE.md` task rates (1 kHz safety, 100 Hz control) imply the safety
task and control task both touch pedal data. Simplest resolution: run the whole
`vcu_step` at 1 kHz and only *transmit* at 100 Hz. You have the headroom, and
one task means no sharing.

### "Fault severities — I have no clue about the difference, I'll probably use CRITICAL for everything."

Don't collapse them, but **you can collapse two of the five today**.

- `INFO` / `WARN` — keep as one. The difference is only whether it reaches
  telemetry, and you have no telemetry yet. Merge into `WARN`.
- `DERATE` — genuinely distinct and you now have two real users:
  overtemperature and cell sag. It means "reduce torque proportionally, stay in
  DRIVE". Without it, a slightly warm motor stops the car dead, mid-event.
- `LIMP` — "hard cap, stay in DRIVE". Currently used by `BPPC` and `OVERSPEED`.
  This one is the weakest of the five; you could fold it into `DERATE` with a
  factor of zero and lose little.
- `CRITICAL` — "open the contactors, go to FAULT". Non-negotiable, and per
  `T11.9.5` it is the rules-mandated safe state for SCS failures.

So the defensible minimum is **three: WARN, DERATE, CRITICAL.** The reason not
to use CRITICAL for everything is `EV2.2` and endurance: a car that opens its
contactors because the motor reached 91 °C does not finish the event. Derate is
how you trade performance for finishing.

### "Drive modes — do we even have these?"

Probably not, for a first car. `DRIVE_MODE_ECO` / `SPORT` currently only select
a different `torque_max`. Keep the *array* indexed by mode (it costs nothing and
the config struct is already shaped for it) but ship with one mode populated
until a driver actually asks for a second. Delete `ECO` if it is never wired to
a physical switch — an unreachable state is a state you cannot test.

### "Coordinate with Manish about the BMS."

The specific things to get from him, in priority order:

1. **The CAN protocol document** — message IDs, byte layout, endianness, scaling
   and offset for every signal. In writing. This is the critical path for the
   whole project.
2. **Does the BMS send a rolling counter and/or checksum?** Required by
   `T11.9.2(d)`. If it does not, you need a different integrity story and you
   both need to know that now, not in scrutineering.
3. **Transmission period** for each message, so you can set timeouts. `T11.9.4`
   caps you at 500 ms regardless.
4. **What the BMS does autonomously** — does it open the AIRs itself on a cell
   fault, or does it expect the VCU to? Overlapping assumptions here are how
   contactors get welded.
5. **Discharge and charge current limits** — units, update rate, and whether
   they are instantaneous or 10-second ratings.

---

# Part 5 — The plan

Ordered so that each phase is useful on its own and nothing blocks on hardware.

### Phase 0 — Decide the language, fix the false claim (1 session)

Do this before anything else; it gets more expensive every week.

1. Decide C++17-restricted vs staying in C (Part 2).
2. Correct the `mv_t`/`dv_t` claim in `types.h`, `README.md`, `START-HERE.md`
   — whichever language you pick.
3. If C++: set up the build with `-fno-exceptions -fno-rtti
   -fno-threadsafe-statics`, convert the two voltage units to strong types,
   `enum class` for `fault_id_t` / `vcu_state_t`, and confirm all 80 checks
   still pass. Nothing else changes.

### Phase 1 — Re-cite the rules, fix the safety gap (1–2 sessions)

4. Commit the Formula Bharat 2027 PDF into `docs/rules/` so the citations have a
   reference that travels with the repo.
5. Fix `shutdown_ok` — `EV4.11.8`, leave R2D immediately when the SDC opens.
   Write the scenario first: in DRIVE, drop `shutdown_ok`, assert the state
   leaves DRIVE and torque goes to zero.
6. Re-cite every rule comment against the confirmed rulebook.
7. Take the APPS severity split (Part 3, gap 5) to your scrutineers. Implement
   only after they confirm.

### Phase 2 — Sensors, properly (2–3 sessions)

8. Factor a shared dual-redundant-sensor module: range check, disagreement
   timer, arbitration direction (low for accelerator, high for brake).
9. Add the second brake channel and its plausibility, per `T11.9`.
10. Model the PWM APPS channel: duty range, frequency check, **edge timeout**.
11. Scenarios for each new failure mode. One per failure, named after it.

### Phase 3 — The CAN layer (the real critical path)

12. **Get the inverter and BMS protocol documents in writing.** If you cannot,
    the VCU is unbuildable on any silicon and you should say so loudly and early.
13. Write a `.dbc` for the vehicle, inventing it if necessary.
14. Generate C with `cantools generate_c_source` — never hand-write bit packing.
    Look at Concordia's YAML→codegen approach if you want to go further.
15. Build `sig/`: per-signal `last_rx_ms`, `valid`, rolling counter and checksum
    validation per `T11.9.2(d)`.
16. Extend the plant model to *send* frames, so scenarios can corrupt a counter
    or delay a message and assert the VCU notices.

### Phase 4 — Torque pipeline completion

17. Implement the power limit: `EV2.2.1` 80 kW, `EV2.2.2` 500 A. This fills
    the slot currently marked as missing in `torque.c`, and needs the motor
    torque constant from the datasheet.
18. Resolve reverse against `EV2.2.4`.
19. Enforce `T11.8.12` explicitly — released pedal gives wheel torque ≤ 0.

### Phase 5 — Datalogging

20. Not optional. You cannot debug a moving vehicle without it, and the scenario
    corpus is only as good as your ability to compare it against real runs.
    Log the full `vcu_in_t` / `vcu_out_t` pair per tick to start — it is small,
    and it means a real drive can be replayed through the simulator.

### Phase 6 — Target bring-up

21. `port/s32k3/main.c` and the `hal/` implementations. One new directory, no
    changes to `src/`.
22. Read the S32K3 **Boot** chapter (the IVT is a classic first-week wall) and
    **AN14893** on linker files and startup. If you went C++, also confirm
    `.init_array` is processed by NXP's startup code before `main`.

---

# Part 6 — What I need from you

Decisions only you can make:

1. **C++17-restricted, or stay in C?** (Part 2 — my recommendation is C++, for
   types rather than classes, and now rather than later.)
2. ~~Which rulebook and year?~~ **Answered** — Formula Bharat 2027 v1.2, the PDF
   in the repo root. It is an adaptation of FS-Rules 2026 v1.1 and the numbering
   and values match. Worth moving that PDF somewhere deliberate (`docs/rules/`)
   and committing it, so the citations in the code have a reference that travels
   with the repo.
3. **Is reverse actually needed?** `EV2.2.4` suggests not.
4. **Do you have telemetry planned?** Decides whether `INFO`/`WARN` survive.
5. **Confirm the APPS severity interpretation with a scrutineer** before I touch
   `fault.c`.

## Status

**Done:** the restructure into `core/` + `board/` (replacing app/svc/sig/hal/
port); stayed in C and removed the false unit claim from all three files;
drive modes and reverse removed; the `shutdown_ok` hole fixed (EV4.11.8); the
APPS severity split implemented (LIMP vs CRITICAL); power and current
ceilings implemented, which also filled the BMS discharge-limit slot by going
through watts instead of needing a motor constant; the second brake channel
and its plausibility; CAN message health per T11.9.2.d; datalogging; every
rule citation moved to FB2027 numbering. 12 scenarios, 140 checks.

**Still open:** CAN bit packing (`sig/`, generate from a DBC with
`cantools`) — this is the critical path and it is blocked on getting the
inverter and BMS protocol documents. Then NVM, a real torque map, and the
S32K3 port.

**Still needs your answer:** the APPS severity interpretation (Part 3,
finding 5) is implemented on my reading of T11.8.8 vs T11.9.5. Being stricter
than the rules is safe and being looser is not, so confirm it with a
scrutineer before the car runs.
