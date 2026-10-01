# Testing

`ctest` (from the build directory) runs all of it:

- every scenario in `tests/scenarios/`
- `tests/unit_tests.c`
- a target build of each `core/` file with `arm-none-eabi-gcc`, if it's
  installed

Everything is built with ASan and UBSan by default (`-DVCU_SANITIZE=OFF` to
turn that off).

## Scenarios

A scenario is a timeline for the simulated car:

```
t=3400   pedal=450
t=3500   apps2_duty=400                    # APPS2 drifts
t=3550   expect_no_fault=APPS_IMPLAUSIBLE  # inside 100 ms
t=3620   expect_fault=APPS_IMPLAUSIBLE expect_torque_max=0
```

Each tick, inputs stamped at or before `t` are applied, the VCU runs once,
then the checks stamped at or before `t` run. Percentages are x10
(`pedal=450` is 45.0 %). Lines can be in any order.

The parser is strict. An unknown key, a value that isn't a number, or a
token without `=` fails the scenario, so a typo can't turn into a check that
never runs.

`./build/run_scenarios -v file.scn` prints the state every 100 ms, which is
the quickest way to work out why a new scenario isn't doing what you
expected.

The simulated BMS sends a frame every 100 ms and the inverter every 10 ms
(`bms_period` / `inv_period` to change it). Keep that in mind when timing
checks around timeouts.

### Inputs

| key | what |
|---|---|
| `pedal`, `brake` | both channels of that pedal, in 0.1 % |
| `apps1_mv`, `apps2_duty`, `brake1_mv`, `brake2_mv` | one channel, raw |
| `ts`, `rtd`, `sdc` | TS request, R2D button, shutdown loop closed (0/1) |
| `dir` | 0 neutral, 1 forward |
| `bms_alive`, `inv_alive` | device sending at all |
| `bms_stuck`, `inv_stuck` | rolling counter frozen |
| `bms_crc_bad`, `inv_crc_bad` | frames fail checksum |
| `bms_fault`, `inv_fault` | device reports its own fault |
| `bms_period`, `inv_period` | frame period in ms |
| `soc`, `cell_mv`, `amp_limit`, `charge_limit` | BMS values (0.1 %, mV, 0.1 A) |
| `rpm`, `temp_motor`, `temp_inv` | inverter values |
| `pc_open` | precharge resistor open circuit |

### Checks

| key | passes when |
|---|---|
| `expect_state` | state name matches (`DRIVE`, `FAULT`, ...) |
| `expect_fault`, `expect_no_fault` | named fault set / not set |
| `expect_torque_min`, `expect_torque_max` | torque command in 0.1 Nm |
| `expect_brake_min`, `expect_brake_max` | arbitrated brake, 0.1 % |
| `expect_air_pos`, `expect_air_neg` | relay commanded (0/1) |
| `expect_enable` | inverter enable |
| `expect_buzzer` | R2D sound |
| `expect_brake_light` | brake light |
| `expect_sdc_assert` | VCU holding its SDC switch open |
| `expect_log_min` | at least n rows waiting in the datalog |
| `expect_log_dropped` | exactly n rows dropped |

Keep every scenario, including the boring ones. The pile of them is what
lets us change something and know we didn't break the shutdown path.
