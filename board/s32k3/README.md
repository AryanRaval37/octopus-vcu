# board/s32k3 — the real one, when the hardware arrives

Empty on purpose. This directory is the whole cost of moving to the target:
one `main.c` and a handful of driver shims. Nothing in `core/` changes.

## What goes here

A `main.c` that does the same three things `board/host/main.c` does, with
NXP RTD calls instead of the simulator:

```c
for (;;) {
    vcu_in_t in = {0};

    /* 1. sense */
    in.now_ms    = OsIf_GetCounter(...);
    in.apps1_mv  = adc_read_mv(APPS1_CH);      /* Adc_Sar_Ip_*   */
    in.apps2_mv  = adc_read_mv(APPS2_CH);
    in.brake1_mv = adc_read_mv(BRAKE1_CH);
    in.brake2_mv = adc_read_mv(BRAKE2_CH);
    in.shutdown_ok = Siul2_Dio_Ip_ReadPin(SDC_SENSE);
    can_drain(&in);                            /* Flexcan_Ip_*   */

    /* 2. decide */
    vcu_out_t out;
    vcu_step(&vcu, &in, &out);

    /* 3. act */
    Siul2_Dio_Ip_WritePin(AIR_POS, out.air_pos);
    can_send_torque(out.torque_cmd, out.inverter_enable);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
}
```

Plus a low-priority task that calls `datalog_pop()` and writes the rows
somewhere persistent. It is allowed to be slow — that is what the ring
buffer is for.

## Things that will bite

- **The IVT.** The S32K3 boot header is mandatory and is the classic
  first-week wall. Read the reference manual's Boot chapter and AN14893
  before anything else.
- **ECC RAM must be initialised** before you touch it, or you take a fault on
  the first read. SEGGER's S32K3xx wiki page covers this and debug
  authentication better than the official docs.
- **RTD is the non-AUTOSAR `*_Ip` APIs.** Those are the examples you want;
  the AUTOSAR-flavoured ones are a different world.
- **`can_drain()` owes the core three things per talker**: `rx`, `counter`
  and `crc_ok`. Do not compute a `valid` flag out here — deciding whether a
  signal is trustworthy is safety logic, and it lives in
  `core/sense/signals.c` where a scenario test can reach it.

## Task rates

1 kHz safety (ADC, plausibility, watchdog) · 100 Hz control (torque, inverter
TX) · 50 Hz state machine · 10 Hz housekeeping.

Simplest correct arrangement: run the whole of `vcu_step()` at 1 kHz and only
*transmit* at 100 Hz. One task owns the `vcu_t`, so nothing inside the core is
ever shared and you need no locks in there at all. The safety task should own
the shutdown GPIO directly so it can open the contactors even if everything
else deadlocks.
