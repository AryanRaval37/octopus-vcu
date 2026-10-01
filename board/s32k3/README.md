# board/s32k3

Empty until the hardware exists. Moving to the target should mean writing a
`main.c` here plus a few driver wrappers, with no changes to `core/`.

## main.c, roughly

Same three steps as `board/host/main.c`, with NXP RTD calls instead of the
simulator:

```c
static vcu_t vcu;            // static, not on the stack: it's a few KB

vcu_init(&vcu, NULL);
for (;;) {
    vcu_in_t in = {0};

    in.now_ms      = ms_since_boot();
    in.apps1_mv    = adc_read_mv(APPS1_CH);          // Adc_Sar_Ip_*
    in.apps2_duty  = pwm_duty_x10(APPS2_CH);         // Emios_Icu_Ip_*
    in.brake1_mv   = adc_read_mv(BRAKE1_CH);
    in.brake2_mv   = adc_read_mv(BRAKE2_CH);
    in.shutdown_ok = Siul2_Dio_Ip_ReadPin(SDC_SENSE);
    in.ts_request  = ...;
    in.rtd_button  = ...;
    can_fill_input(&in);                              // Flexcan_Ip_*

    vcu_out_t out;
    vcu_step(&vcu, &in, &out);

    Siul2_Dio_Ip_WritePin(AIR_POS, out.air_pos);
    ...
    can_send(&out);

    wait_for_next_ms();
}
```

A separate low-priority task calls `datalog_pop()` and writes rows to SD or
UART. It can be slow; that's what the ring buffer is for.

## What the board owes the core

- **APPS2 duty.** Convert the timer capture to 0.1 % duty (0..1000). If no
  edge has been seen for a couple of PWM periods, report 0 if the line is
  low and 1000 if it's high. Both are outside the valid window, so a dead
  line becomes APPS_RANGE. Without this a stalled PWM input just holds its
  last duty forever and looks fine.
- **CAN per device per tick:** `rx` (a frame arrived since the last tick),
  `counter` (its rolling counter) and `crc_ok` (checksum passed, computed
  here with whatever algorithm that device uses). Don't work out a "valid"
  flag here. Deciding whether a device is still trustworthy is
  `core/sense/signals.c`'s job, so the tests can see it.
- **`now_ms`** from a free-running timer, not a tick counter that can skip.
- **`shutdown_ok`** is the SDC sense input. `core/` handles the sense point
  being either before or after the VCU's own switch in the loop, but we
  should still find out which it is.

## Things that will bite

- **The boot header (IVT)** is mandatory on the S32K3 and is the classic
  first-week wall. Read the Boot chapter of the reference manual and AN14893
  first.
- **ECC RAM** has to be initialised before it's read, or the first read
  faults. The startup code usually does this, check that it does.
- **Use the non-AUTOSAR `*_Ip` RTD drivers.** Those are the ones the
  examples use.
- **uint32_t is `unsigned long` on arm-none-eabi**, so `%u` in printf is
  wrong there. Use `PRIu32`. (`ctest` catches this in `core/`.)

## Task rate

Run all of `vcu_step()` at 1 kHz in one task, and only transmit CAN at
whatever rate the inverter wants. One task owning `vcu_t` means nothing in
`core/` needs a lock. Put a hardware watchdog on that task.
