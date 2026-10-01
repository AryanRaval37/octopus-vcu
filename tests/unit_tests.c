// unit_tests.c - direct tests for things a scenario can't easily reach:
// calibration checks and a few module-level behaviours.

#include "core/vcu.h"

#include <stdio.h>

static int checks, failures;

#define CHECK(cond) do {                                               \
        checks++;                                                      \
        if (!(cond)) {                                                 \
            failures++;                                                \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);   \
        }                                                              \
    } while (0)

static void test_default_config_is_valid(void)
{
    CHECK(vcu_cfg_valid(vcu_cfg_default()));
    CHECK(!vcu_cfg_valid(NULL));
}

// Each of these is a calibration someone could plausibly type in, and each
// breaks either a rule or the code's assumptions.
static void test_bad_configs_are_rejected(void)
{
    vcu_cfg_t c;

    c = *vcu_cfg_default(); c.apps_deviation_ms = 150;        // T11.8.8 says 100
    CHECK(!vcu_cfg_valid(&c));

    c = *vcu_cfg_default(); c.precharge_target_pct = 900;     // EV5.7.1 says 95 %
    CHECK(!vcu_cfg_valid(&c));

    c = *vcu_cfg_default(); c.power_max_w = 90000;            // EV2.2.1
    CHECK(!vcu_cfg_valid(&c));

    c = *vcu_cfg_default(); c.bms_timeout_ms = 600;           // T11.9.4
    CHECK(!vcu_cfg_valid(&c));

    c = *vcu_cfg_default(); c.rtd_buzzer_ms = 3500;           // EV4.12.1
    CHECK(!vcu_cfg_valid(&c));

    // Full pedal at 4.8 V with the window ending at 4.7 V would read as a
    // broken wire every time the driver floored it.
    c = *vcu_cfg_default(); c.apps1.hi = 4800;
    CHECK(!vcu_cfg_valid(&c));

    c = *vcu_cfg_default(); c.apps2.lo = c.apps2.hi;          // zero span
    CHECK(!vcu_cfg_valid(&c));

    c = *vcu_cfg_default(); c.derate_motor_start = 120;       // band backwards
    CHECK(!vcu_cfg_valid(&c));

    c = *vcu_cfg_default(); c.drive_efficiency = 0;
    CHECK(!vcu_cfg_valid(&c));
}

// Minimal healthy input: both devices talking, pedals at rest.
static vcu_in_t healthy_input(uint32_t t)
{
    const vcu_cfg_t *c = vcu_cfg_default();
    vcu_in_t in = { 0 };
    in.now_ms      = t;
    in.apps1_mv    = c->apps1.lo;
    in.apps2_duty  = c->apps2.lo;
    in.brake1_mv   = c->brake1.lo;
    in.brake2_mv   = c->brake2.lo;
    in.shutdown_ok = true;
    in.dir_request = DIR_FORWARD;
    in.bms.rx = in.inv.rx = true;
    in.bms.crc_ok = in.inv.crc_ok = true;
    in.bms.counter = in.inv.counter = (uint8_t)t;
    in.bms.pack_dv = 4000;
    in.bms.cell_min_mv = 3800;
    return in;
}

static void test_bad_config_keeps_car_off(void)
{
    vcu_cfg_t c = *vcu_cfg_default();
    c.precharge_target_pct = 500;

    vcu_t v;
    vcu_out_t out;
    vcu_init(&v, &c);

    for (uint32_t t = 0; t < 3000; t++) {
        vcu_in_t in = healthy_input(t);
        in.ts_request = (t / 500) % 2;        // driver keeps trying
        vcu_step(&v, &in, &out);
    }
    CHECK(out.state == VCU_FAULT);
    CHECK(out.faults & FAULT_BIT(FAULT_BAD_CONFIG));
    CHECK(!out.air_pos && !out.air_neg && !out.precharge_relay);
}

// The brake latch used to release on apps_reset_below. Give the two
// different values and check the brake uses its own.
static void test_brake_uses_its_own_reset(void)
{
    vcu_cfg_t c = *vcu_cfg_default();
    c.apps_reset_below  = 50;     //  5 %
    c.brake_reset_below = 300;    // 30 %

    pedals_t p;
    pedals_init(&p);
    vcu_in_t in = healthy_input(0);

    // Channels 40 pp apart for well past the window.
    in.brake1_mv = 2100;          // 40 %
    in.brake2_mv = 500;           //  0 %
    for (int i = 0; i < 400; i++) pedals_step(&p, &c, &in, 1);
    CHECK(p.brake.implausible);

    // Agreeing again at 20 %: under the brake's 30 % but over the APPS 5 %.
    in.brake1_mv = in.brake2_mv = 1300;
    pedals_step(&p, &c, &in, 1);
    CHECK(!p.brake.implausible);
}

// A 100 ms frame period with the counter frozen should go stale after
// counter_stall_ms of wall time, not after that many frames.
static void test_stale_with_slow_frames(void)
{
    sig_health_t h;
    sig_init(&h);

    uint32_t stale_at = 0;
    for (uint32_t t = 0; t < 2000 && !stale_at; t++) {
        const bool frame = (t % 100) == 0;
        sig_step(&h, frame, 7, true, 200, 300, t ? 1 : 0);
        if (h.stale) stale_at = t;
    }
    CHECK(stale_at >= 300 && stale_at <= 400);
    CHECK(!h.timeout);
}

int main(void)
{
    test_default_config_is_valid();
    test_bad_configs_are_rejected();
    test_bad_config_keeps_car_off();
    test_brake_uses_its_own_reset();
    test_stale_with_slow_frames();

    printf("unit tests: %d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
