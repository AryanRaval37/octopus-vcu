// plant.c - the pretend car. Numbers are guesses picked to give the right
// shape of response, not measured from anything.

#include "sim/plant.h"
#include "core/config.h"

#define AMBIENT_C 25
#define UC_PER_C  1000000

void plant_init(plant_t *p)
{
    const plant_t zero = { 0 };
    *p = zero;

    p->pack_dv         = 4000;   // 400.0 V
    p->soc             = 800;    // 80.0 %
    p->cell_min_mv     = 3800;
    p->discharge_limit = 2000;   // 200.0 A
    p->charge_limit    = 500;    //  50.0 A
    p->sdc_ok          = true;
    p->dir             = DIR_FORWARD;

    // Guesses until we have the real protocol docs. The point is that they
    // are not 1 ms: the VCU ticks much faster than anything on the bus.
    p->bms_alive     = true;
    p->inv_alive     = true;
    p->bms_period_ms = 100;
    p->inv_period_ms = 10;

    p->precharge_tau_ms = 400;

    plant_set_temp_motor(p, AMBIENT_C);
    plant_set_temp_inv(p, AMBIENT_C);
    plant_set_pedal_pct(p, 0);
    plant_set_brake_pct(p, 0);
}

void plant_set_pedal_pct(plant_t *p, int pct_x10)
{
    if (pct_x10 < 0) pct_x10 = 0;
    if (pct_x10 > PCT_MAX) pct_x10 = PCT_MAX;

    // Default calibration: channel 1 rises 500 -> 4500 mV, channel 2 is
    // PWM falling 90 % -> 10 % duty.
    p->apps1_mv   = (mv_t)(500 + (4000 * pct_x10) / PCT_MAX);
    p->apps2_duty = (pct_x10_t)(900 - (800 * pct_x10) / PCT_MAX);
}

void plant_set_brake_pct(plant_t *p, int pct_x10)
{
    if (pct_x10 < 0) pct_x10 = 0;
    if (pct_x10 > PCT_MAX) pct_x10 = PCT_MAX;

    p->brake1_mv = (mv_t)(500 + (4000 * pct_x10) / PCT_MAX);
    p->brake2_mv = p->brake1_mv;
}

void plant_set_temp_motor(plant_t *p, degc_t c)
{
    p->temp_motor_uc = (int32_t)c * UC_PER_C;
    p->temp_motor    = c;
}

void plant_set_temp_inv(plant_t *p, degc_t c)
{
    p->temp_inv_uc = (int32_t)c * UC_PER_C;
    p->temp_inv    = c;
}

void plant_set_rpm(plant_t *p, rpm_t r)
{
    p->rpm_x100 = (int32_t)r * 100;
    p->rpm      = r;
}

bool plant_sdc_sensed(const plant_t *p)
{
    return p->sdc_ok && !p->vcu_opened_sdc;
}

void plant_step(plant_t *p, const vcu_out_t *out, uint16_t dt_ms)
{
    p->now_ms += dt_ms;
    p->vcu_opened_sdc = out->shutdown_assert;

    // The AIR and precharge coils are fed through the shutdown loop
    // (EV6.1.1), so whatever the VCU asks for, nothing stays closed once the
    // loop is open.
    const bool loop      = plant_sdc_sensed(p);
    const bool precharge = out->precharge_relay && loop;
    const bool air_pos   = out->air_pos && loop;

    if (precharge && !p->precharge_resistor_open) {
        int32_t gap = (int32_t)p->pack_dv - (int32_t)p->dc_link_dv;
        int32_t d   = (gap * dt_ms) / (p->precharge_tau_ms ? p->precharge_tau_ms : 1);
        if (d < 1 && gap > 0) d = 1;
        p->dc_link_dv = (dv_t)(p->dc_link_dv + d);
    } else if (air_pos) {
        p->dc_link_dv = p->pack_dv;
    } else if (!precharge) {
        // Discharge circuit bleeding the link down.
        int32_t d = ((int32_t)p->dc_link_dv * dt_ms) / 2000;
        p->dc_link_dv = (dv_t)(p->dc_link_dv > d ? p->dc_link_dv - d : 0);
    }

    // Torque accelerates, drag decelerates.
    p->rpm_x100 += ((int32_t)out->torque_cmd * dt_ms) / 24;
    p->rpm_x100 -= (p->rpm_x100 * (int32_t)dt_ms) / 8000;
    if (p->rpm_x100 < 0)       p->rpm_x100 = 0;
    if (p->rpm_x100 > 2000000) p->rpm_x100 = 2000000;
    p->rpm = (rpm_t)(p->rpm_x100 / 100);

    // Heat goes in with torque and leaks back to ambient with a ~60 s time
    // constant. Tuned so that full torque with no derate would take the
    // motor to ~130 C, past its cutoff, otherwise the overtemp path could
    // never be exercised.
    int32_t heat = (out->torque_cmd < 0 ? -out->torque_cmd : out->torque_cmd);

    p->temp_motor_uc += (heat * 7 * dt_ms) / 8;
    p->temp_motor_uc -= ((p->temp_motor_uc - AMBIENT_C * UC_PER_C) * dt_ms) / 60000;
    p->temp_motor = (degc_t)(p->temp_motor_uc / UC_PER_C);

    p->temp_inv_uc += (heat * 5 * dt_ms) / 8;
    p->temp_inv_uc -= ((p->temp_inv_uc - AMBIENT_C * UC_PER_C) * dt_ms) / 60000;
    p->temp_inv = (degc_t)(p->temp_inv_uc / UC_PER_C);
}

static bool frame_due(uint32_t now_ms, uint16_t period_ms)
{
    return period_ms == 0 || now_ms % period_ms == 0;
}

void plant_to_input(plant_t *p, vcu_in_t *in)
{
    const vcu_in_t zero = { 0 };
    *in = zero;

    in->now_ms      = p->now_ms;
    in->apps1_mv    = p->apps1_mv;
    in->apps2_duty  = p->apps2_duty;
    in->brake1_mv   = p->brake1_mv;
    in->brake2_mv   = p->brake2_mv;
    in->ts_request  = p->ts_request;
    in->rtd_button  = p->rtd_button;
    in->shutdown_ok = plant_sdc_sensed(p);
    in->dir_request = p->dir;

    const bool bms_tx = p->bms_alive && frame_due(p->now_ms, p->bms_period_ms);
    const bool inv_tx = p->inv_alive && frame_due(p->now_ms, p->inv_period_ms);
    if (bms_tx && !p->bms_counter_stuck) p->bms_counter++;
    if (inv_tx && !p->inv_counter_stuck) p->inv_counter++;

    // The payload is filled in even on ticks with no frame, the same way a
    // real receive buffer keeps the last thing it got. The VCU has to work
    // out from rx/counter that the numbers are old.
    in->bms.rx              = bms_tx;
    in->bms.counter         = p->bms_counter;
    in->bms.crc_ok          = !p->bms_crc_bad;
    in->bms.soc             = p->soc;
    in->bms.pack_dv         = p->pack_dv;
    in->bms.cell_min_mv     = p->cell_min_mv;
    in->bms.discharge_limit = p->discharge_limit;
    in->bms.charge_limit    = p->charge_limit;
    in->bms.temp_max        = 35;
    in->bms.fault           = p->bms_fault;

    in->inv.rx            = inv_tx;
    in->inv.counter       = p->inv_counter;
    in->inv.crc_ok        = !p->inv_crc_bad;
    in->inv.dc_link_dv    = p->dc_link_dv;
    in->inv.rpm           = p->rpm;
    in->inv.temp_inverter = p->temp_inv;
    in->inv.temp_motor    = p->temp_motor;
    in->inv.fault         = p->inv_fault;
}
