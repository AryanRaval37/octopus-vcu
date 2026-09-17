/* plant.c — the pretend car.
 *
 * Crude on purpose. Every number in here is a guess that produces roughly
 * the right shape, and none came off a datasheet. The point is not accuracy,
 * it is that the VCU cannot tell it is being lied to.
 */
#include "sim/plant.h"
#include "core/config.h"

#define AMBIENT_C   25
#define UC_PER_C    1000000

void plant_init(plant_t *p)
{
    const plant_t zero = { 0 };
    *p = zero;
    p->pack_dv     = 4000;     // 400.0 V
    p->dc_link_dv  = 0;
    p->soc         = 800;      // 80.0 %
    p->cell_min_mv = 3800;
    p->bms_alive   = true;
    p->inv_alive   = true;
    p->sdc_ok      = true;
    p->dir         = DIR_FORWARD;
    p->discharge_limit = 2000; // 200.0 A -> 80 kW at 400 V
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
    // matches the default calibration: ch1 rises, ch2 falls
    p->apps1_mv = (mv_t)(500 + (4000 * pct_x10) / PCT_MAX);
    p->apps2_mv = (mv_t)(4500 - (4000 * pct_x10) / PCT_MAX);
}

void plant_set_brake_pct(plant_t *p, int pct_x10)
{
    if (pct_x10 < 0) pct_x10 = 0;
    if (pct_x10 > PCT_MAX) pct_x10 = PCT_MAX;
    // both brake channels rise together
    p->brake1_mv = (mv_t)(500 + (4000 * pct_x10) / PCT_MAX);
    p->brake2_mv = (mv_t)(500 + (4000 * pct_x10) / PCT_MAX);
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

void plant_step(plant_t *p, const vcu_out_t *out, uint16_t dt_ms)
{
    p->now_ms += dt_ms;

    // The CAN partners tick their rolling counters, unless a scenario has
    // jammed them.
    if (p->bms_alive && !p->bms_counter_stuck) p->bms_counter++;
    if (p->inv_alive && !p->inv_counter_stuck) p->inv_counter++;

    // DC link: climbs toward the pack while precharging, pinned once AIR+
    // is in, bleeding away otherwise.
    if (out->precharge_relay && !p->precharge_resistor_open) {
        int32_t gap = (int32_t)p->pack_dv - (int32_t)p->dc_link_dv;
        int32_t d   = (gap * dt_ms) / (p->precharge_tau_ms ? p->precharge_tau_ms : 1);
        if (d < 1 && gap > 0) d = 1;
        p->dc_link_dv = (dv_t)(p->dc_link_dv + d);
    } else if (out->air_pos) {
        p->dc_link_dv = p->pack_dv;
    } else if (!out->precharge_relay) {
        int32_t d = ((int32_t)p->dc_link_dv * dt_ms) / 2000;
        p->dc_link_dv = (dv_t)(p->dc_link_dv > d ? p->dc_link_dv - d : 0);
    }

    // rpm: torque speeds it up, drag slows it down.
    p->rpm_x100 += ((int32_t)out->torque_cmd * dt_ms) / 24;
    p->rpm_x100 -= (p->rpm_x100 * (int32_t)dt_ms) / 8000;
    if (p->rpm_x100 < 0)       p->rpm_x100 = 0;
    if (p->rpm_x100 > 2000000) p->rpm_x100 = 2000000;
    p->rpm = (rpm_t)(p->rpm_x100 / 100);

    /* Heat in with torque, out toward ambient with a ~60 s time constant.
     *
     * The rates are picked so that torque alone is enough to cook this
     * thing: left unchecked the motor heads for about 130 C, well past its
     * 110 C cut-off. A model that can never overheat cannot test an
     * overheat.
     *
     * In practice it never gets there, and watching that happen is the fun
     * part. Around 90 C the derate starts pulling torque back, less torque
     * makes less heat, and the loop settles. Hold the pedal flat and it
     * parks at about 94 C; ease off and it parks somewhere cooler. Nobody
     * wrote either number anywhere. It is just where the curves meet. */
    int32_t heat = (out->torque_cmd < 0 ? -out->torque_cmd : out->torque_cmd);

    p->temp_motor_uc += (heat * 7 * dt_ms) / 8;
    p->temp_motor_uc -= ((p->temp_motor_uc - AMBIENT_C * UC_PER_C) * dt_ms) / 60000;
    p->temp_motor = (degc_t)(p->temp_motor_uc / UC_PER_C);

    p->temp_inv_uc += (heat * 5 * dt_ms) / 8;
    p->temp_inv_uc -= ((p->temp_inv_uc - AMBIENT_C * UC_PER_C) * dt_ms) / 60000;
    p->temp_inv = (degc_t)(p->temp_inv_uc / UC_PER_C);
}

void plant_to_input(const plant_t *p, vcu_in_t *in)
{
    const vcu_in_t zero = { 0 };
    *in = zero;

    in->now_ms      = p->now_ms;
    in->apps1_mv    = p->apps1_mv;
    in->apps2_mv    = p->apps2_mv;
    in->brake1_mv   = p->brake1_mv;
    in->brake2_mv   = p->brake2_mv;
    in->ts_request  = p->ts_request;
    in->rtd_button  = p->rtd_button;
    in->shutdown_ok = p->sdc_ok;
    in->dir_request = p->dir;

    // A dead bus means no frame arrived. Note the payload is still copied:
    // the values are sitting in the receive buffer where they were left, and
    // the whole point of the health tracking is that the VCU must notice
    // they are old rather than reading them and being reassured.
    in->bms.rx              = p->bms_alive;
    in->bms.counter         = p->bms_counter;
    in->bms.crc_ok          = !p->bms_crc_bad;
    in->bms.soc             = p->soc;
    in->bms.pack_dv         = p->pack_dv;
    in->bms.cell_min_mv     = p->cell_min_mv;
    in->bms.discharge_limit = p->discharge_limit;
    in->bms.charge_limit    = 500;
    in->bms.temp_max        = 35;
    in->bms.fault           = p->bms_fault;

    in->inv.rx            = p->inv_alive;
    in->inv.counter       = p->inv_counter;
    in->inv.crc_ok        = !p->inv_crc_bad;
    in->inv.dc_link_dv    = p->dc_link_dv;
    in->inv.rpm           = p->rpm;
    in->inv.temp_inverter = p->temp_inv;
    in->inv.temp_motor    = p->temp_motor;
    in->inv.fault         = p->inv_fault;
}
