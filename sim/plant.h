// plant.h - a rough model of the car, so the VCU has something to react to.
//
// Nothing here comes from a datasheet. It only has to behave the right way
// round: the DC link charges through the precharge relay, torque spins the
// motor up, sustained torque heats things. Enough that a scenario can check
// what the VCU does about it.

#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include "core/types.h"

typedef struct {
    // Driver inputs and harness, set by the scenario.
    mv_t      apps1_mv;
    pct_x10_t apps2_duty;      // PWM channel, 0.1 %
    mv_t      brake1_mv, brake2_mv;
    bool      ts_request, rtd_button;
    bool      sdc_ok;          // every switch in the loop closed, ignoring the VCU's own
    direction_t dir;

    // CAN partners. Each one sends a frame every *_period_ms. A scenario can
    // kill it (alive), freeze its rolling counter (stuck), or corrupt the
    // checksum (crc_bad), which are the three T11.9.2.d failures.
    bool     bms_alive, inv_alive;
    bool     bms_fault, inv_fault;
    bool     bms_counter_stuck, inv_counter_stuck;
    bool     bms_crc_bad, inv_crc_bad;
    uint8_t  bms_counter, inv_counter;
    uint16_t bms_period_ms, inv_period_ms;

    // Physical state.
    dv_t      pack_dv;
    dv_t      dc_link_dv;
    pct_x10_t soc;
    mv_t      cell_min_mv;
    rpm_t     rpm;
    degc_t    temp_motor, temp_inv;
    amp_x10_t discharge_limit;
    amp_x10_t charge_limit;

    // At a 1 ms tick the per-tick change in rpm and temperature is a
    // fraction, and integer maths rounds it to zero. These hold the value at
    // finer resolution; rpm/temp_* above are just views of them. Set them
    // through the plant_set_* helpers or the change is undone next tick.
    int32_t rpm_x100;
    int32_t temp_motor_uc;     // micro-degrees C
    int32_t temp_inv_uc;

    // Whether the VCU's own switch in the shutdown loop was open last tick.
    // The sense line is modelled downstream of it, so when the VCU opens the
    // loop it also sees the loop open.
    bool vcu_opened_sdc;

    bool     precharge_resistor_open;
    uint16_t precharge_tau_ms;

    uint32_t now_ms;
} plant_t;

void plant_init(plant_t *p);

// Advance the model by dt_ms given what the VCU commanded.
void plant_step(plant_t *p, const vcu_out_t *out, uint16_t dt_ms);

// Fill the VCU input for the current time. Not const: sending a CAN frame
// bumps that device's rolling counter.
void plant_to_input(plant_t *p, vcu_in_t *in);

// What the VCU sees on its SDC sense input.
bool plant_sdc_sensed(const plant_t *p);

// Pedal and brake from a percentage (x10), respecting each channel's slope
// in the default calibration.
void plant_set_pedal_pct(plant_t *p, int pct_x10);
void plant_set_brake_pct(plant_t *p, int pct_x10);

void plant_set_temp_motor(plant_t *p, degc_t c);
void plant_set_temp_inv(plant_t *p, degc_t c);
void plant_set_rpm(plant_t *p, rpm_t r);

#endif // SIM_PLANT_H
