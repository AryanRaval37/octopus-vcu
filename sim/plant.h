/* plant.h — a pretend car, just real enough to argue back.
 *
 * This is not a model of your vehicle and it is not trying to be. It exists
 * so the VCU gets consequences: close the precharge relay and the DC link
 * actually climbs, ask for torque and the rpm actually rises, hold full
 * torque and things actually get hot.
 *
 * Without that, a state machine test is just poking a struct and agreeing
 * with yourself.
 */
#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include "core/types.h"

typedef struct {
    // driver and harness, set by the scenario
    mv_t apps1_mv, apps2_mv;
    mv_t brake1_mv, brake2_mv;
    bool ts_request, rtd_button;
    bool sdc_ok;
    direction_t dir;

    /* The CAN partners, and the three ways a scenario can break each of
     * them — matching the three failures T11.9.2.d asks you to survive.
     * `alive` stops the frames, `counter_stuck` keeps sending the same one,
     * `crc_bad` corrupts them. */
    bool bms_alive, inv_alive;
    bool bms_fault, inv_fault;
    bool bms_counter_stuck, inv_counter_stuck;
    bool bms_crc_bad, inv_crc_bad;
    uint8_t bms_counter, inv_counter;

    // physical state
    dv_t      pack_dv;
    dv_t      dc_link_dv;
    pct_x10_t soc;
    mv_t      cell_min_mv;
    rpm_t     rpm;
    degc_t    temp_motor, temp_inv;
    amp_x10_t discharge_limit;

    /* The three fields below are why this model works at all, and they are
     * the same lesson three times.
     *
     * At a 1 ms tick every per-tick increment here is a fraction, and
     * integer division throws fractions away. Integrate rpm directly and you
     * get zero forever. So each keeps a finer-grained accumulator and the
     * visible field above is just a view of it.
     *
     * rpm was caught early. The temperatures were not, and sat at exactly
     * 25 C for the life of the project — which meant the entire thermal
     * derate had never once executed. Micro-degrees, because milli-degrees
     * still truncated to zero at full torque. */
    int32_t rpm_x100;
    int32_t temp_motor_uc;     // micro-degrees C
    int32_t temp_inv_uc;

    // failure injection
    bool precharge_resistor_open;  // the link will never rise
    uint16_t precharge_tau_ms;     // RC time constant

    uint32_t now_ms;
} plant_t;

void plant_init(plant_t *p);

// Move the world forward by dt_ms, given what the VCU just asked for.
void plant_step(plant_t *p, const vcu_out_t *out, uint16_t dt_ms);

// Render the plant as the VCU's input struct.
void plant_to_input(const plant_t *p, vcu_in_t *in);

// Set a pedal from a percentage, honouring each channel's slope, so a
// scenario can say "pedal = 45 %" and get a consistent pair of millivolts.
void plant_set_pedal_pct(plant_t *p, int pct_x10);
void plant_set_brake_pct(plant_t *p, int pct_x10);

// Temperatures go through these so the accumulator stays in step with the
// value. Assigning p->temp_motor directly gets silently undone next tick.
void plant_set_temp_motor(plant_t *p, degc_t c);
void plant_set_temp_inv(plant_t *p, degc_t c);

// Same story for rpm: it has an accumulator behind it, so assigning p->rpm
// directly lasts exactly one tick.
void plant_set_rpm(plant_t *p, rpm_t r);

#endif /* SIM_PLANT_H */
