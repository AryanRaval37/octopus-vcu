/* types.h
 * Rule citations throughout are Formula Bharat 2027 v1.2 (docs/rules/).
 * Read Next: core/sense/signals.c.
 */

#ifndef VCU_TYPES_H
#define VCU_TYPES_H

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------- units
// Fixed point, scale in the name. Read nm_x10_t as "newton-metres, x10".
//
// No floats in the control path. Not for speed — because a float can be
// NaN, and a NaN torque command is neither big nor small nor zero, so it
// gets past every clamp here without tripping one. An int16 cannot.

typedef int16_t  nm_x10_t;    // torque,  0.1 Nm  -> 1234 = 123.4 Nm
typedef uint16_t mv_t;        // millivolts -- LV rails and single cells
typedef uint16_t dv_t;        // decivolts  -- HV;  4000 = 400.0 V
typedef int16_t  amp_x10_t;   // 0.1 A, signed, + is discharge
typedef uint16_t pct_x10_t;   // 0.1 %    -> 1000 = 100.0 %
typedef int16_t  degc_t;      // whole degrees C
typedef uint16_t rpm_t;
typedef uint32_t watt_t;

// HV is in decivolts because 400 V is 400,000 mV and a uint16 stops at
// 65,535 — a full pack written in millivolts reads 6.1 V.
//
// ! Be careful: mv_t and dv_t are both uint16_t, so they are the SAME type and
//   he compiler will happily let you assign one to the other. Nothing here
//   catches a unit mix-up for you; read the field name before comparing two
//   voltages. (C can express a real distinction with a struct wrapper, at the
//   cost of .v on every use. Judged not worth it — see docs/IMPLEMENTATION-PLAN.md.)

#define PCT_MAX ((pct_x10_t)1000)

// ---------------------------------------------------------------- states
//
//   INIT -> LV_READY -> PRECHARGE -> TS_ACTIVE -> RTD_WAIT -> DRIVE
//                           |            |           |         |
//                           +------------+-----------+---------+--> FAULT
//

typedef enum {
    VCU_INIT = 0,     // power-on self test
    VCU_LV_READY,     // 12 V up, nothing broken, waiting to be asked
    VCU_PRECHARGE,    // filling the DC link through the resistor
    VCU_TS_ACTIVE,    // HV live, AIRs closed, still not drivable
    VCU_RTD_WAIT,     // brake + button + noise
    VCU_DRIVE,        // R2D. torque commands are real now
    VCU_FAULT,        // something broke
    VCU_SHUTDOWN,     // orderly power-down
    VCU_STATE_COUNT
} vcu_state_t;

const char *vcu_state_name(vcu_state_t s); // implmeneted in core/decide/state.c

/* ---------------------------------------------------------------- faults
 *   DERATE    reduce torque proportionally, stay in DRIVE
 *   LIMP      torque to zero, but HV stays up and the AIRs stay closed.
 *             This is exactly what T11.8.8 asks for on an APPS
 *             implausibility: "It is not necessary to completely
 *             deactivate the tractive system".
 *   CRITICAL  open the shutdown circuit and the AIRs. T11.9.5 makes this
 *             the required safe state for a failed System Critical Signal.
 */
typedef enum {
    FAULT_SEV_INFO = 0,  // log it
    FAULT_SEV_WARN,      // log it and tell the pit
    FAULT_SEV_DERATE,    // fade the torque down
    FAULT_SEV_LIMP,      // torque off, HV stays up
    FAULT_SEV_CRITICAL   // torque off, contactors open, go to FAULT
} fault_sev_t;

// A fault's ID is its bit in the active mask, so this order has to match the
// table in decide/faults.c. It drifted once. Now the table checks itself.
typedef enum {
    FAULT_APPS_IMPLAUSIBLE = 0, // T11.8.9: channels differ >10 pp for >100 ms
    FAULT_APPS_RANGE,           // T11.9.2: a channel is open or shorted
    FAULT_BRAKE_IMPLAUSIBLE,    // the two brake channels disagree
    FAULT_BRAKE_RANGE,          // T11.9.2, brake side
    FAULT_BPPC,                 // brake and throttle together -- see A6.4.4
    FAULT_PRECHARGE_TIMEOUT,
    FAULT_PRECHARGE_NO_RISE,    // link isn't climbing: resistor open?
    FAULT_BMS_TIMEOUT,          // T11.9.2.d: message lost or delayed
    FAULT_BMS_STALE,            // T11.9.2.d: counter stopped advancing
    FAULT_BMS_CORRUPT,          // T11.9.2.d: checksum failed
    FAULT_BMS_FAULT,            // the BMS says it is unhappy
    FAULT_INVERTER_TIMEOUT,
    FAULT_INVERTER_STALE,
    FAULT_INVERTER_CORRUPT,
    FAULT_INVERTER_FAULT,
    FAULT_CELL_UNDERVOLT,
    FAULT_OVERTEMP_MOTOR,
    FAULT_OVERTEMP_INVERTER,
    FAULT_OVERSPEED,
    FAULT_SDC_OPEN,             // EV4.11.8: shutdown circuit opened
    FAULT_CAN_BUSOFF,
    FAULT_COUNT
} fault_id_t;

_Static_assert(FAULT_COUNT <= 32, "active-fault mask is a uint32_t");

typedef uint32_t fault_mask_t;
#define FAULT_BIT(id) ((fault_mask_t)1u << (id))

// ------------------------------------------------------------- direction
// No reverse. EV2.2.4: "Wheels must not be spun in reverse." Neutral still implemented
typedef enum { DIR_NEUTRAL = 0, DIR_FORWARD } direction_t;

/* ---------------------------------------------------------------- inputs
 * This struct is the whole world. If it isn't in here the VCU cannot know
 * it — not a pin, not a CAN frame, not even what time it is.
 *
 * That sounds limiting and it is the best thing in the repo. A controller
 * that cannot reach out to the world is one you can drop into any world you
 * like.
 *
 * now_ms is in here with everything else on purpose. Time is handed in, so
 * a scheduler that runs long cannot quietly shorten the 100 ms window.
 */
typedef struct {
    uint32_t now_ms;

    // Pedals, raw. Two accelerator channels and two brake channels, because
    // one sensor cannot tell you it is lying (T11.8.5). They stay in
    // millivolts because deciding what a voltage *means* — in range? which
    // slope? do the two agree? — is the safety logic, and that belongs in
    // here rather than in a driver.
    mv_t apps1_mv;
    mv_t apps2_mv;
    mv_t brake1_mv;
    mv_t brake2_mv;

    bool ts_request;      // "wake up the tractive system" button
    bool rtd_button;      // "ready to drive" button
    bool shutdown_ok;     // the hardware shutdown circuit is closed
    direction_t dir_request;

    /* CAN arrives as events, not as conclusions.
     *
     * The board says "a frame turned up, here is its rolling counter and
     * whether the checksum passed". Deciding whether that adds up to a
     * signal you can steer a car by is this code's job, and T11.9.2.d
     * demands all three checks: corruption, loss, and delay.
     */
    struct {
        bool     rx;              // a frame arrived since the last tick
        uint8_t  counter;         // rolling counter from the frame
        bool     crc_ok;          // checksum verified by the decoder
        pct_x10_t soc;
        dv_t      pack_dv;
        mv_t      cell_min_mv;
        amp_x10_t discharge_limit;
        amp_x10_t charge_limit;   // regen ceiling
        degc_t    temp_max;
        bool      fault;
    } bms;

    struct {
        bool     rx;
        uint8_t  counter;
        bool     crc_ok;
        dv_t     dc_link_dv;
        rpm_t    rpm;
        degc_t   temp_inverter;
        degc_t   temp_motor;
        bool     fault;
        nm_x10_t torque_actual;
    } inv;

    bool can_busoff;
} vcu_in_t;

// --------------------------------------------------------------- outputs
// Everything it can do about all that: seven things it can move, and four
// fields of "here is what I was thinking" for the log.

typedef struct {
    nm_x10_t     torque_cmd;
    bool         inverter_enable;
    bool         air_neg;
    bool         air_pos;
    bool         precharge_relay;
    bool         rtd_buzzer;
    bool         shutdown_assert;  // pull the shutdown line low
    vcu_state_t  state;
    fault_mask_t faults;
    pct_x10_t    pedal;            // arbitrated accelerator, for telemetry
    pct_x10_t    brake;            // arbitrated brake
} vcu_out_t;

#endif /* VCU_TYPES_H */
