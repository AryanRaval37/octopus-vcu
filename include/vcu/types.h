// ! REVIEWED - still work in progress
// ! Decide on vcu_in_t struct and what it should contain.


/* types.h — Dictionary for VCU
 *
 * Nothing here knows about hardware. That is what lets the same code run as debug on a 
 * a laptop and on the S32K344.
 *
 * Next: src/svc/apps.c.
 */

// TODO: Design choice,
// Either vcu_in_t should be renamed to car state or something which is a massive struct being passed around to every function that needs part of it.
// Sounds not optimal because too much redundancy but atleast there is one car state everywhere and not split into pieces.
// (basically equivalent to global variables)
// Second option is make smaller structs
// apps, bms, inverter, vcu_in etc and have separate fields in them (Possibly overlapping) being passed around to functions that need them.
// right now vcu_in_t is also somewhere in the middle, not having all the info but also being passed around to functions like apps_step in apps.c


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

// Rememeber to write HV voltage in decivolts, won't fit in millivolts max size.
typedef int16_t  nm_x10_t;    // torque,  0.1 Nm  -> 1234 = 123.4 Nm
typedef uint16_t mv_t;        // millivolts -- LV and single cells
typedef uint16_t dv_t;        // decivolts  -- HV; 4000 = 400.0 V
typedef int16_t  amp_x10_t;   // 0.1 A, signed, + is discharge
typedef uint16_t pct_x10_t;   // generic perentage type, least count: 0.1 %  1000 meaning 100.0 %
typedef int16_t  degc_t;      // whole degrees C
typedef uint16_t rpm_t;

#define PCT_MAX ((pct_x10_t)1000)

// ---------------------------------------------------------------- states
//
//   INIT -> LV_READY -> PRECHARGE -> TS_ACTIVE -> RTD_WAIT -> DRIVE
//                           |            |           |         |
//                           +------------+-----------+---------+--> FAULT
//
// there is no 'default' state and there shouldn't be
typedef enum {
    VCU_INIT = 0,     // power-on self test
    VCU_LV_READY,     // 12 V up, nothing broken, waiting to be asked
    VCU_PRECHARGE,    // filling the DC link through the resistor
    VCU_TS_ACTIVE,    // HV live, AIRs closed, still not drivable
    VCU_RTD_WAIT,     // brake + button + noise : rethink use of this maybe
    VCU_DRIVE,        // torque commands are real now
    VCU_FAULT,        // something broke
    VCU_SHUTDOWN,     // orderly power-down
    VCU_STATE_COUNT
} vcu_state_t;

const char *vcu_state_name(vcu_state_t s); // Recheck: ??

// ---------------------------------------------------------------- faults
// A fault's ID is its bit in the active mask, so this order has to match
// the table in fault.c. It drifted once. Now the table checks itself.
typedef enum {
    FAULT_SEV_INFO = 0,  // log it (probably will use just as a quick debug thing)
    FAULT_SEV_WARN,      // log it and tell the pit (rethink use of this maybe)
    
    // Note : Literally no clue about the differences between these rn, figure out later, remove if unused.
    // probably will end up using critical for all of these and call it a day.
    FAULT_SEV_DERATE,    // fade the torque down
    FAULT_SEV_LIMP,      // hard cap
    FAULT_SEV_CRITICAL   // zero torque, open the contactors, go to FAULT
} fault_sev_t;


// basically an enum of whatever things I think can be faulty
// Recheck : There is some bitmask thing for faults look into it once
typedef enum {
    FAULT_APPS_IMPLAUSIBLE = 0, // T.4: channels disagree >10% for >100 ms
    FAULT_APPS_RANGE,           // a channel is open or shorted
    FAULT_BPPC,                 // brake pressed while throttle >25%
    FAULT_BRAKE_RANGE,
    FAULT_PRECHARGE_TIMEOUT,
    FAULT_PRECHARGE_NO_RISE,    // link isn't climbing: resistor open?
    FAULT_BMS_TIMEOUT,
    FAULT_BMS_FAULT,            // the BMS says it's unhappy, coordiante with manish about this
    FAULT_INVERTER_TIMEOUT,
    FAULT_INVERTER_FAULT,
    FAULT_CELL_UNDERVOLT,
    FAULT_OVERTEMP_MOTOR,
    FAULT_OVERTEMP_INVERTER,
    FAULT_OVERSPEED,            // if this is actually a ever a thing then damn we're good
    FAULT_CAN_BUSOFF,
    FAULT_COUNT
} fault_id_t;

_Static_assert(FAULT_COUNT <= 32, "active-fault mask is a uint32_t");

typedef uint32_t fault_mask_t;
#define FAULT_BIT(id) ((fault_mask_t)1u << (id))

// ----------------------------------------------------------- drive modes
// Recheck: umm... do we even have these driving modes - probably should remove
typedef enum { DRIVE_MODE_ECO = 0, DRIVE_MODE_SPORT, DRIVE_MODE_COUNT } drive_mode_t;
typedef enum { DIR_NEUTRAL = 0, DIR_FORWARD, DIR_REVERSE } direction_t;

/* ---------------------------------------------------------------- inputs
 * Basically the design is that vcu step is a function which does literally everything critical
 * It won't derive any time, global variables, things passed around or modified by other functions.
 * It will be explicitly given a strut set of imputs that it needs and it will provide the outputs.
 * Easier to test and track what variables are being passed to it.
 * 
 * According to the great Claude Overlord...
 * This struct is the whole world. If it isn't in here the VCU cannot know
 * it — not a pin, not a CAN frame, not even what time it is.
 *
 * That sounds limiting and it is the best thing in the repo. A controller
 * that cannot reach out to the world is one you can drop into any world you
 * like, including one where the pedal sensor dies at exactly t=3500 ms,
 * forty times, in a second.
 *
 * now_ms is in here with everything else on purpose. Time is handed in, so
 * a scheduler that runs long cannot quietly shorten the 100 ms window.
 */


// todo: Work in progress here
// this is still stock code untouched so far.
// can only change once I have a better idea of where this affects rest of codebase
typedef struct {
    uint32_t now_ms;

    // Recheck: why is this meant to be raw? and not percentage?
    // shouldn't they be already convereted as they are being read and this shouldn't be the job of the vcu
    // vcu should just do the decision making and not sit calling functions to do these conversions.
    // raw analogue, straight off the ADC
    mv_t apps1_mv;
    mv_t apps2_mv;
    mv_t brake_mv;

    bool ts_request;      // "wake up the tractive system" button
    bool rtd_button;      // "ready to drive" button
    bool shutdown_ok;     // the hardware shutdown loop is closed
    drive_mode_t mode;
    direction_t  dir_request;

    // Decoded CAN. `valid` means "fresh", and it is not decoration: a stale
    // battery reading that still looks sensible is worse than a missing
    // one, so everything on the bus carries a timeout.
    struct {
        bool      valid;
        pct_x10_t soc;
        dv_t      pack_dv;
        mv_t      cell_min_mv;
        amp_x10_t discharge_limit;
        amp_x10_t charge_limit;   // regen ceiling 
        degc_t    temp_max;
        bool      fault;
    } bms;

    struct {
        bool     valid;
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
// Everything it can do about all that: seven things it can move, and three
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
    pct_x10_t    pedal;            // arbitrated pedal, for telemetry
} vcu_out_t;

#endif /* VCU_TYPES_H */
