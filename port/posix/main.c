// port/posix/main.c -- the host port, and the only file that knows it isn't
// in a car.
//
// On the S32K344 this becomes: read the ADCs, drain the CAN RX queues, call
// vcu_step(), write the torque frame and the contactor pins. Here it reads
// the plant model instead. The middle line is identical in both, and that
// is the whole point of the layering.
//
// Run it to watch a drive:  ./vcu_demo

#include "sim/plant.h"
#include "vcu/vcu.h"

#include <stdio.h>

static void print_faults(fault_mask_t m)
{
    if (!m) { printf("-"); return; }
    for (int i = 0; i < FAULT_COUNT; i++)
        if (m & FAULT_BIT(i)) printf("%s ", fault_name((fault_id_t)i));
}

int main(void)
{
    plant_t   plant;  plant_init(&plant);
    vcu_t     vcu;    vcu_init(&vcu, vcu_cfg_default());
    vcu_in_t  in;
    vcu_out_t out = { 0 };

    const uint16_t TICK_MS = 1;
    vcu_state_t last = VCU_STATE_COUNT;

    printf("  time  state       pedal  torque  rpm   dclink  faults\n");
    printf("  ----  ----------  -----  ------  ----  ------  ------\n");

    for (uint32_t t = 0; t <= 8000; t += TICK_MS) {
        // A driver, of sorts. Each of these is one thing a person does, and
        // the interesting ones are the last two.
        if (t == 100)  plant.ts_request = true;                              // "wake up"
        if (t == 1500) { plant.rtd_button = true; plant_set_brake_pct(&plant, 400); }
        if (t == 3300) { plant.rtd_button = false; plant_set_brake_pct(&plant, 0); }
        if (t == 3400) plant_set_pedal_pct(&plant, 600);                     // go
        if (t == 5000) plant_set_pedal_pct(&plant, 0);                       // lift: regen
        if (t == 6000) plant.apps2_mv = 2000;                                // a sensor dies

        plant.now_ms = t;
        plant_to_input(&plant, &in);
        vcu_step(&vcu, &in, &out);
        plant_step(&plant, &out, TICK_MS);

        // Print on a state change, and twice a second otherwise, so the
        // transitions stand out instead of scrolling past.
        bool interesting = (out.state != last) || (t % 500 == 0);
        if (interesting) {
            printf("  %4u  %-10s  %5u  %6d  %4u  %6u  ",
                   t, vcu_state_name(out.state), out.pedal,
                   out.torque_cmd, plant.rpm, plant.dc_link_dv / 10);
            print_faults(out.faults);
            printf("\n");
            last = out.state;
        }
    }
    return 0;
}
