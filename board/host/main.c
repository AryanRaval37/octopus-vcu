// board/host/main.c -- the "board" that is your own machine.
//
// Embedded projects call your computer the HOST and the thing you are
// building for the TARGET. This is the host build: same controller, but the
// world it senses and acts on is the simulator rather than silicon.
//
// A board has exactly three jobs, and you can see all three below:
//
//   1. fill  vcu_in_t   from whatever this hardware has
//   2. call  vcu_step()
//   3. apply vcu_out_t   to whatever this hardware drives, and drain the log
//
// On the S32K344 step 1 becomes ADC reads and CAN RX, and step 3 becomes
// GPIO writes and a CAN TX frame. Here they are the simulator instead. The
// middle line is identical in both, and that is the entire point of the
// split between core/ and board/.
//
//   ./vcu_demo             watch a drive
//   ./vcu_demo --csv FILE  and write the datalog out

#include "core/vcu.h"
#include "sim/plant.h"

#include <stdio.h>
#include <string.h>

static void print_faults(fault_mask_t m)
{
    if (!m) { printf("-"); return; }
    for (int i = 0; i < FAULT_COUNT; i++)
        if (m & FAULT_BIT(i)) printf("%s ", fault_name((fault_id_t)i));
}

// Step 3b: drain the recorder. On the target this is a UART or an SD card
// and it runs in a low-priority task; the ring buffer is what lets the
// control loop not care how slow it is.
static void drain_log(vcu_t *vcu, FILE *csv, unsigned *rows)
{
    log_rec_t r;
    char line[256];
    while (datalog_pop(&vcu->log, &r)) {
        (*rows)++;
        if (csv) {
            datalog_format_row(&r, line, sizeof line);
            fprintf(csv, "%s\n", line);
        }
    }
}

int main(int argc, char **argv)
{
    FILE *csv = NULL;
    if (argc >= 3 && strcmp(argv[1], "--csv") == 0) {
        csv = fopen(argv[2], "w");
        if (!csv) { fprintf(stderr, "cannot write %s\n", argv[2]); return 2; }
        fprintf(csv, "%s\n", datalog_csv_header());
    }

    plant_t   plant;  plant_init(&plant);
    vcu_t     vcu;    vcu_init(&vcu, vcu_cfg_default());
    vcu_in_t  in;
    vcu_out_t out = { 0 };

    const uint16_t TICK_MS = 1;
    vcu_state_t last = VCU_STATE_COUNT;
    unsigned rows = 0;

    printf("  time  state       pedal  brake  torque  rpm   dclink  faults\n");
    printf("  ----  ----------  -----  -----  ------  ----  ------  ------\n");

    for (uint32_t t = 0; t <= 9000; t += TICK_MS) {
        // A driver, of sorts. Each line is one thing a person does, and the
        // last two are the ones worth watching.
        if (t == 100)  plant.ts_request = true;                              // "wake up"
        if (t == 1500) { plant.rtd_button = true; plant_set_brake_pct(&plant, 400); }
        if (t == 3300) { plant.rtd_button = false; plant_set_brake_pct(&plant, 0); }
        if (t == 3400) plant_set_pedal_pct(&plant, 600);                     // go
        if (t == 5000) plant_set_pedal_pct(&plant, 0);                       // lift: regen
        if (t == 6000) plant.apps2_mv = 2000;                                // a sensor drifts
        if (t == 7000) plant_set_pedal_pct(&plant, 0);                       // release: recover
        if (t == 8000) plant.sdc_ok = false;                                 // shutdown circuit opens

        plant.now_ms = t;
        plant_to_input(&plant, &in);      // 1. sense
        vcu_step(&vcu, &in, &out);        // 2. decide
        plant_step(&plant, &out, TICK_MS);// 3. act (on a pretend car)
        drain_log(&vcu, csv, &rows);

        // Print on a state change, and twice a second otherwise, so the
        // transitions stand out instead of scrolling past.
        if (out.state != last || t % 500 == 0) {
            printf("  %4u  %-10s  %5u  %5u  %6d  %4u  %6u  ",
                   t, vcu_state_name(out.state), out.pedal, out.brake,
                   out.torque_cmd, plant.rpm, plant.dc_link_dv / 10);
            print_faults(out.faults);
            printf("\n");
            last = out.state;
        }
    }

    printf("\n  datalog: %u rows, %u dropped\n", rows, vcu.log.dropped);
    if (csv) { fclose(csv); printf("  written to %s\n", argv[2]); }
    return 0;
}
