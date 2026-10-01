// board/host/main.c - runs the VCU against the simulated car on a laptop.
//
// Every board does the same three things each tick:
//   1. fill vcu_in_t from whatever it has (here: the plant model)
//   2. vcu_step()
//   3. apply vcu_out_t, and drain the log
// On the S32K344, 1 becomes ADC reads and CAN RX and 3 becomes GPIO writes
// and CAN TX. Step 2 is the same code.
//
//   ./vcu_demo              print a short drive
//   ./vcu_demo --csv FILE   and write the datalog to FILE

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

// On the target this runs in a low-priority task writing to SD or UART.
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
    vcu_t     vcu;    vcu_init(&vcu, NULL);
    vcu_in_t  in;
    vcu_out_t out = { 0 };

    const uint16_t TICK_MS = 1;
    vcu_state_t last = VCU_STATE_COUNT;
    unsigned rows = 0;

    printf("  time  state       pedal  brake  torque  rpm   dclink  faults\n");
    printf("  ----  ----------  -----  -----  ------  ----  ------  ------\n");

    for (uint32_t t = 0; t <= 9000; t += TICK_MS) {
        // The "driver".
        if (t == 100)  plant.ts_request = true;
        if (t == 1500) { plant_set_brake_pct(&plant, 400); plant.rtd_button = true; }
        if (t == 3300) { plant.rtd_button = false; plant_set_brake_pct(&plant, 0); }
        if (t == 3400) plant_set_pedal_pct(&plant, 600);
        if (t == 5000) plant_set_pedal_pct(&plant, 0);     // lift, regen
        if (t == 6000) plant.apps2_duty = 400;             // APPS2 drifts
        if (t == 7000) plant_set_pedal_pct(&plant, 0);     // sensor back, latch clears
        if (t == 8000) plant.sdc_ok = false;               // shutdown button

        plant.now_ms = t;
        plant_to_input(&plant, &in);
        vcu_step(&vcu, &in, &out);
        plant_step(&plant, &out, TICK_MS);
        drain_log(&vcu, csv, &rows);

        // Every state change, plus every 500 ms.
        if (out.state != last || t % 500 == 0) {
            printf("  %4u  %-10s  %5u  %5u  %6d  %4u  %6u  ",
                   (unsigned)t, vcu_state_name(out.state), out.pedal, out.brake,
                   out.torque_cmd, plant.rpm, plant.dc_link_dv / 10);
            print_faults(out.faults);
            printf("\n");
            last = out.state;
        }
    }

    printf("\n  datalog: %u rows, %lu dropped\n", rows, (unsigned long)vcu.log.dropped);
    if (csv) { fclose(csv); printf("  written to %s\n", argv[2]); }
    return 0;
}
