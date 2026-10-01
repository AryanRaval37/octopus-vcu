// scenario.h - timeline tests for the whole VCU.
//
//     t=3400  pedal=450
//     t=3500  apps2_duty=400                # APPS2 drifts to ~62 %
//     t=3620  expect_fault=APPS_IMPLAUSIBLE expect_torque_max=0
//
// The runner ticks the plant and the VCU together and fires each line when
// its time comes. Key list is in docs/testing.md.

#ifndef SIM_SCENARIO_H
#define SIM_SCENARIO_H

#include <stdbool.h>

typedef struct {
    int  checks;
    int  failures;
    char first_failure[256];
} scenario_result_t;

// tick_ms is the simulated control period; 1 ms matches the target.
bool scenario_run_file(const char *path, unsigned tick_ms,
                       bool verbose, scenario_result_t *res);

#endif // SIM_SCENARIO_H
