/* scenario.h — vehicle tests that read like a description of a drive.
 *
 * A scenario is a timeline: at t=N set this, at t=N expect that. The runner
 * ticks the plant and the VCU at a fixed rate and fires events as their
 * moment arrives.
 *
 *     t=3000  pedal=450
 *     t=3500  apps2_mv=2000          # channel 2 drifts off
 *     t=3610  expect_torque_max=0    # must be dead within 100 ms
 *     t=3610  expect_fault=APPS_IMPLAUSIBLE
 *
 * Keep every scenario you ever write, including the boring ones. The corpus
 * ends up worth more than the code — it is what lets you change the torque
 * map on a Friday afternoon and still know the shutdown path works.
 */
#ifndef SIM_SCENARIO_H
#define SIM_SCENARIO_H

#include <stdbool.h>

typedef struct {
    int  checks;
    int  failures;
    char first_failure[256];
} scenario_result_t;

// tick_ms is the simulated control period. 1 ms matches the real safety task.
bool scenario_run_file(const char *path, unsigned tick_ms,
                       bool verbose, scenario_result_t *res);

#endif /* SIM_SCENARIO_H */
