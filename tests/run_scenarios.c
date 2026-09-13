// run_scenarios -- point it at .scn files, it tells you which ones broke.
//
//   ./run_scenarios tests/scenarios/*.scn
//   ./run_scenarios -v tests/scenarios/02_apps_disagreement.scn
//
// The -v flag prints a state trace every 100 ms, which is the fastest way
// to find out why a scenario you just wrote does not do what you expected.

#include "sim/scenario.h"

#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    bool verbose = false;
    int  first = 1;

    if (argc > 1 && strcmp(argv[1], "-v") == 0) { verbose = true; first = 2; }
    if (first >= argc) {
        fprintf(stderr, "usage: %s [-v] <scenario.scn> ...\n", argv[0]);
        return 2;
    }

    int files = 0, failed = 0, checks = 0;

    for (int i = first; i < argc; i++) {
        scenario_result_t r;
        files++;
        if (verbose) printf("== %s\n", argv[i]);
        bool ok = scenario_run_file(argv[i], 1, verbose, &r);
        checks += r.checks;
        if (ok) {
            printf("PASS  %-44s %3d checks\n", argv[i], r.checks);
        } else {
            failed++;
            printf("FAIL  %-44s %3d checks, %d failed\n", argv[i], r.checks, r.failures);
            printf("      %s\n", r.first_failure);
        }
    }

    printf("\n%d file%s, %d checks, %d failed\n",
           files, files == 1 ? "" : "s", checks, failed);
    return failed ? 1 : 0;
}
