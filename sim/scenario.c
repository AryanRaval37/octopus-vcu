/* scenario.c — the thing that reads a .scn file and drives the pretend car.
 *
 * Parse every line into a flat list of timestamped events, sort it, then
 * tick the clock from zero and fire each event as its moment arrives. That
 * is the whole runner. Keeping events flat and sorted rather than nesting
 * them under timestamps is what lets a scenario be written out of order and
 * still mean the obvious thing.
 */
#include "sim/scenario.h"
#include "sim/plant.h"
#include "core/vcu.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_EVENTS 512
#define MAX_LINE   512

typedef struct {
    uint32_t t_ms;
    char     key[48];
    char     val[64];
    int      line;
} event_t;

static int parse_state(const char *s)
{
    for (int i = 0; i < VCU_STATE_COUNT; i++)
        if (strcmp(s, vcu_state_name((vcu_state_t)i)) == 0) return i;
    return -1;
}

static int parse_fault(const char *s)
{
    for (int i = 0; i < FAULT_COUNT; i++)
        if (strcmp(s, fault_name((fault_id_t)i)) == 0) return i;
    return -1;
}

static int cmp_event(const void *a, const void *b)
{
    const event_t *x = a, *y = b;
    if (x->t_ms != y->t_ms) return (x->t_ms < y->t_ms) ? -1 : 1;
    return x->line - y->line;   /* stable within a timestamp */
}

static void fail(scenario_result_t *res, const char *path, const event_t *e,
                 const char *fmt, ...)
{
    res->failures++;
    if (res->first_failure[0]) return;

    char detail[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(detail, sizeof detail, fmt, ap);
    va_end(ap);

    snprintf(res->first_failure, sizeof res->first_failure,
             "%s:%d  t=%u  %s", path, e->line, e->t_ms, detail);
}

/* Apply one event. Returns false on an unknown key, and the caller turns
 * that into a failure.
 *
 * Being strict here matters more than it looks. A typo in an expectation —
 * expect_torqe_max=0 — would otherwise be a line that runs, checks nothing,
 * and passes forever. A test that cannot fail is worse than no test,
 * because you believe it. */
static bool apply(const event_t *e, plant_t *p, const vcu_out_t *out,
                  const vcu_t *vcu, scenario_result_t *res, const char *path)
{
    const char *k = e->key;
    const char *v = e->val;
    long n = strtol(v, NULL, 0);

    /* ---- inputs ---- */
    if      (!strcmp(k, "pedal"))      plant_set_pedal_pct(p, (int)n);
    else if (!strcmp(k, "brake"))      plant_set_brake_pct(p, (int)n);
    else if (!strcmp(k, "apps1_mv"))   p->apps1_mv = (mv_t)n;
    else if (!strcmp(k, "apps2_mv"))   p->apps2_mv = (mv_t)n;
    else if (!strcmp(k, "brake1_mv"))  p->brake1_mv = (mv_t)n;
    else if (!strcmp(k, "brake2_mv"))  p->brake2_mv = (mv_t)n;
    else if (!strcmp(k, "ts"))         p->ts_request = n != 0;
    else if (!strcmp(k, "rtd"))        p->rtd_button = n != 0;
    else if (!strcmp(k, "sdc"))        p->sdc_ok = n != 0;
    else if (!strcmp(k, "dir"))        p->dir  = (direction_t)n;
    else if (!strcmp(k, "bms_alive"))  p->bms_alive = n != 0;
    else if (!strcmp(k, "inv_alive"))  p->inv_alive = n != 0;
    else if (!strcmp(k, "bms_fault"))  p->bms_fault = n != 0;
    else if (!strcmp(k, "inv_fault"))  p->inv_fault = n != 0;
    else if (!strcmp(k, "bms_stuck"))  p->bms_counter_stuck = n != 0;
    else if (!strcmp(k, "inv_stuck"))  p->inv_counter_stuck = n != 0;
    else if (!strcmp(k, "bms_crc_bad")) p->bms_crc_bad = n != 0;
    else if (!strcmp(k, "inv_crc_bad")) p->inv_crc_bad = n != 0;
    else if (!strcmp(k, "soc"))        p->soc = (pct_x10_t)n;
    else if (!strcmp(k, "cell_mv"))    p->cell_min_mv = (mv_t)n;
    else if (!strcmp(k, "amp_limit"))  p->discharge_limit = (amp_x10_t)n;
    else if (!strcmp(k, "rpm"))        plant_set_rpm(p, (rpm_t)n);
    else if (!strcmp(k, "temp_motor")) plant_set_temp_motor(p, (degc_t)n);
    else if (!strcmp(k, "temp_inv"))   plant_set_temp_inv(p, (degc_t)n);
    else if (!strcmp(k, "pc_open"))    p->precharge_resistor_open = n != 0;

    /* ---- expectations ---- */
    else if (!strcmp(k, "expect_state")) {
        res->checks++;
        int want = parse_state(v);
        if (want < 0) { fail(res, path, e, "unknown state '%s'", v); return true; }
        if (out->state != (vcu_state_t)want)
            fail(res, path, e, "state: want %s, got %s", v, vcu_state_name(out->state));
    }
    else if (!strcmp(k, "expect_torque_max")) {
        res->checks++;
        if (out->torque_cmd > (nm_x10_t)n)
            fail(res, path, e, "torque %d > max %ld", out->torque_cmd, n);
    }
    else if (!strcmp(k, "expect_torque_min")) {
        res->checks++;
        if (out->torque_cmd < (nm_x10_t)n)
            fail(res, path, e, "torque %d < min %ld", out->torque_cmd, n);
    }
    else if (!strcmp(k, "expect_fault") || !strcmp(k, "expect_no_fault")) {
        res->checks++;
        int id = parse_fault(v);
        if (id < 0) { fail(res, path, e, "unknown fault '%s'", v); return true; }
        bool set  = (out->faults & FAULT_BIT(id)) != 0;
        bool want = !strcmp(k, "expect_fault");
        if (set != want)
            fail(res, path, e, "%s: want %s, got %s", v,
                 want ? "set" : "clear", set ? "set" : "clear");
    }
    else if (!strcmp(k, "expect_air_pos")) {
        res->checks++;
        if (out->air_pos != (n != 0))
            fail(res, path, e, "air_pos: want %ld, got %d", n, out->air_pos);
    }
    else if (!strcmp(k, "expect_enable")) {
        res->checks++;
        if (out->inverter_enable != (n != 0))
            fail(res, path, e, "inverter_enable: want %ld, got %d", n, out->inverter_enable);
    }
    else if (!strcmp(k, "expect_brake_min")) {
        res->checks++;
        if (out->brake < (pct_x10_t)n)
            fail(res, path, e, "brake %u < min %ld", out->brake, n);
    }
    else if (!strcmp(k, "expect_brake_max")) {
        res->checks++;
        if (out->brake > (pct_x10_t)n)
            fail(res, path, e, "brake %u > max %ld", out->brake, n);
    }
    else if (!strcmp(k, "expect_air_neg")) {
        res->checks++;
        if (out->air_neg != (n != 0))
            fail(res, path, e, "air_neg: want %ld, got %d", n, out->air_neg);
    }
    /* The log is part of the product, so it gets assertions too. A recorder
     * that quietly stops recording is worth finding here rather than after
     * the one run you needed it for. */
    else if (!strcmp(k, "expect_log_min")) {
        res->checks++;
        if (datalog_pending(&vcu->log) < (uint16_t)n)
            fail(res, path, e, "log has %u rows, want >= %ld",
                 datalog_pending(&vcu->log), n);
    }
    else if (!strcmp(k, "expect_log_dropped")) {
        res->checks++;
        if (vcu->log.dropped != (uint32_t)n)
            fail(res, path, e, "log dropped %u, want %ld", vcu->log.dropped, n);
    }
    else {
        return false;
    }
    return true;
}

bool scenario_run_file(const char *path, unsigned tick_ms,
                       bool verbose, scenario_result_t *res)
{
    const scenario_result_t zero = { 0 };
    *res = zero;

    FILE *fp = fopen(path, "r");
    if (!fp) { snprintf(res->first_failure, sizeof res->first_failure,
                        "cannot open %s", path); res->failures = 1; return false; }

    static event_t ev[MAX_EVENTS];
    int nev = 0, lineno = 0;
    char line[MAX_LINE];

    while (fgets(line, sizeof line, fp)) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';

        uint32_t t = 0;
        char *tok = strtok(line, " \t\r\n;");
        if (!tok) continue;
        if (sscanf(tok, "t=%u", &t) != 1) {
            fprintf(stderr, "%s:%d: line must start with t=<ms>\n", path, lineno);
            fclose(fp); res->failures++; return false;
        }
        while ((tok = strtok(NULL, " \t\r\n;")) != NULL) {
            char *eq = strchr(tok, '=');
            if (!eq) continue;
            *eq = '\0';
            if (nev >= MAX_EVENTS) { fprintf(stderr, "too many events\n"); break; }
            ev[nev].t_ms = t;
            ev[nev].line = lineno;
            snprintf(ev[nev].key, sizeof ev[nev].key, "%s", tok);
            snprintf(ev[nev].val, sizeof ev[nev].val, "%s", eq + 1);
            nev++;
        }
    }
    fclose(fp);

    qsort(ev, (size_t)nev, sizeof ev[0], cmp_event);

    plant_t plant;  plant_init(&plant);
    vcu_t   vcu;    vcu_init(&vcu, vcu_cfg_default());
    vcu_in_t  in;
    vcu_out_t out = { 0 };

    const uint32_t end_ms = nev ? ev[nev - 1].t_ms + tick_ms : 0;
    int next = 0;

    for (uint32_t t = 0; t <= end_ms; t += tick_ms) {
        /* Three phases per tick, in this order:
         *   1. apply every INPUT stamped at or before t
         *   2. run one control tick
         *   3. check every ASSERTION stamped at or before t
         *
         * So a line that sets something and checks something reads
         * left-to-right and means what it looks like, no matter which order
         * the two were written in. */
        const int first = next;

        for (int i = first; i < nev && ev[i].t_ms <= t; i++) {
            if (strncmp(ev[i].key, "expect_", 7) == 0) continue;
            if (!apply(&ev[i], &plant, &out, &vcu, res, path)) {
                fprintf(stderr, "%s:%d: unknown key '%s'\n",
                        path, ev[i].line, ev[i].key);
                res->failures++;
            }
        }

        plant.now_ms = t;
        plant_to_input(&plant, &in);
        vcu_step(&vcu, &in, &out);

        while (next < nev && ev[next].t_ms <= t) {
            if (strncmp(ev[next].key, "expect_", 7) == 0) {
                if (!apply(&ev[next], &plant, &out, &vcu, res, path)) {
                    fprintf(stderr, "%s:%d: unknown key '%s'\n",
                            path, ev[next].line, ev[next].key);
                    res->failures++;
                }
            }
            next++;
        }

        plant_step(&plant, &out, (uint16_t)tick_ms);

        if (verbose && (t % 100 == 0))
            printf("  t=%5u  %-10s pedal=%4u torque=%5d faults=0x%06x\n",
                   t, vcu_state_name(out.state), out.pedal,
                   out.torque_cmd, out.faults);
    }
    return res->failures == 0;
}
