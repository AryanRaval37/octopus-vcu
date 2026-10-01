// scenario.c - reads a .scn file and runs it against the plant.
//
// Every key=value on every line becomes a timestamped event. The events are
// sorted, then the clock ticks from zero and each one fires when its time
// comes. Flat and sorted means a file can be written in any order and still
// do the obvious thing.
//
// The parser is strict on purpose. A typo like expect_torqe_max=0 or
// pedal=5O0 would otherwise be a line that checks nothing and passes
// forever.

#include "sim/scenario.h"
#include "sim/plant.h"
#include "core/vcu.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_EVENTS 1024
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

// Decimal only, and the whole string has to be a number. (strtol with base
// 0 would read "0450" as octal.)
static bool parse_num(const char *s, long *out)
{
    char *end;
    *out = strtol(s, &end, 10);
    return end != s && *end == '\0';
}

static int cmp_event(const void *a, const void *b)
{
    const event_t *x = a, *y = b;
    if (x->t_ms != y->t_ms) return (x->t_ms < y->t_ms) ? -1 : 1;
    return x->line - y->line;   // keep file order within one timestamp
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
             "%s:%d  t=%" PRIu32 "  %s", path, e->line, e->t_ms, detail);
}

static bool is_expect(const event_t *e)
{
    return strncmp(e->key, "expect_", 7) == 0;
}

static void check_bool(scenario_result_t *res, const char *path, const event_t *e,
                       const char *what, bool got, long want)
{
    res->checks++;
    if (got != (want != 0))
        fail(res, path, e, "%s: want %ld, got %d", what, want, got);
}

// Apply one event. Returns false if the key isn't one we know.
static bool apply(const event_t *e, plant_t *p, const vcu_out_t *out,
                  const vcu_t *vcu, scenario_result_t *res, const char *path)
{
    const char *k = e->key;
    const char *v = e->val;

    // These three take a name, everything else takes a number.
    if (!strcmp(k, "expect_state")) {
        res->checks++;
        int want = parse_state(v);
        if (want < 0) fail(res, path, e, "unknown state '%s'", v);
        else if (out->state != (vcu_state_t)want)
            fail(res, path, e, "state: want %s, got %s", v, vcu_state_name(out->state));
        return true;
    }
    if (!strcmp(k, "expect_fault") || !strcmp(k, "expect_no_fault")) {
        res->checks++;
        int id = parse_fault(v);
        if (id < 0) { fail(res, path, e, "unknown fault '%s'", v); return true; }
        bool set  = (out->faults & FAULT_BIT(id)) != 0;
        bool want = !strcmp(k, "expect_fault");
        if (set != want)
            fail(res, path, e, "%s: want %s, got %s", v,
                 want ? "set" : "clear", set ? "set" : "clear");
        return true;
    }

    long n;
    if (!parse_num(v, &n)) {
        fail(res, path, e, "%s: '%s' is not a number", k, v);
        return true;
    }

    // inputs
    if      (!strcmp(k, "pedal"))        plant_set_pedal_pct(p, (int)n);
    else if (!strcmp(k, "brake"))        plant_set_brake_pct(p, (int)n);
    else if (!strcmp(k, "apps1_mv"))     p->apps1_mv = (mv_t)n;
    else if (!strcmp(k, "apps2_duty"))   p->apps2_duty = (pct_x10_t)n;
    else if (!strcmp(k, "brake1_mv"))    p->brake1_mv = (mv_t)n;
    else if (!strcmp(k, "brake2_mv"))    p->brake2_mv = (mv_t)n;
    else if (!strcmp(k, "ts"))           p->ts_request = n != 0;
    else if (!strcmp(k, "rtd"))          p->rtd_button = n != 0;
    else if (!strcmp(k, "sdc"))          p->sdc_ok = n != 0;
    else if (!strcmp(k, "dir"))          p->dir = (direction_t)n;
    else if (!strcmp(k, "bms_alive"))    p->bms_alive = n != 0;
    else if (!strcmp(k, "inv_alive"))    p->inv_alive = n != 0;
    else if (!strcmp(k, "bms_fault"))    p->bms_fault = n != 0;
    else if (!strcmp(k, "inv_fault"))    p->inv_fault = n != 0;
    else if (!strcmp(k, "bms_stuck"))    p->bms_counter_stuck = n != 0;
    else if (!strcmp(k, "inv_stuck"))    p->inv_counter_stuck = n != 0;
    else if (!strcmp(k, "bms_crc_bad"))  p->bms_crc_bad = n != 0;
    else if (!strcmp(k, "inv_crc_bad"))  p->inv_crc_bad = n != 0;
    else if (!strcmp(k, "bms_period"))   p->bms_period_ms = (uint16_t)n;
    else if (!strcmp(k, "inv_period"))   p->inv_period_ms = (uint16_t)n;
    else if (!strcmp(k, "soc"))          p->soc = (pct_x10_t)n;
    else if (!strcmp(k, "cell_mv"))      p->cell_min_mv = (mv_t)n;
    else if (!strcmp(k, "amp_limit"))    p->discharge_limit = (amp_x10_t)n;
    else if (!strcmp(k, "charge_limit")) p->charge_limit = (amp_x10_t)n;
    else if (!strcmp(k, "rpm"))          plant_set_rpm(p, (rpm_t)n);
    else if (!strcmp(k, "temp_motor"))   plant_set_temp_motor(p, (degc_t)n);
    else if (!strcmp(k, "temp_inv"))     plant_set_temp_inv(p, (degc_t)n);
    else if (!strcmp(k, "pc_open"))      p->precharge_resistor_open = n != 0;

    // expectations
    else if (!strcmp(k, "expect_torque_max")) {
        res->checks++;
        if (out->torque_cmd > n)
            fail(res, path, e, "torque %d > max %ld", out->torque_cmd, n);
    }
    else if (!strcmp(k, "expect_torque_min")) {
        res->checks++;
        if (out->torque_cmd < n)
            fail(res, path, e, "torque %d < min %ld", out->torque_cmd, n);
    }
    else if (!strcmp(k, "expect_brake_min")) {
        res->checks++;
        if (out->brake < n)
            fail(res, path, e, "brake %u < min %ld", out->brake, n);
    }
    else if (!strcmp(k, "expect_brake_max")) {
        res->checks++;
        if (out->brake > n)
            fail(res, path, e, "brake %u > max %ld", out->brake, n);
    }
    else if (!strcmp(k, "expect_air_pos"))     check_bool(res, path, e, "air_pos", out->air_pos, n);
    else if (!strcmp(k, "expect_air_neg"))     check_bool(res, path, e, "air_neg", out->air_neg, n);
    else if (!strcmp(k, "expect_enable"))      check_bool(res, path, e, "inverter_enable", out->inverter_enable, n);
    else if (!strcmp(k, "expect_buzzer"))      check_bool(res, path, e, "rtd_buzzer", out->rtd_buzzer, n);
    else if (!strcmp(k, "expect_brake_light")) check_bool(res, path, e, "brake_light", out->brake_light, n);
    else if (!strcmp(k, "expect_sdc_assert"))  check_bool(res, path, e, "shutdown_assert", out->shutdown_assert, n);
    else if (!strcmp(k, "expect_log_min")) {
        res->checks++;
        if (datalog_pending(&vcu->log) < n)
            fail(res, path, e, "log has %u rows, want >= %ld", datalog_pending(&vcu->log), n);
    }
    else if (!strcmp(k, "expect_log_dropped")) {
        res->checks++;
        if (vcu->log.dropped != (uint32_t)n)
            fail(res, path, e, "log dropped %" PRIu32 ", want %ld", vcu->log.dropped, n);
    }
    else {
        return false;
    }
    return true;
}

static void apply_or_fail(const event_t *e, plant_t *p, const vcu_out_t *out,
                          const vcu_t *vcu, scenario_result_t *res, const char *path)
{
    if (!apply(e, p, out, vcu, res, path))
        fail(res, path, e, "unknown key '%s'", e->key);
}

// Parse the whole file into ev[]. Returns the event count, or -1 with
// res->first_failure filled in.
static int load(const char *path, event_t *ev, scenario_result_t *res)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        snprintf(res->first_failure, sizeof res->first_failure, "cannot open %s", path);
        return -1;
    }

    int nev = 0, lineno = 0;
    char line[MAX_LINE];
    const char *err = NULL;

    while (!err && fgets(line, sizeof line, fp)) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';

        char *tok = strtok(line, " \t\r\n;");
        if (!tok) continue;

        long t;
        if (strncmp(tok, "t=", 2) != 0 || !parse_num(tok + 2, &t) || t < 0) {
            err = "line must start with t=<ms>";
            break;
        }

        while ((tok = strtok(NULL, " \t\r\n;")) != NULL) {
            char *eq = strchr(tok, '=');
            if (!eq)               { err = "expected key=value"; break; }
            if (nev >= MAX_EVENTS) { err = "too many events, raise MAX_EVENTS"; break; }
            *eq = '\0';
            ev[nev].t_ms = (uint32_t)t;
            ev[nev].line = lineno;
            snprintf(ev[nev].key, sizeof ev[nev].key, "%s", tok);
            snprintf(ev[nev].val, sizeof ev[nev].val, "%s", eq + 1);
            nev++;
        }
    }
    fclose(fp);

    if (err) {
        snprintf(res->first_failure, sizeof res->first_failure, "%s:%d  %s", path, lineno, err);
        return -1;
    }
    return nev;
}

bool scenario_run_file(const char *path, unsigned tick_ms,
                       bool verbose, scenario_result_t *res)
{
    const scenario_result_t zero = { 0 };
    *res = zero;

    static event_t ev[MAX_EVENTS];
    const int nev = load(path, ev, res);
    if (nev < 0) { res->failures = 1; return false; }

    qsort(ev, (size_t)nev, sizeof ev[0], cmp_event);

    plant_t   plant;  plant_init(&plant);
    vcu_t     vcu;    vcu_init(&vcu, vcu_cfg_default());
    vcu_in_t  in;
    vcu_out_t out = { 0 };

    const uint32_t end_ms = nev ? ev[nev - 1].t_ms + tick_ms : 0;
    int next = 0;

    for (uint32_t t = 0; t <= end_ms; t += tick_ms) {
        // Inputs stamped at or before t go in before the tick, checks run
        // after it. So "t=100 ts=1 expect_state=PRECHARGE" means what it
        // looks like whichever order the two are written in.
        for (int i = next; i < nev && ev[i].t_ms <= t; i++)
            if (!is_expect(&ev[i])) apply_or_fail(&ev[i], &plant, &out, &vcu, res, path);

        plant.now_ms = t;
        plant_to_input(&plant, &in);
        vcu_step(&vcu, &in, &out);

        for (; next < nev && ev[next].t_ms <= t; next++)
            if (is_expect(&ev[next])) apply_or_fail(&ev[next], &plant, &out, &vcu, res, path);

        plant_step(&plant, &out, (uint16_t)tick_ms);

        if (verbose && t % 100 == 0)
            printf("  t=%5" PRIu32 "  %-10s pedal=%4u brake=%4u torque=%5d faults=0x%06" PRIX32 "\n",
                   t, vcu_state_name(out.state), out.pedal, out.brake,
                   out.torque_cmd, out.faults);
    }
    return res->failures == 0;
}
