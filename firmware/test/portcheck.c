/* Differential harness: run the ported C against cases the Python generates.
 *
 * The conventions require every algorithm to be proven in Python and then
 * "re-verified against the same corpus" after the port. This is the program
 * that makes that possible. It reads cases on stdin, one per line, and writes
 * the C answers on stdout; analysis/tests/test_c_port.py generates the cases,
 * computes the Python answers, and compares.
 *
 * Why it matters more here than on most projects: there is no JTAG on this
 * board (parent spec 14.3). A translation error found after the port is found
 * with printf, on hardware, against no reference. Found here it is a diff.
 *
 * Deliberately dependency-free and line-oriented so it builds with whatever
 * compiler is to hand -- MSVC on this machine, xtensa-gcc on the device,
 * anything on a CI box.
 */

#include <stdio.h>
#include <string.h>

#include "plumb/quat.h"

#define LINE_MAX_CHARS 512

static void print_reals(const pl_real *values, int count)
{
    int i;
    for (i = 0; i < count; i++) {
        /* 17 significant digits round-trips a double exactly, so the harness
         * compares numbers rather than their decimal shadows. */
        printf("%s%.17g", i ? " " : "", (double)values[i]);
    }
    printf("\n");
}

int main(void)
{
    char line[LINE_MAX_CHARS];

    while (fgets(line, sizeof(line), stdin) != NULL) {
        char op[32];
        double a[4], b[4], v[3], dt;
        pl_real qa[4], qb[4], qv[3], out4[4], out3[3], out9[9];
        int i;

        if (sscanf(line, "%31s", op) != 1) {
            continue;
        }

        if (strcmp(op, "identity") == 0) {
            pl_quat_identity(out4);
            print_reals(out4, 4);

        } else if (strcmp(op, "multiply") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3],
                       &b[0], &b[1], &b[2], &b[3]) != 9) {
                fprintf(stderr, "bad multiply: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; qb[i] = (pl_real)b[i]; }
            pl_quat_multiply(qa, qb, out4);
            print_reals(out4, 4);

        } else if (strcmp(op, "conjugate") == 0
                   || strcmp(op, "normalize") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3]) != 5) {
                fprintf(stderr, "bad %s: %s", op, line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            if (op[0] == 'c') {
                pl_quat_conjugate(qa, out4);
            } else {
                pl_quat_normalize(qa, out4);
            }
            print_reals(out4, 4);

        } else if (strcmp(op, "rotate") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3],
                       &v[0], &v[1], &v[2]) != 8) {
                fprintf(stderr, "bad rotate: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            pl_quat_rotate(qa, qv, out3);
            print_reals(out3, 3);

        } else if (strcmp(op, "integrate") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3],
                       &v[0], &v[1], &v[2], &dt) != 9) {
                fprintf(stderr, "bad integrate: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            pl_quat_integrate(qa, qv, (pl_real)dt, out4);
            print_reals(out4, 4);

        } else if (strcmp(op, "twist") == 0) {
            pl_real angle;
            if (sscanf(line, "%31s %lf %lf %lf %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3],
                       &v[0], &v[1], &v[2]) != 8) {
                fprintf(stderr, "bad twist: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            angle = pl_quat_twist_angle(qa, qv);
            print_reals(&angle, 1);

        } else if (strcmp(op, "axisangle") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf", op,
                       &v[0], &v[1], &v[2], &dt) != 5) {
                fprintf(stderr, "bad axisangle: %s", line);
                return 1;
            }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            pl_quat_from_axis_angle(qv, (pl_real)dt, out4);
            print_reals(out4, 4);

        } else if (strcmp(op, "tomatrix") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3]) != 5) {
                fprintf(stderr, "bad tomatrix: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            pl_quat_to_matrix(qa, out9);
            print_reals(out9, 9);

        } else if (strcmp(op, "precision") == 0) {
            printf("%s\n", PL_REAL_NAME);

        } else {
            fprintf(stderr, "unknown op: %s", line);
            return 1;
        }
    }

    return 0;
}
