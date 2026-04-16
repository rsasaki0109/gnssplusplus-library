// Wrapper to call CLASLIB's postpos() from gnssplusplus
// Compile: gcc -o claslib_ppp claslib_wrapper.c -L/tmp -lclaslib -lm -lpthread
#include <stdio.h>
#include <string.h>

// CLASLIB header
#include "rtklib.h"

// Stubs for GUI callbacks
int showmsg(char *fmt, ...) { return 0; }
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t t) {}

int main(int argc, char **argv) {
    if (argc < 5) {
        fprintf(stderr, "Usage: claslib_ppp <obs> <nav> <l6> <out> [conf]\n");
        return 1;
    }

    gtime_t ts = {0}, te = {0};
    double tint = 0.0;
    prcopt_t prcopt = prcopt_default;
    solopt_t solopt = solopt_default;
    filopt_t filopt = {""};
    char *infile[3], outfile[1024];

    infile[0] = argv[1];  // obs
    infile[1] = argv[2];  // nav
    infile[2] = argv[3];  // l6
    strncpy(outfile, argv[4], sizeof(outfile) - 1);

    // Load config if provided
    if (argc > 5) {
        resetsysopts();
        loadopts(argv[5], sysopts);
        getsysopts(&prcopt, &solopt, &filopt);
    } else {
        // Default CLAS PPP-RTK config (matching static.conf)
        prcopt.mode = 9;  // ppp-rtk
        prcopt.nf = 3;    // l1+l2+l5
        prcopt.navsys = 25; // GPS+GAL+QZS
        prcopt.elmin = 15.0 * D2R;
        prcopt.sateph = EPHOPT_BRDC + 3; // brdc+ssrapc
        prcopt.ionoopt = 9; // est-adaptive
        prcopt.tropopt = 0; // off
        prcopt.posopt[1] = 1; // receiver antenna
        prcopt.posopt[2] = 1; // phase windup
        prcopt.posopt[3] = 1; // eclipse
        prcopt.posopt[4] = 1; // raim
        prcopt.posopt[5] = 2; // compensate time variation (meas)
        prcopt.posopt[6] = 1; // partial AR
        prcopt.posopt[7] = 1; // shapiro
        solopt.posf = SOLF_NMEA;
    }

    // Set grid file
    strncpy(filopt.grid,
            "/media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/clas_grid.def",
            sizeof(filopt.grid) - 1);
    strncpy(filopt.blq,
            "/media/sasaki/aiueo/ai_coding_ws/gnssplusplus_thesis_ws/data/clas/claslib/data/clas_grid.blq",
            sizeof(filopt.blq) - 1);

    int ret = postpos(ts, te, tint, 0.0, &prcopt, &solopt, &filopt, infile, 3, outfile, "", "");

    printf("postpos returned: %d\n", ret);
    return ret ? 0 : 1;
}
