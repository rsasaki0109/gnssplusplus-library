# PPC wrong-FIX event ledger

Reference truth labels events offline only; fingerprints use runtime POS/debug telemetry.

Wrong FIX: **574 epochs / 188 events**; events containing >10 m error: **4**.

| Event | Run | TOW span | Epochs | Max error | >10 m | Fingerprints | Debug |
|---|---|---:|---:|---:|---:|---|---|
| `tokyo_run1-2324-189450.800-066` | tokyo_run1 | 189450.800-189451.000 | 2 | 47.956 m | 2 | high_prefit_basin, outlier_suppression_storm | no |
| `nagoya_run1-2325-551066.800-003` | nagoya_run1 | 551066.800-551066.800 | 1 | 13.766 m | 1 | unclassified | no |
| `nagoya_run1-2325-551063.800-001` | nagoya_run1 | 551063.800-551063.800 | 1 | 12.086 m | 1 | unclassified | no |
| `nagoya_run1-2325-551064.400-002` | nagoya_run1 | 551064.400-551064.400 | 1 | 12.017 m | 1 | unclassified | no |
| `tokyo_run1-2324-188203.400-005` | tokyo_run1 | 188203.400-188203.400 | 1 | 9.083 m | 0 | unclassified | no |
| `tokyo_run1-2324-188203.800-006` | tokyo_run1 | 188203.800-188203.800 | 1 | 9.020 m | 0 | unclassified | no |
| `tokyo_run1-2324-188210.400-009` | tokyo_run1 | 188210.400-188210.600 | 2 | 8.926 m | 0 | unclassified | no |
| `tokyo_run1-2324-188207.200-007` | tokyo_run1 | 188207.200-188207.600 | 3 | 8.915 m | 0 | unclassified | no |
| `tokyo_run1-2324-188208.400-008` | tokyo_run1 | 188208.400-188208.400 | 1 | 8.895 m | 0 | unclassified | no |
| `nagoya_run3-2325-554681.400-027` | nagoya_run3 | 554681.400-554682.200 | 5 | 8.593 m | 0 | unclassified | no |
| `tokyo_run1-2324-188247.200-015` | tokyo_run1 | 188247.200-188247.200 | 1 | 8.300 m | 0 | unclassified | no |
| `tokyo_run1-2324-188244.600-014` | tokyo_run1 | 188244.600-188244.600 | 1 | 8.223 m | 0 | unclassified | no |
| `nagoya_run2-2323-557220.200-014` | nagoya_run2 | 557220.200-557220.200 | 1 | 8.031 m | 0 | high_prefit_basin, outlier_suppression_storm | no |
| `nagoya_run2-2323-556046.800-002` | nagoya_run2 | 556046.800-556047.000 | 2 | 7.977 m | 0 | high_prefit_basin, low_ar_margin | no |
| `tokyo_run3-2324-182077.200-034` | tokyo_run3 | 182077.200-182077.400 | 2 | 7.549 m | 0 | low_ar_margin | no |
| `tokyo_run2-2324-178245.000-013` | tokyo_run2 | 178245.000-178245.000 | 1 | 7.052 m | 0 | unclassified | no |
| `nagoya_run1-2325-551586.400-010` | nagoya_run1 | 551586.400-551586.600 | 2 | 7.052 m | 0 | unclassified | no |
| `tokyo_run3-2324-180837.600-014` | tokyo_run3 | 180837.600-180837.600 | 1 | 6.863 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-180824.600-012` | tokyo_run3 | 180824.600-180825.200 | 4 | 6.755 m | 0 | high_prefit_basin, outlier_suppression_storm, low_ar_margin | no |
| `tokyo_run2-2324-177722.600-008` | tokyo_run2 | 177722.600-177723.400 | 5 | 6.747 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-180825.800-013` | tokyo_run3 | 180825.800-180825.800 | 1 | 6.618 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-180070.000-005` | tokyo_run3 | 180070.000-180070.800 | 5 | 6.047 m | 0 | unclassified | no |
| `nagoya_run2-2323-556749.000-011` | nagoya_run2 | 556749.000-556749.200 | 2 | 5.370 m | 0 | high_prefit_basin, outlier_suppression_storm, low_ar_margin | no |
| `nagoya_run2-2323-556769.000-013` | nagoya_run2 | 556769.000-556769.000 | 1 | 5.191 m | 0 | high_prefit_basin, low_ar_margin | no |
| `nagoya_run3-2325-554711.200-030` | nagoya_run3 | 554711.200-554712.800 | 9 | 4.746 m | 0 | unclassified | no |
| `nagoya_run2-2323-556746.200-010` | nagoya_run2 | 556746.200-556746.200 | 1 | 4.675 m | 0 | high_prefit_basin | no |
| `nagoya_run2-2323-556749.600-012` | nagoya_run2 | 556749.600-556749.600 | 1 | 4.661 m | 0 | high_prefit_basin, low_ar_margin | no |
| `nagoya_run2-2323-556739.000-008` | nagoya_run2 | 556739.000-556739.000 | 1 | 4.619 m | 0 | high_prefit_basin, outlier_suppression_storm | no |
| `nagoya_run2-2323-556741.400-009` | nagoya_run2 | 556741.400-556742.800 | 8 | 4.587 m | 0 | high_prefit_basin | no |
| `nagoya_run2-2323-556738.600-007` | nagoya_run2 | 556738.600-556738.600 | 1 | 4.450 m | 0 | high_prefit_basin, outlier_suppression_storm | no |
| `tokyo_run1-2324-188139.800-004` | tokyo_run1 | 188139.800-188140.600 | 5 | 4.438 m | 0 | low_ar_margin | no |
| `nagoya_run1-2325-551587.000-011` | nagoya_run1 | 551587.000-551587.800 | 5 | 4.305 m | 0 | unclassified | no |
| `tokyo_run3-2324-182401.800-041` | tokyo_run3 | 182401.800-182402.000 | 2 | 3.750 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-179710.800-003` | tokyo_run3 | 179710.800-179710.800 | 1 | 3.740 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-179710.400-002` | tokyo_run3 | 179710.400-179710.400 | 1 | 3.731 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-189179.400-044` | tokyo_run1 | 189179.400-189180.000 | 4 | 3.635 m | 0 | unclassified | no |
| `nagoya_run1-2325-551591.400-012` | nagoya_run1 | 551591.400-551593.400 | 11 | 3.598 m | 0 | unclassified | no |
| `tokyo_run1-2324-189099.800-036` | tokyo_run1 | 189099.800-189101.800 | 11 | 3.544 m | 0 | unclassified | no |
| `tokyo_run1-2324-188138.800-003` | tokyo_run1 | 188138.800-188138.800 | 1 | 3.355 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-181684.800-025` | tokyo_run3 | 181684.800-181685.000 | 2 | 3.355 m | 0 | low_ar_margin | no |
| `tokyo_run2-2324-178404.000-018` | tokyo_run2 | 178404.000-178404.600 | 4 | 3.336 m | 0 | unclassified | no |
| `nagoya_run1-2325-551573.000-009` | nagoya_run1 | 551573.000-551573.200 | 2 | 3.140 m | 0 | unclassified | no |
| `tokyo_run1-2324-189440.800-065` | tokyo_run1 | 189440.800-189441.200 | 3 | 3.087 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-189097.400-035` | tokyo_run1 | 189097.400-189098.800 | 8 | 3.040 m | 0 | unclassified | no |
| `tokyo_run2-2324-178317.200-017` | tokyo_run2 | 178317.200-178317.200 | 1 | 2.857 m | 0 | low_ar_margin | no |
| `nagoya_run3-2325-554235.200-006` | nagoya_run3 | 554235.200-554235.200 | 1 | 2.660 m | 0 | unclassified | no |
| `tokyo_run3-2324-180799.200-011` | tokyo_run3 | 180799.200-180799.400 | 2 | 2.282 m | 0 | low_ar_margin | no |
| `nagoya_run2-2323-556204.200-004` | nagoya_run2 | 556204.200-556204.200 | 1 | 2.272 m | 0 | low_ar_margin | no |
| `nagoya_run2-2323-556375.200-005` | nagoya_run2 | 556375.200-556375.200 | 1 | 2.252 m | 0 | unclassified | no |
| `tokyo_run1-2324-189172.400-040` | tokyo_run1 | 189172.400-189173.400 | 6 | 2.191 m | 0 | high_prefit_basin | no |
| `nagoya_run3-2325-554245.800-009` | nagoya_run3 | 554245.800-554246.200 | 3 | 2.156 m | 0 | unclassified | no |
| `nagoya_run3-2325-554423.400-020` | nagoya_run3 | 554423.400-554423.600 | 2 | 2.155 m | 0 | unclassified | no |
| `tokyo_run1-2324-188221.800-012` | tokyo_run1 | 188221.800-188225.600 | 20 | 2.155 m | 0 | unclassified | no |
| `tokyo_run1-2324-188541.800-027` | tokyo_run1 | 188541.800-188542.000 | 2 | 2.152 m | 0 | unclassified | no |
| `tokyo_run1-2324-188220.000-011` | tokyo_run1 | 188220.000-188221.400 | 8 | 2.138 m | 0 | unclassified | no |
| `tokyo_run3-2324-179711.200-004` | tokyo_run3 | 179711.200-179711.200 | 1 | 2.111 m | 0 | unclassified | no |
| `tokyo_run1-2324-188226.400-013` | tokyo_run1 | 188226.400-188226.600 | 2 | 2.105 m | 0 | unclassified | no |
| `tokyo_run1-2324-188219.400-010` | tokyo_run1 | 188219.400-188219.400 | 1 | 2.088 m | 0 | unclassified | no |
| `tokyo_run1-2324-189554.400-068` | tokyo_run1 | 189554.400-189555.600 | 7 | 1.992 m | 0 | unclassified | no |
| `tokyo_run2-2324-177710.200-007` | tokyo_run2 | 177710.200-177710.400 | 2 | 1.982 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-182398.200-039` | tokyo_run3 | 182398.200-182398.600 | 3 | 1.976 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-182069.400-033` | tokyo_run3 | 182069.400-182069.600 | 2 | 1.933 m | 0 | unclassified | no |
| `tokyo_run3-2324-182397.200-038` | tokyo_run3 | 182397.200-182397.800 | 4 | 1.931 m | 0 | low_ar_margin | no |
| `tokyo_run2-2324-178255.000-014` | tokyo_run2 | 178255.000-178255.000 | 1 | 1.878 m | 0 | unclassified | no |
| `tokyo_run1-2324-188572.600-031` | tokyo_run1 | 188572.600-188573.600 | 6 | 1.874 m | 0 | low_ar_margin | no |
| `nagoya_run3-2325-554005.000-001` | nagoya_run3 | 554005.000-554005.600 | 4 | 1.846 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-188541.000-026` | tokyo_run1 | 188541.000-188541.400 | 3 | 1.829 m | 0 | unclassified | no |
| `tokyo_run1-2324-188569.000-028` | tokyo_run1 | 188569.000-188569.000 | 1 | 1.811 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-182399.000-040` | tokyo_run3 | 182399.000-182399.200 | 2 | 1.758 m | 0 | unclassified | no |
| `nagoya_run3-2325-554713.200-031` | nagoya_run3 | 554713.200-554714.000 | 5 | 1.729 m | 0 | unclassified | no |
| `nagoya_run2-2323-557258.400-015` | nagoya_run2 | 557258.400-557258.400 | 1 | 1.695 m | 0 | unclassified | no |
| `tokyo_run3-2324-181357.800-020` | tokyo_run3 | 181357.800-181358.000 | 2 | 1.676 m | 0 | unclassified | no |
| `tokyo_run1-2324-189143.600-039` | tokyo_run1 | 189143.600-189143.600 | 1 | 1.642 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-189176.400-043` | tokyo_run1 | 189176.400-189176.600 | 2 | 1.641 m | 0 | unclassified | no |
| `tokyo_run1-2324-188570.800-029` | tokyo_run1 | 188570.800-188571.000 | 2 | 1.583 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-189128.200-037` | tokyo_run1 | 189128.200-189128.400 | 2 | 1.583 m | 0 | unclassified | no |
| `tokyo_run3-2324-181708.800-027` | tokyo_run3 | 181708.800-181720.000 | 57 | 1.555 m | 0 | high_prefit_basin, outlier_suppression_storm | no |
| `tokyo_run1-2324-189208.800-056` | tokyo_run1 | 189208.800-189214.200 | 28 | 1.491 m | 0 | unclassified | no |
| `tokyo_run1-2324-188699.400-032` | tokyo_run1 | 188699.400-188699.600 | 2 | 1.478 m | 0 | unclassified | no |
| `tokyo_run1-2324-189175.600-042` | tokyo_run1 | 189175.600-189176.000 | 3 | 1.458 m | 0 | unclassified | no |
| `tokyo_run1-2324-189173.800-041` | tokyo_run1 | 189173.800-189175.000 | 7 | 1.456 m | 0 | unclassified | no |
| `nagoya_run3-2325-554567.000-025` | nagoya_run3 | 554567.000-554567.000 | 1 | 1.430 m | 0 | unclassified | no |
| `tokyo_run3-2324-180967.200-015` | tokyo_run3 | 180967.200-180968.600 | 8 | 1.421 m | 0 | unclassified | no |
| `tokyo_run2-2324-178293.400-016` | tokyo_run2 | 178293.400-178294.000 | 4 | 1.380 m | 0 | unclassified | no |
| `nagoya_run3-2325-554422.400-019` | nagoya_run3 | 554422.400-554422.400 | 1 | 1.341 m | 0 | unclassified | no |
| `tokyo_run3-2324-180093.200-006` | tokyo_run3 | 180093.200-180093.400 | 2 | 1.340 m | 0 | unclassified | no |
| `tokyo_run1-2324-189214.800-057` | tokyo_run1 | 189214.800-189215.400 | 4 | 1.315 m | 0 | unclassified | no |
| `nagoya_run3-2325-554565.000-024` | nagoya_run3 | 554565.000-554565.000 | 1 | 1.310 m | 0 | high_prefit_basin | no |
| `tokyo_run1-2324-189232.400-064` | tokyo_run1 | 189232.400-189232.800 | 3 | 1.302 m | 0 | unclassified | no |
| `nagoya_run3-2325-554564.600-023` | nagoya_run3 | 554564.600-554564.600 | 1 | 1.292 m | 0 | unclassified | no |
| `nagoya_run3-2325-554725.800-032` | nagoya_run3 | 554725.800-554727.000 | 7 | 1.279 m | 0 | outlier_suppression_storm | no |
| `tokyo_run3-2324-181192.600-019` | tokyo_run3 | 181192.600-181192.600 | 1 | 1.249 m | 0 | high_prefit_basin | no |
| `tokyo_run1-2324-189231.200-063` | tokyo_run1 | 189231.200-189231.200 | 1 | 1.247 m | 0 | unclassified | no |
| `nagoya_run3-2325-554044.800-003` | nagoya_run3 | 554044.800-554044.800 | 1 | 1.194 m | 0 | outlier_suppression_storm, low_ar_margin | no |
| `nagoya_run3-2325-554351.000-011` | nagoya_run3 | 554351.000-554351.000 | 1 | 1.125 m | 0 | unclassified | no |
| `tokyo_run1-2324-188359.200-021` | tokyo_run1 | 188359.200-188363.200 | 21 | 1.120 m | 0 | high_prefit_basin, outlier_suppression_storm | no |
| `nagoya_run2-2323-556099.200-003` | nagoya_run2 | 556099.200-556099.200 | 1 | 1.105 m | 0 | high_prefit_basin, low_ar_margin | no |
| `tokyo_run3-2324-182065.600-032` | tokyo_run3 | 182065.600-182065.600 | 1 | 1.091 m | 0 | unclassified | no |
| `tokyo_run3-2324-181076.000-017` | tokyo_run3 | 181076.000-181076.000 | 1 | 1.079 m | 0 | unclassified | no |
| `nagoya_run3-2325-554563.000-021` | nagoya_run3 | 554563.000-554563.000 | 1 | 1.039 m | 0 | unclassified | no |
| `nagoya_run3-2325-554563.600-022` | nagoya_run3 | 554563.600-554564.000 | 3 | 1.037 m | 0 | unclassified | no |
| `tokyo_run3-2324-180647.200-008` | tokyo_run3 | 180647.200-180647.400 | 2 | 1.019 m | 0 | low_ar_margin | no |
| `nagoya_run1-2325-551146.800-004` | nagoya_run1 | 551146.800-551146.800 | 1 | 1.013 m | 0 | low_ar_margin | no |
| `tokyo_run2-2324-178292.400-015` | tokyo_run2 | 178292.400-178292.600 | 2 | 0.987 m | 0 | unclassified | no |
| `tokyo_run3-2324-181810.200-028` | tokyo_run3 | 181810.200-181810.200 | 1 | 0.984 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-188350.200-020` | tokyo_run1 | 188350.200-188351.200 | 6 | 0.977 m | 0 | high_prefit_basin | no |
| `tokyo_run3-2324-181816.000-030` | tokyo_run3 | 181816.000-181821.400 | 28 | 0.963 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-180785.200-010` | tokyo_run3 | 180785.200-180785.400 | 2 | 0.962 m | 0 | unclassified | no |
| `tokyo_run3-2324-181810.600-029` | tokyo_run3 | 181810.600-181810.800 | 2 | 0.959 m | 0 | low_ar_margin | no |
| `nagoya_run3-2325-554418.000-012` | nagoya_run3 | 554418.000-554418.000 | 1 | 0.958 m | 0 | unclassified | no |
| `nagoya_run3-2325-554240.000-007` | nagoya_run3 | 554240.000-554240.600 | 4 | 0.952 m | 0 | unclassified | no |
| `nagoya_run3-2325-554244.600-008` | nagoya_run3 | 554244.600-554244.600 | 1 | 0.946 m | 0 | high_prefit_basin, low_ar_margin | no |
| `nagoya_run1-2325-551343.400-008` | nagoya_run1 | 551343.400-551343.400 | 1 | 0.922 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-189217.800-060` | tokyo_run1 | 189217.800-189218.600 | 5 | 0.890 m | 0 | unclassified | no |
| `nagoya_run1-2325-551313.200-007` | nagoya_run1 | 551313.200-551313.400 | 2 | 0.877 m | 0 | unclassified | no |
| `nagoya_run3-2325-554420.000-015` | nagoya_run3 | 554420.000-554420.200 | 2 | 0.845 m | 0 | unclassified | no |
| `tokyo_run1-2324-189219.000-061` | tokyo_run1 | 189219.000-189219.000 | 1 | 0.839 m | 0 | unclassified | no |
| `tokyo_run3-2324-182396.200-037` | tokyo_run3 | 182396.200-182396.200 | 1 | 0.830 m | 0 | low_ar_margin | no |
| `nagoya_run3-2325-554418.600-013` | nagoya_run3 | 554418.600-554418.600 | 1 | 0.829 m | 0 | unclassified | no |
| `tokyo_run3-2324-179705.400-001` | tokyo_run3 | 179705.400-179705.400 | 1 | 0.828 m | 0 | unclassified | no |
| `tokyo_run2-2324-177775.000-009` | tokyo_run2 | 177775.000-177775.000 | 1 | 0.826 m | 0 | unclassified | no |
| `tokyo_run2-2324-178062.400-011` | tokyo_run2 | 178062.400-178062.600 | 2 | 0.825 m | 0 | unclassified | no |
| `tokyo_run1-2324-189216.400-059` | tokyo_run1 | 189216.400-189217.400 | 6 | 0.795 m | 0 | unclassified | no |
| `tokyo_run3-2324-181687.200-026` | tokyo_run3 | 181687.200-181687.200 | 1 | 0.793 m | 0 | high_prefit_basin, low_ar_margin | no |
| `tokyo_run1-2324-189140.600-038` | tokyo_run1 | 189140.600-189141.000 | 3 | 0.787 m | 0 | high_prefit_basin | no |
| `tokyo_run1-2324-188402.000-024` | tokyo_run1 | 188402.000-188402.200 | 2 | 0.786 m | 0 | unclassified | no |
| `tokyo_run3-2324-180564.600-007` | tokyo_run3 | 180564.600-180564.800 | 2 | 0.783 m | 0 | unclassified | no |
| `nagoya_run3-2325-554421.800-018` | nagoya_run3 | 554421.800-554421.800 | 1 | 0.781 m | 0 | unclassified | no |
| `tokyo_run2-2324-178208.600-012` | tokyo_run2 | 178208.600-178208.600 | 1 | 0.768 m | 0 | unclassified | no |
| `nagoya_run3-2325-554058.600-004` | nagoya_run3 | 554058.600-554059.200 | 4 | 0.767 m | 0 | unclassified | no |
| `tokyo_run1-2324-189543.000-067` | tokyo_run1 | 189543.000-189544.400 | 8 | 0.767 m | 0 | high_prefit_basin | no |
| `tokyo_run1-2324-189207.600-055` | tokyo_run1 | 189207.600-189208.000 | 3 | 0.753 m | 0 | unclassified | no |
| `tokyo_run3-2324-182063.200-031` | tokyo_run3 | 182063.200-182063.400 | 2 | 0.753 m | 0 | unclassified | no |
| `tokyo_run3-2324-182160.800-036` | tokyo_run3 | 182160.800-182161.000 | 2 | 0.742 m | 0 | unclassified | no |
| `tokyo_run1-2324-188571.400-030` | tokyo_run1 | 188571.400-188571.400 | 1 | 0.733 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-189198.000-048` | tokyo_run1 | 189198.000-189198.400 | 3 | 0.724 m | 0 | unclassified | no |
| `tokyo_run1-2324-189201.000-051` | tokyo_run1 | 189201.000-189201.200 | 2 | 0.721 m | 0 | unclassified | no |
| `tokyo_run2-2324-177466.800-002` | tokyo_run2 | 177466.800-177466.800 | 1 | 0.704 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-181463.200-023` | tokyo_run3 | 181463.200-181463.200 | 1 | 0.699 m | 0 | low_ar_margin | no |
| `nagoya_run3-2325-554421.400-017` | nagoya_run3 | 554421.400-554421.400 | 1 | 0.691 m | 0 | unclassified | no |
| `tokyo_run1-2324-188316.800-019` | tokyo_run1 | 188316.800-188317.600 | 5 | 0.676 m | 0 | high_prefit_basin, outlier_suppression_storm | no |
| `tokyo_run1-2324-188315.400-018` | tokyo_run1 | 188315.400-188316.400 | 6 | 0.671 m | 0 | high_prefit_basin, outlier_suppression_storm | no |
| `tokyo_run1-2324-188402.800-025` | tokyo_run1 | 188402.800-188402.800 | 1 | 0.665 m | 0 | unclassified | no |
| `tokyo_run1-2324-189199.000-049` | tokyo_run1 | 189199.000-189199.200 | 2 | 0.663 m | 0 | unclassified | no |
| `tokyo_run3-2324-181679.200-024` | tokyo_run3 | 181679.200-181679.600 | 3 | 0.662 m | 0 | unclassified | no |
| `nagoya_run3-2325-554163.000-005` | nagoya_run3 | 554163.000-554163.200 | 2 | 0.661 m | 0 | unclassified | no |
| `tokyo_run1-2324-189203.600-053` | tokyo_run1 | 189203.600-189203.600 | 1 | 0.661 m | 0 | unclassified | no |
| `tokyo_run1-2324-188700.200-033` | tokyo_run1 | 188700.200-188700.200 | 1 | 0.657 m | 0 | unclassified | no |
| `tokyo_run1-2324-188314.600-017` | tokyo_run1 | 188314.600-188314.800 | 2 | 0.650 m | 0 | high_prefit_basin, outlier_suppression_storm | no |
| `tokyo_run1-2324-189215.800-058` | tokyo_run1 | 189215.800-189215.800 | 1 | 0.650 m | 0 | unclassified | no |
| `tokyo_run1-2324-188314.200-016` | tokyo_run1 | 188314.200-188314.200 | 1 | 0.645 m | 0 | high_prefit_basin | no |
| `tokyo_run3-2324-181369.800-022` | tokyo_run3 | 181369.800-181370.200 | 3 | 0.627 m | 0 | unclassified | no |
| `tokyo_run3-2324-182116.400-035` | tokyo_run3 | 182116.400-182116.400 | 1 | 0.625 m | 0 | unclassified | no |
| `tokyo_run1-2324-189200.400-050` | tokyo_run1 | 189200.400-189200.400 | 1 | 0.621 m | 0 | unclassified | no |
| `tokyo_run1-2324-189219.400-062` | tokyo_run1 | 189219.400-189219.400 | 1 | 0.617 m | 0 | unclassified | no |
| `nagoya_run3-2325-554249.000-010` | nagoya_run3 | 554249.000-554249.000 | 1 | 0.613 m | 0 | unclassified | no |
| `tokyo_run2-2324-177059.000-001` | tokyo_run2 | 177059.000-177059.000 | 1 | 0.610 m | 0 | unclassified | no |
| `tokyo_run1-2324-188401.200-023` | tokyo_run1 | 188401.200-188401.200 | 1 | 0.608 m | 0 | unclassified | no |
| `nagoya_run3-2325-554420.800-016` | nagoya_run3 | 554420.800-554420.800 | 1 | 0.607 m | 0 | unclassified | no |
| `tokyo_run1-2324-189201.800-052` | tokyo_run1 | 189201.800-189201.800 | 1 | 0.602 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-188400.800-022` | tokyo_run1 | 188400.800-188400.800 | 1 | 0.600 m | 0 | unclassified | no |
| `nagoya_run3-2325-554686.000-029` | nagoya_run3 | 554686.000-554686.000 | 1 | 0.598 m | 0 | unclassified | no |
| `tokyo_run3-2324-181359.200-021` | tokyo_run3 | 181359.200-181359.200 | 1 | 0.584 m | 0 | unclassified | no |
| `tokyo_run1-2324-189197.400-047` | tokyo_run1 | 189197.400-189197.600 | 2 | 0.573 m | 0 | unclassified | no |
| `tokyo_run1-2324-189194.800-045` | tokyo_run1 | 189194.800-189194.800 | 1 | 0.571 m | 0 | unclassified | no |
| `nagoya_run1-2325-551282.800-005` | nagoya_run1 | 551282.800-551283.600 | 5 | 0.567 m | 0 | unclassified | no |
| `nagoya_run3-2325-554668.200-026` | nagoya_run3 | 554668.200-554668.200 | 1 | 0.566 m | 0 | unclassified | no |
| `tokyo_run1-2324-187792.000-001` | tokyo_run1 | 187792.000-187792.000 | 1 | 0.566 m | 0 | unclassified | no |
| `nagoya_run3-2325-554006.400-002` | nagoya_run3 | 554006.400-554006.400 | 1 | 0.563 m | 0 | unclassified | no |
| `tokyo_run2-2324-177928.400-010` | tokyo_run2 | 177928.400-177928.400 | 1 | 0.557 m | 0 | unclassified | no |
| `tokyo_run1-2324-189196.600-046` | tokyo_run1 | 189196.600-189196.600 | 1 | 0.543 m | 0 | unclassified | no |
| `nagoya_run3-2325-554419.400-014` | nagoya_run3 | 554419.400-554419.400 | 1 | 0.543 m | 0 | unclassified | no |
| `tokyo_run1-2324-189565.000-069` | tokyo_run1 | 189565.000-189565.000 | 1 | 0.541 m | 0 | low_ar_margin | no |
| `tokyo_run2-2324-178405.000-019` | tokyo_run2 | 178405.000-178405.000 | 1 | 0.532 m | 0 | unclassified | no |
| `nagoya_run2-2323-555989.400-001` | nagoya_run2 | 555989.400-555989.600 | 2 | 0.528 m | 0 | unclassified | no |
| `tokyo_run3-2324-180771.400-009` | tokyo_run3 | 180771.400-180771.400 | 1 | 0.527 m | 0 | unclassified | no |
| `nagoya_run2-2323-556628.600-006` | nagoya_run2 | 556628.600-556628.600 | 1 | 0.525 m | 0 | unclassified | no |
| `nagoya_run3-2325-554683.800-028` | nagoya_run3 | 554683.800-554683.800 | 1 | 0.524 m | 0 | unclassified | no |
| `tokyo_run1-2324-187807.600-002` | tokyo_run1 | 187807.600-187807.600 | 1 | 0.523 m | 0 | unclassified | no |
| `tokyo_run1-2324-189088.400-034` | tokyo_run1 | 189088.400-189088.400 | 1 | 0.519 m | 0 | unclassified | no |
| `tokyo_run2-2324-177621.400-004` | tokyo_run2 | 177621.400-177621.400 | 1 | 0.518 m | 0 | low_ar_margin | no |
| `tokyo_run2-2324-177622.200-005` | tokyo_run2 | 177622.200-177622.400 | 2 | 0.517 m | 0 | low_ar_margin | no |
| `tokyo_run2-2324-177624.000-006` | tokyo_run2 | 177624.000-177624.000 | 1 | 0.513 m | 0 | low_ar_margin | no |
| `tokyo_run3-2324-181024.800-016` | tokyo_run3 | 181024.800-181024.800 | 1 | 0.512 m | 0 | unclassified | no |
| `tokyo_run2-2324-177620.800-003` | tokyo_run2 | 177620.800-177621.000 | 2 | 0.509 m | 0 | low_ar_margin | no |
| `tokyo_run1-2324-189204.200-054` | tokyo_run1 | 189204.200-189204.400 | 2 | 0.508 m | 0 | unclassified | no |
| `tokyo_run3-2324-181192.200-018` | tokyo_run3 | 181192.200-181192.200 | 1 | 0.505 m | 0 | low_ar_margin | no |
| `nagoya_run1-2325-551301.600-006` | nagoya_run1 | 551301.600-551301.600 | 1 | 0.504 m | 0 | high_prefit_basin | no |
