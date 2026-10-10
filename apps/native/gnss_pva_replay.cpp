// Offline file adapter; the processor sees only records admitted at each
// simulated reception event. No reference trajectory is opened here.
#include <libgnss++/fusion/online_pva_csv.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace libgnss;
namespace fs = std::filesystem;
namespace {
void open(io::RINEXReader& reader, const fs::path& path, io::RINEXReader::RINEXHeader& header) {
    if (!reader.open(path.string()) || !reader.readHeader(header))
        throw std::runtime_error("cannot read RINEX header: " + path.string());
}
}
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout << "gnss_pva_replay RAW_RUN NEW_OUTPUT_DIR MAX_EPOCHS [normal|gnss_outage|imu_gap|loose_only] [START_S DURATION_S] [--candidate none|vehicle_nhc_latched_v1|velocity_consistency_v1|velocity_consistency_v2|velocity_consistency_v3|velocity_consistency_v4|velocity_consistency_v5|velocity_consistency_v6|velocity_consistency_v7|velocity_consistency_v8|velocity_consistency_v9|velocity_consistency_v10|rtk_base_extrapolation_v1|rtk_online_product_v1]\n"
                "RAW_RUN is <tokyo|nagoya>/<run> (PPC) or urbannav/<run> (zero lever arm).\n"
                "MAX_EPOCHS=0 means full input. Body FLU, local ENU, GPST. No reference input.\n";
            return 0;
        }
        int positional_argc = argc;
        std::string candidate = "none";
        if (argc >= 6 && std::string(argv[argc-2]) == "--candidate") {
            candidate = argv[argc-1];
            positional_argc -= 2;
        }
        if (positional_argc < 4 || positional_argc > 7 || positional_argc == 6 ||
            (candidate != "none" && candidate != "vehicle_nhc_latched_v1" &&
             candidate != "velocity_consistency_v1" && candidate != "velocity_consistency_v2" &&
             candidate != "velocity_consistency_v3" && candidate != "velocity_consistency_v4" &&
             candidate != "velocity_consistency_v5" && candidate != "velocity_consistency_v6" &&
             candidate != "velocity_consistency_v7" && candidate != "velocity_consistency_v8" &&
             candidate != "velocity_consistency_v9" && candidate != "velocity_consistency_v10" &&
             candidate != "rtk_base_extrapolation_v1" && candidate != "rtk_online_product_v1"))
            throw std::invalid_argument("see --help for argument contract");
        const fs::path data(argv[1]), output(argv[2]);
        std::size_t consumed = 0;
        const int limit = std::stoi(argv[3], &consumed);
        if (consumed != std::string(argv[3]).size()) throw std::invalid_argument("invalid epoch limit");
        const std::string scenario = positional_argc >= 5 ? argv[4] : "normal";
        const double start_s = positional_argc == 7 ? std::stod(argv[5]) : 60.0;
        const double duration_s = positional_argc == 7 ? std::stod(argv[6]) : (scenario == "imu_gap" ? 4.0 : 10.0);
        if (limit < 0 || fs::exists(output) || !std::isfinite(start_s) || start_s < 0.0 ||
            !std::isfinite(duration_s) || duration_s <= 0.0 ||
            (scenario != "normal" && scenario != "gnss_outage" && scenario != "imu_gap" && scenario != "loose_only"))
            throw std::invalid_argument("invalid replay scenario or existing output directory");
        io::RINEXReader rover_reader, base_reader, nav_reader;
        io::RINEXReader::RINEXHeader rover_header, base_header, nav_header;
        open(rover_reader, data / "rover.obs", rover_header);
        open(base_reader, data / "base.obs", base_header);
        open(nav_reader, data / "base.nav", nav_header);
        NavigationData archive;
        if (!nav_reader.readNavigationData(archive)) throw std::runtime_error("cannot read navigation");
        ImuSeries imu;
        const auto loaded = loadImuCsv((data / "imu.csv").string(), imu);
        if (!loaded.ok) throw std::runtime_error(loaded.error);
        imu.sortByTime();
        OnlineRtkImuProcessor::Config config;
        config.base_position_ecef = base_header.approximate_position;
        const auto city = fs::absolute(data).lexically_normal().parent_path().filename().string();
        if (city != "tokyo" && city != "nagoya" && city != "urbannav")
            throw std::invalid_argument("expected PPC <tokyo|nagoya>/<run> or urbannav/<run> directory layout");
        const bool nagoya = city == "nagoya";
        // velocity_consistency_v8 contains everything of velocity_consistency_v7,
        // velocity_consistency_v9 everything of velocity_consistency_v8, and
        // velocity_consistency_v10 everything of velocity_consistency_v9.
        const bool v9_or_later = candidate == "velocity_consistency_v9" || candidate == "velocity_consistency_v10";
        const bool v8_or_later = candidate == "velocity_consistency_v8" || v9_or_later;
        const bool v7_or_later = candidate == "velocity_consistency_v7" || v8_or_later;
        // urbannav: UrbanNav documents no antenna-IMU lever arm; zero is a declared
        // assumption (docs/online_pva_default_switch_holdout_v1.md).
        config.fusion.lever_arm_body = city == "urbannav" ? Vector3d(0., 0., 0.)
            : nagoya ? Vector3d(.593, -.670, -1.216) : Vector3d(.31, 0., .55);
        config.tight_time_update = scenario != "loose_only";
        if (candidate == "velocity_consistency_v3" || candidate == "velocity_consistency_v4" ||
            candidate == "velocity_consistency_v5" || candidate == "velocity_consistency_v6" ||
            v7_or_later) {
            // v3 frozen in docs/online_pva_candidate_v4.md. Fixed values, not tuned.
            // The control fusion configuration is snapshotted first so the RTK
            // filter's INS prior is produced by an isolated control filter.
            config.rtk_prior_fusion = config.fusion;
            config.rtk.reported_covariance_mode =
                RTKProcessor::RTKConfig::ReportedCovarianceMode::SPP_CONSISTENCY_SCALED;
            config.fusion.max_position_update_nis_per_observation = 9.0;
            config.fusion.max_velocity_update_nis_per_observation = 9.0;
            config.fusion.float_position_reanchor_after_rejections = 30;
        }
        if (candidate == "velocity_consistency_v4" || candidate == "velocity_consistency_v5" ||
            candidate == "velocity_consistency_v6" || v7_or_later) {
            // v4 = v3 + post-gap re-anchor, frozen in docs/online_pva_candidate_v5.md.
            // The gap horizon is the existing coarse-position currency horizon.
            config.fusion.position_reanchor_after_gnss_gap_s =
                config.fusion.float_reanchor_max_coarse_age_s;
        }
        if (candidate == "velocity_consistency_v5" || candidate == "velocity_consistency_v6" ||
            v7_or_later) {
            // v5 = v4 + heading-latch direction test + rover-gap RTK-only reset,
            // frozen in docs/online_pva_candidate_v6.md. No constants. The
            // direction test is applied to the fused filter only (the
            // rtk_prior_fusion snapshot above does not carry it).
            config.fusion.heading_latch_direction_test = true;
            config.rover_gap_keeps_inertial_filters = true;
        }
        if (candidate == "velocity_consistency_v6" || v7_or_later) {
            // v6 = v5 + gyro-bias carry across fused-filter resets, frozen in
            // docs/online_pva_candidate_v7.md. No constants. The RTK-prior
            // filter is recreated without a seed.
            config.carry_gyro_bias_across_reset = true;
        }
        if (v7_or_later) {
            // v7 = v6 + the RTK input of rtk_online_product_v1 (library low-cost
            // preset, 2 s causal base hold, independent Doppler velocity), frozen
            // in docs/online_pva_candidate_v8.md. No new option or constant. The
            // rtk_prior_fusion snapshot above is the control fusion config, as in
            // v3-v6; the preset applies to the RTK filter only.
            config.rtk_preset = "low-cost";
            config.base_extrapolation_max_age_s = 2.0;
            config.independent_doppler_velocity = true;
        }
        if (v8_or_later) {
            // v8 = v7 + four default-off options, frozen in
            // docs/online_pva_candidate_v9.md. The 3.0 s horizon is the one of
            // the sibling trusted-jump rule in the same function. RTK options
            // are set on config.rtk, which the preset (applied to a copy in
            // OnlineRtkImuProcessor::recreateRtkFilter) does not touch; the
            // fusion option goes to config.fusion after the rtk_prior_fusion
            // snapshot, so the isolated prior filter keeps the v7 settings.
            config.rtk.reject_float_seeded_at_base = true;
            config.rtk.spp_fallback_blank_max_anchor_age_s = 3.0;
            config.fusion.reanchor_requires_prefit_gate_pass = true;
            config.independent_velocity_from_epoch_spp = true;
        }
        if (v9_or_later) {
            // v9 = v8 + the existing velocity_consistency_v1 option
            // reanchor_velocity_on_heading_latch on the fused filter only,
            // frozen in docs/online_pva_candidate_v10.md. Set after the
            // rtk_prior_fusion snapshot, so the isolated prior filter keeps
            // the v7 settings. No new option or constant.
            config.fusion.reanchor_velocity_on_heading_latch = true;
        }
        if (candidate == "velocity_consistency_v10") {
            // v10 = v9 + the Schmidt-Kalman consider update of the attitude and
            // both bias states before the heading latch, on the fused filter
            // only, frozen in docs/online_pva_candidate_v11.md. Set after the
            // rtk_prior_fusion snapshot, so the isolated prior filter keeps
            // the v7 settings. No new constant.
            config.fusion.consider_attitude_and_biases_before_heading_latch = true;
        }
        if (candidate == "rtk_online_product_v1") {
            // Candidate none + product RTK configuration, frozen in
            // docs/online_rtk_product_config_v1.md: library low-cost preset,
            // 2 s causal base hold (as rtk_base_extrapolation_v1) and the
            // independent Doppler velocity (as velocity_consistency_v1, without
            // its fusion options). No new constant.
            config.rtk_preset = "low-cost";
            config.base_extrapolation_max_age_s = 2.0;
            config.independent_doppler_velocity = true;
        }
        if (candidate == "rtk_base_extrapolation_v1") {
            // Candidate none + base extrapolation, frozen in
            // docs/online_rtk_base_extrapolation_v1.md. The 2 s horizon is the
            // batch kMaxInterpolationGapSeconds, not tuned. Nothing else changes.
            config.base_extrapolation_max_age_s = 2.0;
        }
        // Any candidate that holds a past base epoch writes the extrapolated_base
        // column and the extrapolation fields of replay.json.
        const bool extrapolation_candidate = config.base_extrapolation_max_age_s > 0.0;
        if (candidate == "vehicle_nhc_latched_v1") {
            config.fusion.nhc_enable = true;
            config.fusion.nhc_require_heading_alignment = true;
        }
        if (candidate == "velocity_consistency_v1" || candidate == "velocity_consistency_v2") {
            // v1 frozen in docs/online_pva_candidate_v2.md. Fixed values, not tuned.
            config.independent_doppler_velocity = true;
            config.fusion.reanchor_velocity_on_heading_latch = true;
            config.fusion.max_position_update_nis_per_observation = 9.0;
            config.fusion.max_velocity_update_nis_per_observation = 9.0;
        }
        if (candidate == "velocity_consistency_v2") {
            // v2 = v1 + FLOAT/FIXED gate-lockout recovery, frozen in
            // docs/online_pva_candidate_v3.md. Fixed value, not tuned.
            config.fusion.float_position_reanchor_after_rejections = 30;
        }
        OnlineRtkImuProcessor processor(config);
        fs::create_directories(output);
        std::ofstream csv(output / "pva.csv"), meta(output / "replay.json");
        if (!csv || !meta) throw std::runtime_error("cannot create replay output");
        csv << kOnlinePvaCsvHeader << (extrapolation_candidate ? kOnlinePvaCsvExtrapolatedBaseColumn : "") << '\n';
        ObservationData rover, base;
        bool have_base = base_reader.readObservationEpoch(base);
        std::size_t imu_cursor = 0;
        std::vector<Ephemeris> records;
        for (const auto& entry : archive.ephemeris_data)
            for (const auto& eph : entry.second) if (eph.valid) records.push_back(eph);
        std::vector<bool> sent(records.size(), false);
        int count = 0;
        GNSSTime start;
        while ((limit == 0 || count < limit) && rover_reader.readObservationEpoch(rover)) {
            if (count == 0) {
                start = rover.time;
                while (imu_cursor < imu.size() && imu.samples[imu_cursor].time < start - 2.5) ++imu_cursor;
            }
            NavigationData received;
            for (std::size_t i = 0; i < records.size(); ++i) {
                if (!sent[i] && records[i].toc <= rover.time && records[i].tof <= rover.time) {
                    received.addEphemeris(records[i]); sent[i] = true;
                }
            }
            processor.pushNavigation(received, rover.time);
            while (imu_cursor < imu.size() && imu.samples[imu_cursor].time <= rover.time) {
                const auto& sample = imu.samples[imu_cursor++];
                const double age = sample.time - start;
                if (!(scenario == "imu_gap" && age >= start_s && age < start_s + duration_s))
                    processor.pushImu(sample, rover.time);
            }
            while (have_base && base.time <= rover.time) {
                processor.pushBase(base, rover.time);
                have_base = base_reader.readObservationEpoch(base);
            }
            const double age = rover.time - start;
            const bool outage = scenario == "gnss_outage" && age >= start_s && age < start_s + duration_s;
            writeOnlinePvaCsv(csv, rover.time, processor.processRover(
                outage ? ObservationData(rover.time) : rover, rover.time), extrapolation_candidate);
            ++count;
        }
        if (count == 0 || (limit > 0 && count != limit)) throw std::runtime_error("insufficient rover epochs");
        const auto diagnostics = processor.diagnostics();
        meta << std::setprecision(17) << "{\"schema\":\"libgnsspp.online_pva_replay.v1\",\"state\":\"passed\","
            "\"reference_used\":false,\"receipt_times\":\"simulated at rover epoch\","
            "\"imu_axes\":\"raw xyz is FLU identity; deg/s converted once to rad/s by CSV loader\","
            "\"candidate\":\"" << candidate << "\",\"nhc_enable\":" << (config.fusion.nhc_enable ? "true" : "false") << ','
            << "\"navigation_policy\":\"toc and tof <= rover epoch\",\"scenario\":\"" << scenario
            << "\",\"scenario_start_s\":" << start_s << ",\"scenario_duration_s\":" << duration_s
            << ",\"epochs\":" << count << ",\"max_epochs\":" << limit << ",\"reset_count\":"
            << diagnostics.reset_generation << ",\"start_week\":" << start.week << ",\"start_tow\":" << start.tow
            << ",\"base_ecef\":[" << config.base_position_ecef.x() << ',' << config.base_position_ecef.y() << ','
            << config.base_position_ecef.z() << "],\"lever_arm_flu_m\":[" << config.fusion.lever_arm_body.x() << ','
            << config.fusion.lever_arm_body.y() << ',' << config.fusion.lever_arm_body.z() << "]";
        // Candidate-only fields; the default replay.json is unchanged.
        if (extrapolation_candidate)
            meta << ",\"base_extrapolation_max_age_s\":" << config.base_extrapolation_max_age_s
                 << ",\"extrapolated_base_epochs\":" << diagnostics.extrapolated_base_epochs
                 << ",\"missing_base_epochs\":" << diagnostics.missing_base_epochs;
        if (!config.rtk_preset.empty())
            meta << ",\"rtk_preset\":\"" << config.rtk_preset << "\""
                 << ",\"independent_doppler_velocity\":"
                 << (config.independent_doppler_velocity ? "true" : "false");
        if (v8_or_later)
            meta << ",\"reject_float_seeded_at_base\":" << (config.rtk.reject_float_seeded_at_base ? "true" : "false")
                 << ",\"spp_fallback_blank_max_anchor_age_s\":" << config.rtk.spp_fallback_blank_max_anchor_age_s
                 << ",\"reanchor_requires_prefit_gate_pass\":" << (config.fusion.reanchor_requires_prefit_gate_pass ? "true" : "false")
                 << ",\"independent_velocity_from_epoch_spp\":" << (config.independent_velocity_from_epoch_spp ? "true" : "false")
                 << ",\"rtk_base_seed_rejections\":" << diagnostics.rtk_base_seed_rejections
                 << ",\"rtk_spp_blank_age_limited\":" << diagnostics.rtk_spp_blank_age_limited
                 << ",\"rtk_float_prefit_gate_exceeded\":" << diagnostics.rtk_float_prefit_gate_exceeded
                 << ",\"fusion_reanchor_prefit_refusals\":" << diagnostics.fusion_reanchor_prefit_refusals
                 << ",\"epoch_spp_velocity_exports\":" << diagnostics.epoch_spp_velocity_exports;
        if (v9_or_later)
            meta << ",\"reanchor_velocity_on_heading_latch\":"
                 << (config.fusion.reanchor_velocity_on_heading_latch ? "true" : "false");
        if (candidate == "velocity_consistency_v10")
            meta << ",\"consider_attitude_and_biases_before_heading_latch\":"
                 << (config.fusion.consider_attitude_and_biases_before_heading_latch ? "true" : "false");
        meta << "}\n";
        std::cout << "replayed " << count << " epochs (" << scenario << ")\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << "gnss_pva_replay: " << error.what() << '\n'; return 2; }
}
