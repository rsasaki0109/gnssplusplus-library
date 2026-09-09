#include <gtest/gtest.h>

#include <libgnss++/algorithms/fgo.hpp>

namespace {

using Diagnostics = libgnss::FGOProcessor::FGOProblemDiagnostics;
using Snapshot = Diagnostics::Phase131DiagnosticsSnapshot;

Snapshot populatedSnapshot() {
    Snapshot snapshot;
    snapshot.enabled = true;
    snapshot.configuration_valid = true;
    snapshot.configuration_failure = "";
    snapshot.canonical_rows = 11U;
    snapshot.canonical_rejected_rows = 3U;
    snapshot.unknown_band_rows = 2U;
    snapshot.canonical_key_conflicts = 1U;
    snapshot.canonical_duplicate_rows = 4U;
    snapshot.canonical_streams = 9U;
    snapshot.canonical_selected_streams = 7U;
    snapshot.canonical_merged_streams = 2U;
    snapshot.failure_counts = {{"unknown-band", 2U}, {"duplicate", 4U}};
    snapshot.canonicalization_attempt_rows = 14U;
    snapshot.resolver_call_count = 14U;
    snapshot.source_miss_mask_enabled = true;
    snapshot.source_miss_mask_canonical_key_mode = true;
    snapshot.source_miss_mask_matching_key =
        "(GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN])";
    snapshot.original_adopted_pseudorange_rows = 100U;
    snapshot.retained_finite_pc_pseudorange_rows = 91U;
    snapshot.dropped_missing_exact_stream_rows = 5U;
    snapshot.dropped_out_of_domain_rows = 3U;
    snapshot.dropped_nonfinite_correction_rows = 1U;
    snapshot.matched_factor_rows = 96U;
    snapshot.finite_correction_rows_among_matched = 91U;
    snapshot.source_model_build_count = 1U;
    snapshot.correction_application_pass_count = 1U;
    snapshot.corrected_rows = 91U;
    snapshot.pseudorange_factor_count_consistent = true;
    snapshot.signal_count_consistent = true;
    snapshot.applied = true;
    snapshot.correction_applied_exactly_once = true;
    snapshot.duplicate_correction_rejected = false;
    return snapshot;
}

}  // namespace

TEST(Phase134NativePhase131DiagnosticsBridge, CopiesEveryFieldExactly) {
    const Snapshot source = populatedSnapshot();
    Diagnostics destination;

    ASSERT_TRUE(destination.synchronizePhase131Diagnostics(source));
    EXPECT_EQ(destination.phase131_diagnostics_bridge_sync_count, 1U);
    EXPECT_TRUE(destination.phase131_canonical_correction_band_key_enabled);
    EXPECT_TRUE(destination.phase131_configuration_valid);
    EXPECT_EQ(destination.phase131_canonical_rows, 11U);
    EXPECT_EQ(destination.phase131_canonical_rejected_rows, 3U);
    EXPECT_EQ(destination.phase131_unknown_band_rows, 2U);
    EXPECT_EQ(destination.phase131_canonical_key_conflicts, 1U);
    EXPECT_EQ(destination.phase131_canonical_duplicate_rows, 4U);
    EXPECT_EQ(destination.phase131_canonical_streams, 9U);
    EXPECT_EQ(destination.phase131_canonical_selected_streams, 7U);
    EXPECT_EQ(destination.phase131_canonical_merged_streams, 2U);
    EXPECT_EQ(destination.phase131_failure_counts, source.failure_counts);
    EXPECT_EQ(destination.phase131_canonicalization_attempt_rows, 14U);
    EXPECT_EQ(destination.phase131_resolver_call_count, 14U);
    EXPECT_TRUE(destination.phase131_source_miss_mask_enabled);
    EXPECT_TRUE(destination.phase131_source_miss_mask_canonical_key_mode);
    EXPECT_EQ(destination.phase131_source_miss_mask_matching_key,
              source.source_miss_mask_matching_key);
    EXPECT_EQ(destination.phase131_original_adopted_pseudorange_rows, 100U);
    EXPECT_EQ(destination.phase131_retained_finite_pc_pseudorange_rows, 91U);
    EXPECT_EQ(destination.phase131_dropped_missing_exact_stream_rows, 5U);
    EXPECT_EQ(destination.phase131_dropped_out_of_domain_rows, 3U);
    EXPECT_EQ(destination.phase131_dropped_nonfinite_correction_rows, 1U);
    EXPECT_EQ(destination.phase131_matched_factor_rows, 96U);
    EXPECT_EQ(destination.phase131_finite_correction_rows_among_matched, 91U);
    EXPECT_EQ(destination.phase131_source_model_build_count, 1U);
    EXPECT_EQ(destination.phase131_correction_application_pass_count, 1U);
    EXPECT_EQ(destination.phase131_corrected_rows, 91U);
    EXPECT_TRUE(destination.phase131_pseudorange_factor_count_consistent);
    EXPECT_TRUE(destination.phase131_signal_count_consistent);
    EXPECT_TRUE(destination.phase131_applied);
    EXPECT_TRUE(destination.phase131_correction_applied_exactly_once);
    EXPECT_FALSE(destination.phase131_duplicate_correction_rejected);
}

TEST(Phase134NativePhase131DiagnosticsBridge,
     RejectsSecondSynchronizationWithoutAccumulatingCounters) {
    Diagnostics destination;
    const Snapshot first = populatedSnapshot();
    ASSERT_TRUE(destination.synchronizePhase131Diagnostics(first));

    Snapshot second = first;
    second.canonical_rows = 999U;
    second.canonicalization_attempt_rows = 1000U;
    second.resolver_call_count = 1000U;
    second.failure_counts = {{"replacement", 999U}};
    EXPECT_FALSE(destination.synchronizePhase131Diagnostics(second));
    EXPECT_EQ(destination.phase131_diagnostics_bridge_sync_count, 1U);
    EXPECT_EQ(destination.phase131_canonical_rows, 11U);
    EXPECT_EQ(destination.phase131_canonicalization_attempt_rows, 14U);
    EXPECT_EQ(destination.phase131_resolver_call_count, 14U);
    EXPECT_EQ(destination.phase131_failure_counts, first.failure_counts);
}

TEST(Phase134NativePhase131DiagnosticsBridge,
     PropagatesDisabledAndConfigurationFailureSnapshots) {
    Diagnostics disabled;
    Snapshot zero;
    ASSERT_TRUE(disabled.synchronizePhase131Diagnostics(zero));
    EXPECT_EQ(disabled.phase131_diagnostics_bridge_sync_count, 1U);
    EXPECT_FALSE(disabled.phase131_canonical_correction_band_key_enabled);
    EXPECT_EQ(disabled.phase131_canonical_rows, 0U);
    EXPECT_EQ(disabled.phase131_canonical_selected_streams, 0U);
    EXPECT_TRUE(disabled.phase131_failure_counts.empty());

    Diagnostics failed;
    Snapshot failure;
    failure.enabled = true;
    failure.configuration_valid = false;
    failure.configuration_failure = "synthetic canonical configuration failure";
    failure.failure_counts = {{"configuration", 1U}};
    ASSERT_TRUE(failed.synchronizePhase131Diagnostics(failure));
    EXPECT_TRUE(failed.phase131_canonical_correction_band_key_enabled);
    EXPECT_FALSE(failed.phase131_configuration_valid);
    EXPECT_EQ(failed.phase131_configuration_failure,
              failure.configuration_failure);
    EXPECT_EQ(failed.phase131_canonical_rows, 0U);
    EXPECT_EQ(failed.phase131_canonicalization_attempt_rows, 0U);
    EXPECT_EQ(failed.phase131_resolver_call_count, 0U);
    EXPECT_EQ(failed.phase131_failure_counts, failure.failure_counts);
}
