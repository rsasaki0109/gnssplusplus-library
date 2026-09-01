"""Structural, truth-free contracts for the Phase41 Doppler audit lane."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_android_rate_is_retained_as_diagnostic_source_value():
    observation = _read("include/libgnss++/core/observation.hpp")
    loader = _read("src/io/android_raw_gnss.cpp")

    assert "double pseudorange_rate_mps = 0.0" in observation
    assert "bool has_pseudorange_rate_mps = false" in observation
    assert "observation.pseudorange_rate_mps = raw.pseudorange_rate_mps" in loader
    assert "observation.has_pseudorange_rate_mps =" in loader
    assert "observation.doppler = -raw.pseudorange_rate_mps / wavelength" in loader


def test_estimators_do_not_consume_the_diagnostic_raw_rate_field():
    spp_velocity = _read("src/algorithms/spp_velocity.cpp")
    fgo = _read("src/algorithms/fgo_problems.cpp")

    # The only production consumer of the source-domain value is the Android
    # adapter.  SPP/FGO must consume the adapter's RINEX-convention Doppler Hz
    # field so this audit cannot silently change estimator semantics.
    assert "pseudorange_rate_mps" not in spp_velocity
    assert "pseudorange_rate_mps" not in fgo


def test_audit_is_raw_only_and_keeps_contract_equations_explicit():
    audit = _read("apps/native/gnss_doppler_contract_audit.cpp")
    cmake = _read("apps/CMakeLists.txt")
    raw_test = _read("tests/test_android_raw_gnss.cpp")

    assert "gnss_doppler_contract_audit" in cmake
    assert "verify_enriched_pseudorange = false" in audit
    assert "const Vector3d raw_seed(kInitialRawSeedX, kInitialRawSeedY" in audit
    assert "device_wls_coordinates_used" in audit
    assert "truth_used" in audit
    assert "rinexDopplerToRangeRate" in audit
    assert "receiverOnlyResidual" in audit
    assert "earthRotationCorrectedSatelliteState" in audit
    assert "EXPECT_TRUE(observation.has_pseudorange_rate_mps)" in raw_test
    assert "EXPECT_NEAR(observation.pseudorange_rate_mps," in raw_test
