#include <gtest/gtest.h>

#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/fgo_config.hpp>
#include <libgnss++/algorithms/phase126_raw_base_compound.hpp>
#include <libgnss++/algorithms/source_pseudorange_miss_mask.hpp>
#include <libgnss++/algorithms/source_transmission_clock.hpp>
#include <libgnss++/algorithms/source_tracking_selection.hpp>
#include <libgnss++/algorithms/source_transmission_time.hpp>
#include <libgnss++/algorithms/source_ephemeris_selection.hpp>
#include <libgnss++/algorithms/source_epoch_states.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/rinex.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace {

std::string headerLine(const std::string& content, const std::string& label) {
    std::string line = content;
    if (line.size() < 60U) line.append(60U - line.size(), ' ');
    line += label;
    return line + "\n";
}

std::string observationField(const std::string& value) {
    std::string field;
    if (value.size() < 14U) field.append(14U - value.size(), ' ');
    field += value;
    field += "  ";
    return field;
}

void writeAdditionalBandFixture(const std::filesystem::path& path) {
    std::ofstream file(path);
    ASSERT_TRUE(file.is_open());
    file << headerLine("     3.04           OBSERVATION DATA    G                   ",
                       "RINEX VERSION / TYPE");
    file << headerLine("G    6 C1C L1C C2W L2W C5Q L5Q", "SYS / # / OBS TYPES");
    file << headerLine("", "END OF HEADER");
    file << "> 2024 08 03 09 51 20.0000000  0  1\n";
    file << "G01"
         << observationField("22000000.000")
         << observationField("110000000.000")
         << observationField("22000010.000")
         << observationField("110000010.000")
         << observationField("22000020.000")
         << observationField("110000020.000") << "\n";
}

void writeSourceCompleteHeaderFixture(const std::filesystem::path& path) {
    std::ofstream file(path);
    ASSERT_TRUE(file.is_open());
    file << headerLine("     3.04           OBSERVATION DATA    G                   ",
                       "RINEX VERSION / TYPE");
    file << headerLine("   6378137.0000          0.0000          0.0000                  ",
                       "APPROX POSITION XYZ");
    file << headerLine("        1.2000        0.3000        0.4000                  ",
                       "ANTENNA: DELTA H/E/N");
    file << headerLine("", "END OF HEADER");
}

}  // namespace

using namespace libgnss;
using namespace libgnss::base_pseudorange_compensation;

TEST(BasePseudorangeCompensationTest, RinexGpsCalendarRemainsGpsWithoutUtcLeapShift) {
    const auto path = std::filesystem::temp_directory_path() / "gnss_base_gps_calendar_test.obs";
    {
        std::ofstream file(path);
        ASSERT_TRUE(file.is_open());
        file << headerLine("     3.03           OBSERVATION DATA    G", "RINEX VERSION / TYPE");
        file << headerLine("G    1 C1C", "SYS / # / OBS TYPES");
        file << headerLine("  2021    08    24    20    29   58.0000000     GPS", "TIME OF FIRST OBS");
        file << headerLine("", "END OF HEADER");
        file << "> 2021 08 24 20 29 58.0000000  0  1\n";
        file << "G01" << observationField("22000000.000") << "\n";
    }
    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    ObservationData epoch;
    const bool read = reader.readObservationEpoch(epoch);
    reader.close();
    std::filesystem::remove(path);
    ASSERT_TRUE(read);
    EXPECT_EQ(epoch.time.week, 2172);
    EXPECT_DOUBLE_EQ(epoch.time.tow, 246598.0);
    EXPECT_DOUBLE_EQ(header.first_obs.tow, epoch.time.tow);
    EXPECT_DOUBLE_EQ(epoch.receiver_clock_bias, 0.0);
}

TEST(BasePseudorangeCompensationTest, BroadcastSatelliteClockDoesNotApplyExplicitGroupDelay) {
    Ephemeris eph;
    eph.valid = true;
    eph.satellite = SatelliteId(GNSSSystem::GPS,1);
    eph.toe = eph.toc = GNSSTime(2200,100000);
    eph.toes = eph.toe.tow;
    eph.sqrt_a = std::sqrt(26560000.0);
    eph.e = 0.01; eph.i0 = 0.95; eph.m0 = 0.3;
    eph.af0 = 1e-4; eph.af1 = 2e-12;
    Vector3d p1,v1,p2,v2;
    double c1,d1,c2,d2;
    ASSERT_TRUE(eph.calculateSatelliteState(eph.toe+30,p1,v1,c1,d1));
    eph.tgd = 1e-7; eph.tgd_secondary = -2e-7;
    ASSERT_TRUE(eph.calculateSatelliteState(eph.toe+30,p2,v2,c2,d2));
    EXPECT_DOUBLE_EQ(c1,c2);
    EXPECT_DOUBLE_EQ(d1,d2);
    EXPECT_TRUE(p1.isApprox(p2,0));
}

TEST(BasePseudorangeCompensationTest, RoverCodeMaskMustPrecedeSharedTransmissionSelection) {
    using namespace source_transmission_clock;
    Observation l1; l1.valid = l1.has_pseudorange = true;
    l1.satellite = SatelliteId(GNSSSystem::GPS,1);
    l1.signal = SignalType::GPS_L1CA;
    l1.pseudorange_observation_type = "C1C"; l1.pseudorange = 20000000;
    auto l5 = l1; l5.signal = SignalType::GPS_L5;
    l5.pseudorange_observation_type = "C5I"; l5.pseudorange = 20000300;
    const std::vector<Observation> original{l1,l5};
    auto masked = original;
    masked[0].has_pseudorange = false;
    const auto before = selectNativeEpoch(original).at(l1.satellite);
    const auto after = selectNativeEpoch(masked).at(l1.satellite);
    EXPECT_EQ(before.slot, Slot::L1);
    EXPECT_EQ(after.slot, Slot::L5);
    EXPECT_DOUBLE_EQ(after.metres-before.metres,300.0);
    EXPECT_TRUE(original[0].has_pseudorange);
    masked[1].has_pseudorange = false;
    EXPECT_TRUE(selectNativeEpoch(masked).empty());
}

TEST(BasePseudorangeCompensationTest, SourceEpochStateModelBuildsAndClearsOnLaterFailure) {
    Config config; config.source_complete = config.use_source_epoch_states = true;
    config.base_position_ecef = Vector3d(6378137,0,0);
    config.approximate_position_present = config.station_reference_verified = true;
    config.antenna_reference_is_approx_position = true;
    config.expected_interval_s = 1; config.moving_mean_samples = 151;
    config.use_ionosphere_model = config.use_troposphere_model = false;
    NavigationData nav;
    Ephemeris eph; eph.valid = true; eph.satellite = SatelliteId(GNSSSystem::SBAS,1);
    eph.toe = GNSSTime(2200,100);
    eph.glonass_position = Vector3d(26000000,0,0);
    eph.glonass_velocity.setZero(); eph.glonass_acceleration.setZero();
    nav.addEphemeris(eph);
    Observation row; row.valid = row.has_pseudorange = true;
    row.satellite = eph.satellite; row.signal = SignalType::GPS_L1CA;
    row.pseudorange_observation_type = "C1C"; row.pseudorange = 20000000;
    ObservationSeries series;
    for (int i=0;i<3;++i) {
        ObservationData epoch; epoch.time = eph.toe+10+i;
        epoch.observations.push_back(row); series.addEpoch(epoch);
    }
    Model model;
    ASSERT_TRUE(model.build(series,nav,config)) << model.diagnostics().failure;
    EXPECT_EQ(model.diagnostics().source_epoch_states_built,3U);
    GNSSTime first, last;
    ASSERT_TRUE(model.streamTimeDomain(row.satellite,row.signal,first,last));
    EXPECT_DOUBLE_EQ(first.tow,110.0);
    EXPECT_DOUBLE_EQ(last.tow,112.0);
    double correction;
    EXPECT_TRUE(model.correctionAt(first,row.satellite,row.signal,correction));
    EXPECT_TRUE(model.correctionAt(last,row.satellite,row.signal,correction));
    EXPECT_FALSE(model.correctionAt(first-0.001,row.satellite,row.signal,correction));
    EXPECT_FALSE(model.correctionAt(last+0.001,row.satellite,row.signal,correction));
    EXPECT_TRUE(model.hasStream(row.satellite,row.signal));
    // Source FTYPE selects slots 0/2, not every preserved native band.
    // SBAS band 5 occupies slot 1 and must not form a correction stream.
    auto other = row;
    other.signal = SignalType::GPS_L5;
    other.pseudorange_observation_type = "C5I";
    for (auto& epoch : series.epochs) epoch.observations.push_back(other);
    config.use_source_fgo_frequency_slots = true;
    ASSERT_TRUE(model.build(series,nav,config)) << model.diagnostics().failure;
    EXPECT_EQ(model.diagnostics().source_frequency_rows_excluded,3U);
    EXPECT_TRUE(model.hasStream(row.satellite,row.signal));
    EXPECT_FALSE(model.hasStream(other.satellite,other.signal));
    series.epochs.back().observations[0].satellite.prn = 2;
    EXPECT_FALSE(model.build(series,nav,config));
    EXPECT_FALSE(model.hasStream(row.satellite,row.signal));
    EXPECT_EQ(model.diagnostics().source_epoch_states_built,2U);
    EXPECT_FALSE(model.streamTimeDomain(row.satellite,row.signal,first,last));
}

TEST(BasePseudorangeCompensationTest, SourceEpochStateModelRejectsMixedLegacyEquations) {
    Config config;
    EXPECT_FALSE(config.use_source_epoch_states);
    config.use_source_epoch_states = true;
    Model model;
    ObservationSeries observations;
    NavigationData nav;
    EXPECT_FALSE(model.build(observations, nav, config));
    EXPECT_TRUE(model.diagnostics().source_epoch_states_requested);
    EXPECT_EQ(model.diagnostics().source_epoch_states_built, 0U);
    EXPECT_EQ(model.diagnostics().failure,
              "source epoch states require source-complete base equations");
}

TEST(BasePseudorangeCompensationTest, EpochStateCompositionSelectsOnceAndRejectsIncompleteNavigation) {
    using namespace source_transmission_clock;
    NavigationData nav;
    Ephemeris eph; eph.valid = true;
    eph.satellite = SatelliteId(GNSSSystem::SBAS,1);
    eph.toe = GNSSTime(2200,100);
    eph.glonass_position = Vector3d(26000000,0,0);
    eph.glonass_velocity.setZero(); eph.glonass_acceleration.setZero();
    nav.addEphemeris(eph);
    Observation l1; l1.valid = l1.has_pseudorange = true;
    l1.satellite = eph.satellite; l1.signal = SignalType::GPS_L1CA;
    l1.pseudorange_observation_type = "C1C"; l1.pseudorange = 20000000;
    auto l5 = l1; l5.pseudorange_observation_type = "C5I"; l5.pseudorange = 21000000;
    const auto result = buildEpochStates(eph.toe+1, {l5,l1}, nav);
    ASSERT_EQ(result.size(),1U);
    EXPECT_DOUBLE_EQ(result.at(eph.satellite).selected_pseudorange.metres,20000000);
    EXPECT_DOUBLE_EQ(result.at(eph.satellite).state.position_ecef.x(),26000000);
    auto missing = l1; missing.satellite.prn = 2;
    EXPECT_THROW(buildEpochStates(eph.toe+1, {l1,missing}, nav),std::invalid_argument);
    EXPECT_EQ(nav.ephemeris_data.size(),1U);
    EXPECT_EQ(result.size(),1U);
    EXPECT_THROW(buildEpochStates(eph.toe+1, {l1,l1}, nav),std::invalid_argument);
}

TEST(BasePseudorangeCompensationTest, SourceBroadcastSelectionLimitsTiesAndGalileoRules) {
    using namespace source_transmission_clock;
    const GNSSTime time(2200, 30000);
    const std::vector<std::pair<GNSSSystem,double>> limits{
        {GNSSSystem::GPS,7201},{GNSSSystem::QZSS,7201},{GNSSSystem::NavIC,7201},
        {GNSSSystem::Galileo,14400},{GNSSSystem::BeiDou,21601},
        {GNSSSystem::GLONASS,1800},{GNSSSystem::SBAS,360}};
    for (const auto& [system, limit] : limits) {
        Ephemeris eph; eph.valid = true; eph.satellite = SatelliteId(system,1);
        eph.toe = time-limit; eph.data_source_code = 1 << 9;
        std::vector<Ephemeris> records{eph};
        EXPECT_EQ(selectBroadcastMessage(records,eph.satellite,time), &records[0]);
        records[0].toe = time-limit-0.01;
        EXPECT_EQ(selectBroadcastMessage(records,eph.satellite,time), nullptr);
        records = {eph,eph};
        EXPECT_EQ(selectBroadcastMessage(records,eph.satellite,time), &records[1]);
    }
    Ephemeris gal; gal.valid = true; gal.satellite = SatelliteId(GNSSSystem::Galileo,1);
    gal.toe = time; gal.data_source_code = 1<<9;
    std::vector<Ephemeris> records{gal};
    EXPECT_EQ(selectBroadcastMessage(records,gal.satellite,time), nullptr);
    records[0].toe = time-1;
    records[0].data_source_code = 1<<8;
    EXPECT_EQ(selectBroadcastMessage(records,gal.satellite,time), nullptr);
    records[0].data_source_code = 1<<9;
    EXPECT_EQ(selectBroadcastMessage(records,gal.satellite,time), &records[0]);
}

TEST(BasePseudorangeCompensationTest, SelectionTimeBoundaryDoesNotReselectDuringPropagation) {
    using namespace source_transmission_clock;
    NavigationData nav;
    Ephemeris older;
    older.valid = true;
    older.satellite = SatelliteId(GNSSSystem::SBAS, 1);
    older.toe = GNSSTime(2200, 100.0);
    older.toc = older.toe;
    older.glonass_position = Vector3d(26000000, 0, 0);
    older.glonass_velocity.setZero();
    older.glonass_acceleration.setZero();
    auto newer = older;
    newer.toe = GNSSTime(2200, 102.0);
    newer.toc = newer.toe;
    newer.glonass_position.x() += 100.0;
    nav.addEphemeris(older);
    nav.addEphemeris(newer);
    const GNSSTime receive(2200, 101.05);
    const double range = constants::SPEED_OF_LIGHT*0.1;
    const auto* receive_selected = nav.getEphemeris(older.satellite, receive);
    const auto* transmit_selected = nav.getEphemeris(older.satellite, receive-0.1);
    ASSERT_NE(receive_selected, nullptr);
    ASSERT_NE(transmit_selected, nullptr);
    EXPECT_NE(receive_selected, transmit_selected);
    EXPECT_DOUBLE_EQ(receive_selected->toe.tow, 102.0);
    EXPECT_DOUBLE_EQ(transmit_selected->toe.tow, 100.0);
    const auto state = stateFromSelectedEphemeris(receive, range, *receive_selected);
    EXPECT_DOUBLE_EQ(state.position_ecef.x(), 26000100.0);
    EXPECT_TRUE(state.velocity_ecef.isZero());
    // Reselecting at transmit time would use the other message and shift
    // this synthetic state by 100 m. This is not a measured real-data error.
    const auto other = stateFromSelectedEphemeris(receive, range, *transmit_selected);
    EXPECT_DOUBLE_EQ(state.position_ecef.x()-other.position_ecef.x(), 100.0);
}

TEST(BasePseudorangeCompensationTest, SelectedEphemerisStateUsesForwardMillisecondDifference) {
    using namespace source_transmission_clock;
    Ephemeris eph;
    eph.valid = true;
    eph.satellite = SatelliteId(GNSSSystem::SBAS, 1);
    eph.toe = GNSSTime(2200, 100);
    eph.glonass_position = Vector3d(26000000, 0, 0);
    eph.glonass_velocity = Vector3d(0, 100, 0);
    eph.glonass_acceleration = Vector3d(2, 0, 0);
    const auto result = stateFromSelectedEphemeris(
        eph.toe+1.1, constants::SPEED_OF_LIGHT*0.1, eph);
    EXPECT_NEAR(result.transmit_time-eph.toe, 1.0, 1e-10);
    EXPECT_NEAR(result.position_ecef.x(), 26000001.0, 1e-7);
    // Forward derivative includes half the acceleration times the step.
    EXPECT_NEAR(result.velocity_ecef.x(), 2.001, 5e-6);
    EXPECT_NEAR(result.velocity_ecef.y(), 100.0, 1e-6);
    EXPECT_DOUBLE_EQ(result.clock_seconds, 0.0);
    EXPECT_DOUBLE_EQ(result.clock_drift, 0.0);
    eph.glonass_position.x() = NAN;
    EXPECT_THROW(stateFromSelectedEphemeris(eph.toe+1.1, 20000000.0, eph),
                 std::invalid_argument);
}

TEST(BasePseudorangeCompensationTest, SourceTransmissionTimeUsesConstellationClockAndWeekRollover) {
    using namespace source_transmission_clock;
    Ephemeris eph;
    eph.valid = true;
    eph.satellite = SatelliteId(GNSSSystem::GPS, 1);
    eph.toc = GNSSTime(2200, 0.0);
    eph.toe = eph.toc;
    eph.af0 = 0.001;
    const auto time = transmissionTime(GNSSTime(2200, 0.05),
                                       constants::SPEED_OF_LIGHT * 0.1, eph);
    EXPECT_EQ(time.week, 2199);
    EXPECT_NEAR(time.tow, 604799.949, 1e-8);
    eph.satellite.system = GNSSSystem::GLONASS;
    eph.glonass_taun = 0.002;
    EXPECT_DOUBLE_EQ(initialClockSeconds(eph, eph.toe), -0.002);
    eph.satellite.system = GNSSSystem::SBAS;
    eph.af0 = 1.0; eph.af1 = 0.1;
    EXPECT_DOUBLE_EQ(initialClockSeconds(eph, eph.toe+10.0), 2.0);
    eph.valid = false;
    EXPECT_THROW(initialClockSeconds(eph, eph.toe), std::invalid_argument);
    EXPECT_THROW(transmissionTime(GNSSTime(2200, 0), 0.0, eph), std::invalid_argument);
}

TEST(BasePseudorangeCompensationTest, SourceHeaderFilterSelectsHDeclaredGpsL2WOverX) {
    const auto path = std::filesystem::temp_directory_path() / "gnss_phase298_l2_header.obs";
    {
        std::ofstream file(path);
        file << headerLine("     3.03           OBSERVATION DATA    G", "RINEX VERSION / TYPE")
             << headerLine("G    4 C2X L2X C2W L2W", "SYS / # / OBS TYPES")
             << headerLine("", "END OF HEADER");
        for (int epoch = 0; epoch < 2; ++epoch) {
            file << "> 2024 08 03 09 51 " << (20+epoch) << ".0000000  0  1\n"
                 << "G01" << observationField("22000000.000")
                 << observationField("110000000.000")
                 << observationField(epoch == 0 ? "22000012.000" : "")
                 << observationField(epoch == 0 ? "110000012.000" : "") << "\n";
        }
    }
    io::RINEXReader reader;
    reader.setSourceHeaderTrackingFilter(true);
    ASSERT_TRUE(reader.open(path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    ObservationData first, missing;
    ASSERT_TRUE(reader.readObservationEpoch(first));
    ASSERT_EQ(first.observations.size(), 1U);
    const auto& selected = first.observations.front();
    EXPECT_EQ(selected.pseudorange_observation_type, "C2W");
    EXPECT_EQ(selected.carrier_phase_observation_type, "L2W");
    EXPECT_DOUBLE_EQ(selected.pseudorange, 22000012.0);
    EXPECT_DOUBLE_EQ(selected.carrier_phase, 110000012.0);
    ASSERT_TRUE(reader.readObservationEpoch(missing));
    EXPECT_TRUE(missing.observations.empty());
    std::filesystem::remove(path);
}

TEST(BasePseudorangeCompensationTest, SourceHeaderFilterDoesNotFallbackWhenPreferredCodeIsMissing) {
    const auto path = std::filesystem::temp_directory_path() / "gnss_phase297_header_filter.obs";
    {
        std::ofstream file(path);
        file << headerLine("     3.04           OBSERVATION DATA    G", "RINEX VERSION / TYPE")
             << headerLine("G    2 C1C C1P", "SYS / # / OBS TYPES")
             << headerLine("", "END OF HEADER")
             << "> 2024 08 03 09 51 20.0000000  0  1\n"
             << "G01" << observationField("") << observationField("22000000.000") << "\n";
    }
    io::RINEXReader legacy, source;
    ASSERT_TRUE(legacy.open(path.string()));
    ASSERT_TRUE(source.open(path.string()));
    io::RINEXReader::RINEXHeader h;
    ASSERT_TRUE(legacy.readHeader(h));
    ASSERT_TRUE(source.readHeader(h));
    source.setSourceHeaderTrackingFilter(true);
    ObservationData old_epoch, source_epoch;
    ASSERT_TRUE(legacy.readObservationEpoch(old_epoch));
    ASSERT_TRUE(source.readObservationEpoch(source_epoch));
    EXPECT_FALSE(old_epoch.observations.empty());
    EXPECT_TRUE(source_epoch.observations.empty());
    std::filesystem::remove(path);
}

TEST(BasePseudorangeCompensationTest, SourceTrackingSelectionUsesHeaderPriorityAndStableTies) {
    using namespace source_transmission_clock;
    EXPECT_EQ(selectHeaderTrackingCodes(GNSSSystem::GPS, {"5I","5X","5Q"})
                  .at(Slot::L5), "5Q");
    EXPECT_EQ(selectHeaderTrackingCodes(GNSSSystem::GPS, {"5Q","5I"})
                  .at(Slot::L5), "5Q");
    EXPECT_EQ(selectHeaderTrackingCodes(GNSSSystem::BeiDou, {"1X","2X"})
                  .at(Slot::L1), "1X");
    EXPECT_EQ(selectHeaderTrackingCodes(GNSSSystem::BeiDou, {"2X","1X"})
                  .at(Slot::L1), "2X");
    EXPECT_EQ(defaultTrackingPriority(GNSSSystem::GPS, "1P"), 0);
    EXPECT_EQ(defaultTrackingPriority(GNSSSystem::GPS, "2P"), 14);
    EXPECT_EQ(defaultTrackingPriority(GNSSSystem::GPS, "5I"), 12);
    EXPECT_TRUE(selectHeaderTrackingCodes(GNSSSystem::UNKNOWN, {"1C"}).empty());
}

TEST(BasePseudorangeCompensationTest, NativeTransmissionAdapterRequiresCodeProvenance) {
    using namespace source_transmission_clock;
    Observation e5b;
    e5b.satellite = SatelliteId(GNSSSystem::Galileo, 1);
    e5b.signal = SignalType::GAL_E5B;
    e5b.valid = e5b.has_pseudorange = true;
    e5b.pseudorange = 21e6;
    e5b.pseudorange_observation_type = "C7Q";
    auto e5a = e5b;
    e5a.signal = SignalType::GAL_E5A;
    e5a.pseudorange = 22e6;
    e5a.pseudorange_observation_type = "C5Q";
    auto result = selectNativeEpoch({e5a, e5b});
    ASSERT_EQ(result.size(), 1U);
    EXPECT_EQ(result.at(e5b.satellite).slot, Slot::L2);
    EXPECT_DOUBLE_EQ(result.at(e5b.satellite).metres, 21e6);
    auto combined = e5b;
    combined.pseudorange_observation_type = "C8Q";
    EXPECT_EQ(selectNativeEpoch({combined}).at(e5b.satellite).slot, Slot::L7);
    auto missing = e5b;
    missing.pseudorange_observation_type.clear();
    EXPECT_THROW(selectNativeEpoch({missing}), std::invalid_argument);
    missing.valid = false;
    EXPECT_TRUE(selectNativeEpoch({missing}).empty());
    auto duplicate = e5b;
    duplicate.pseudorange_observation_type = "C7I";
    EXPECT_THROW(selectNativeEpoch({e5b, duplicate}), std::invalid_argument);
}

TEST(BasePseudorangeCompensationTest, SourceRinexBandMappingIsConstellationDependent) {
    using namespace source_transmission_clock;
    const std::array<GNSSSystem, 7> systems{GNSSSystem::GPS, GNSSSystem::GLONASS,
        GNSSSystem::Galileo, GNSSSystem::BeiDou, GNSSSystem::QZSS,
        GNSSSystem::SBAS, GNSSSystem::NavIC};
    // Columns are RINEX band digits 1..9; -1 means unsupported.
    const int expected[7][9] = {
        {0,1,-1,-1,2,-1,-1,-1,-1},
        {0,1,2,0,-1,1,-1,-1,-1},
        {0,-1,-1,-1,2,3,1,4,-1},
        {0,0,-1,-1,2,3,1,4,-1},
        {0,1,-1,-1,2,3,-1,-1,-1},
        {0,-1,-1,-1,1,-1,-1,-1,-1},
        {-1,-1,-1,-1,0,-1,-1,-1,1}};
    for (std::size_t system = 0; system < systems.size(); ++system) {
        for (int band = 1; band <= 9; ++band) {
            const auto slot = slotForRinexBand(systems[system], band);
            EXPECT_EQ(slot ? static_cast<int>(*slot) : -1, expected[system][band-1]);
        }
        EXPECT_FALSE(slotForRinexBand(systems[system], 0));
        EXPECT_FALSE(slotForRinexBand(systems[system], 10));
    }
    EXPECT_FALSE(slotForRinexBand(GNSSSystem::UNKNOWN, 1));
}

TEST(BasePseudorangeCompensationTest, SourceTransmissionGroupsSatellitesWithoutArrivalPriority) {
    using namespace source_transmission_clock;
    const SatelliteId gps(GNSSSystem::GPS, 1), gal(GNSSSystem::Galileo, 1);
    std::vector<SlottedPseudorange> rows{
        {gps, Slot::L5, 22e6}, {gal, Slot::L1, 23e6}, {gps, Slot::L1, 20e6}};
    const auto result = selectBySatellite(rows);
    ASSERT_EQ(result.size(), 2U);
    EXPECT_DOUBLE_EQ(result.at(gps).metres, 20e6);
    EXPECT_DOUBLE_EQ(result.at(gal).metres, 23e6);
    std::reverse(rows.begin(), rows.end());
    EXPECT_DOUBLE_EQ(selectBySatellite(rows).at(gps).metres, 20e6);
    rows.push_back({gps, Slot::L1, 20e6});
    EXPECT_THROW(selectBySatellite(rows), std::invalid_argument);
    EXPECT_EQ(rows.size(), 4U);
    EXPECT_THROW(selectBySatellite({{gps, static_cast<Slot>(7), 20e6}}),
                 std::invalid_argument);
    EXPECT_TRUE(selectBySatellite({{gps, Slot::L1, NAN}}).empty());
}

TEST(BasePseudorangeCompensationTest, SourceTransmissionClockUsesTwoPolynomialIterations) {
    using source_transmission_clock::polynomialClockSeconds;
    EXPECT_DOUBLE_EQ(polynomialClockSeconds(123.0, 0.001, 0.0, 0.0), 0.001);
    // Deliberately large synthetic coefficients make iteration count visible:
    // t0=10; t1=10-(1+.1*10)=8; t2=10-(1+.1*8)=8.2.
    EXPECT_DOUBLE_EQ(polynomialClockSeconds(10.0, 1.0, 0.1, 0.0), 1.82);
    // Quadratic: t0=2; t1=2-.25*4=1; t2=2-.25=1.75.
    EXPECT_DOUBLE_EQ(polynomialClockSeconds(2.0, 0.0, 0.0, 0.25), 0.765625);
    EXPECT_THROW(polynomialClockSeconds(NAN, 0.0, 0.0, 0.0), std::invalid_argument);
    EXPECT_THROW(polynomialClockSeconds(1.0, INFINITY, 0.0, 0.0), std::invalid_argument);
    EXPECT_THROW(polynomialClockSeconds(1e300, 0.0, 0.0, 1e300), std::invalid_argument);
}

TEST(BasePseudorangeCompensationTest, SourceTransmissionPseudorangeUsesFrequencySlotOrder) {
    using namespace source_transmission_clock;
    std::array<double, 7> slots{20000000.0, 21000000.0, 22000000.0, 0, 0, 0, 0};
    auto selected = selectPseudorange(slots);
    ASSERT_TRUE(selected);
    EXPECT_EQ(selected->slot, Slot::L1);
    EXPECT_DOUBLE_EQ(selected->metres, slots[0]);
    slots[0] = NAN;
    selected = selectPseudorange(slots);
    ASSERT_TRUE(selected);
    EXPECT_EQ(selected->slot, Slot::L2);
    slots[1] = -0.0;
    selected = selectPseudorange(slots);
    ASSERT_TRUE(selected);
    EXPECT_EQ(selected->slot, Slot::L5);
    EXPECT_DOUBLE_EQ(selected->metres, 22000000.0);
    slots[0] = -1.0;
    EXPECT_DOUBLE_EQ(selectPseudorange(slots)->metres, -1.0);
    slots[0] = INFINITY;
    EXPECT_THROW(selectPseudorange(slots), std::invalid_argument);
    slots.fill(0.0);
    EXPECT_FALSE(selectPseudorange(slots));
    slots.fill(NAN);
    EXPECT_FALSE(selectPseudorange(slots));
    slots[6] = 23000000.0;
    selected = selectPseudorange(slots);
    ASSERT_TRUE(selected);
    EXPECT_EQ(selected->slot, Slot::L9);
}

TEST(BasePseudorangeCompensationTest, CenteredMovingMeanShrinksAtEdges) {
    const auto result = centeredMovingMean({1.0, 2.0, 3.0, 4.0, 5.0}, 3U);
    ASSERT_EQ(result.size(), 5U);
    EXPECT_DOUBLE_EQ(result[0], 1.5);
    EXPECT_DOUBLE_EQ(result[1], 2.0);
    EXPECT_DOUBLE_EQ(result[2], 3.0);
    EXPECT_DOUBLE_EQ(result[3], 4.0);
    EXPECT_DOUBLE_EQ(result[4], 4.5);
}

TEST(BasePseudorangeCompensationTest, MovingMeanOmitsNonfiniteSamples) {
    const auto result = centeredMovingMean(
        {1.0, std::numeric_limits<double>::quiet_NaN(), 3.0}, 3U);
    ASSERT_EQ(result.size(), 3U);
    EXPECT_DOUBLE_EQ(result[0], 1.0);
    EXPECT_DOUBLE_EQ(result[1], 2.0);
    EXPECT_DOUBLE_EQ(result[2], 3.0);
}

TEST(BasePseudorangeCompensationTest, SourceSignSubtractsPositiveCorrection) {
    EXPECT_DOUBLE_EQ(subtractCorrection(2'000'000.0, 12.5), 1'999'987.5);
    EXPECT_DOUBLE_EQ(subtractCorrection(2'000'000.0, -12.5), 2'000'012.5);
}

TEST(BasePseudorangeCompensationTest, Phase126OfficialResidualIsMetreValued) {
    EXPECT_DOUBLE_EQ(
        phase126_raw_base::officialBaseCodeResidual(
            20'000'000.0, 12.0, 19'999'500.0, 4.0, 6.0),
        502.0);
    EXPECT_DOUBLE_EQ(phase126_raw_base::officialExplicitCodeBiasMeters(), 0.0);
}

TEST(BasePseudorangeCompensationTest, SharedCodeBiasCancelsOnlyWhenSmoothedBaseBiasMatches) {
    // Fixed synthetic raw residuals isolate the explicit code-bias operator.
    // No orbital, atmospheric, seed or observation-admission differences.
    const double rover_without_bias = 20000000.0;
    const std::vector<double> base_without_bias{100.0, 100.0, 100.0};
    const auto source_pc = centeredMovingMean(base_without_bias, 3U);
    const auto constant_pc = centeredMovingMean({94.0, 94.0, 94.0}, 3U);
    EXPECT_DOUBLE_EQ(subtractCorrection(rover_without_bias - 6.0, constant_pc[1]),
                     subtractCorrection(rover_without_bias, source_pc[1]));

    // An ephemeris/code-bias transition falls inside the base smoothing
    // window. The rover uses 9 m now, but the base window averages 6 m.
    const auto transition_pc = centeredMovingMean({97.0, 94.0, 91.0}, 3U);
    const double native = subtractCorrection(rover_without_bias - 9.0, transition_pc[1]);
    const double source = subtractCorrection(rover_without_bias, source_pc[1]);
    EXPECT_DOUBLE_EQ(native - source, -3.0); // mean(base bias) - rover bias
    EXPECT_NE(native, source);
    // Changing only the base convention leaves the whole rover bias behind.
    EXPECT_DOUBLE_EQ(subtractCorrection(rover_without_bias - 9.0, source_pc[1])
                         - source, -9.0);
}

TEST(BasePseudorangeCompensationTest, Phase126GeodistAppliesOneSagnacTerm) {
    const Vector3d receiver(6'378'137.0, 0.0, 0.0);
    const Vector3d satellite(0.0, 26'560'000.0, 0.0);
    const auto geometry = phase126_raw_base::geodistWithSagnac(receiver, satellite);
    const double raw_range = (satellite - receiver).norm();
    const double expected = raw_range +
                            constants::OMEGA_E / constants::SPEED_OF_LIGHT *
                                (satellite.x() * receiver.y() -
                                 satellite.y() * receiver.x());
    EXPECT_DOUBLE_EQ(geometry.range_m, expected);
    EXPECT_TRUE(geometry.line_of_sight.allFinite());
    EXPECT_TRUE(std::isfinite(geometry.elevation_rad));
    EXPECT_TRUE(std::isfinite(geometry.azimuth_rad));
}

TEST(BasePseudorangeCompensationTest, Phase126StationReferenceRequiresProof) {
    phase126_raw_base::StationReference reference;
    reference.approximate_position_ecef = Vector3d(6'378'137.0, 0.0, 0.0);
    reference.has_approximate_position = true;
    std::string failure;
    EXPECT_FALSE(phase126_raw_base::validateStationReference(reference, failure));
    EXPECT_EQ(failure, "RINEX antenna reference convention is unproven");
    reference.antenna_reference_convention_proven = true;
    EXPECT_TRUE(phase126_raw_base::validateStationReference(reference, failure));
    EXPECT_TRUE(phase126_raw_base::antennaReferenceEcef(reference, false).allFinite());
}

TEST(BasePseudorangeCompensationTest, Phase126SelectorIsDefaultOffEverywhere) {
    EXPECT_FALSE(Config{}.source_complete);
    EXPECT_FALSE(libgnss::fgo::Config{}.use_native_phase126_raw_base_source_complete);
    EXPECT_FALSE(libgnss::SPPProcessor::SPPConfig{}.use_official_no_explicit_code_bias);
}

TEST(BasePseudorangeCompensationTest, InvalidBuildFailsClosedWithoutPayloadIo) {
    Model model;
    ObservationSeries empty;
    NavigationData nav;
    Config config;
    config.base_position_ecef = Vector3d(1.0, 2.0, 3.0);
    config.expected_interval_s = 1.0;
    config.moving_mean_samples = 151U;
    EXPECT_FALSE(model.build(empty, nav, config));
    EXPECT_FALSE(model.diagnostics().built);
    EXPECT_FALSE(model.diagnostics().failure.empty());
}

TEST(BasePseudorangeCompensationTest,
     Phase126ModelRejectsUnprovenHeaderPresenceBeforeStateConstruction) {
    Model model;
    ObservationSeries empty;
    NavigationData nav;
    Config config;
    config.source_complete = true;
    config.base_position_ecef = Vector3d(6'378'137.0, 0.0, 0.0);
    config.expected_interval_s = 1.0;
    config.moving_mean_samples = 151U;
    config.station_reference_verified = true;
    config.antenna_reference_is_approx_position = true;
    EXPECT_FALSE(model.build(empty, nav, config));
    EXPECT_EQ(model.diagnostics().failure,
              "RINEX header lacks APPROX POSITION XYZ");
}

TEST(BasePseudorangeCompensationTest,
     Phase126ModelRejectsRouteWindowMismatchBeforeBuildingStreams) {
    Model model;
    ObservationSeries empty;
    NavigationData nav;
    Config config;
    config.source_complete = true;
    config.base_position_ecef = Vector3d(6'378'137.0, 0.0, 0.0);
    config.approximate_position_present = true;
    config.station_reference_verified = true;
    config.antenna_reference_is_approx_position = true;
    config.expected_interval_s = 1.0;
    config.moving_mean_samples = 11U;
    EXPECT_FALSE(model.build(empty, nav, config));
    EXPECT_EQ(model.diagnostics().failure,
              "source-complete moving-mean window does not match frozen route");
    EXPECT_FALSE(model.hasStream(SatelliteId(GNSSSystem::GPS, 1),
                                 SignalType::GPS_L1CA));
}

TEST(BasePseudorangeCompensationTest, AdditionalBandsAreExplicitlyOptIn) {
    io::RINEXReader reader;
    EXPECT_FALSE(reader.preservesAdditionalFrequencyBands());
    reader.setPreserveAdditionalFrequencyBands(true);
    EXPECT_TRUE(reader.preservesAdditionalFrequencyBands());
    reader.setPreserveAdditionalFrequencyBands(false);
    EXPECT_FALSE(reader.preservesAdditionalFrequencyBands());
}

TEST(BasePseudorangeCompensationTest, Phase126HeaderPresenceAndEnuSemanticsAreExplicit) {
    const auto path = std::filesystem::temp_directory_path() /
                      "libgnss_phase126_header_fixture.obs";
    std::filesystem::remove(path);
    writeSourceCompleteHeaderFixture(path);
    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    EXPECT_TRUE(header.has_approximate_position);
    EXPECT_TRUE(header.has_antenna_delta);
    EXPECT_DOUBLE_EQ(header.approximate_position.x(), 6378137.0);
    // Parser order is the source RINEX H/E/N record normalized to E/N/H.
    EXPECT_DOUBLE_EQ(header.antenna_delta.x(), 0.3);
    EXPECT_DOUBLE_EQ(header.antenna_delta.y(), 0.4);
    EXPECT_DOUBLE_EQ(header.antenna_delta.z(), 1.2);
    reader.close();
    std::filesystem::remove(path);
}

TEST(BasePseudorangeCompensationTest, AdditionalBandFixtureAddsGpsL5OnlyWhenEnabled) {
    const auto path = std::filesystem::temp_directory_path() /
                      "libgnss_phase71_additional_band_fixture.obs";
    std::filesystem::remove(path);
    writeAdditionalBandFixture(path);

    io::RINEXReader default_reader;
    ASSERT_TRUE(default_reader.open(path.string()));
    io::RINEXReader::RINEXHeader default_header;
    ASSERT_TRUE(default_reader.readHeader(default_header));
    ObservationData default_epoch;
    ASSERT_TRUE(default_reader.readObservationEpoch(default_epoch));
    EXPECT_EQ(default_epoch.observations.size(), 2U);
    EXPECT_NE(default_epoch.getObservation(
                  SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L1CA),
              nullptr);
    EXPECT_NE(default_epoch.getObservation(
                  SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L2C),
              nullptr);
    EXPECT_EQ(default_epoch.getObservation(
                  SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L5),
              nullptr);
    default_reader.close();

    io::RINEXReader preserved_reader;
    preserved_reader.setPreserveAdditionalFrequencyBands(true);
    ASSERT_TRUE(preserved_reader.open(path.string()));
    io::RINEXReader::RINEXHeader preserved_header;
    ASSERT_TRUE(preserved_reader.readHeader(preserved_header));
    ObservationData preserved_epoch;
    ASSERT_TRUE(preserved_reader.readObservationEpoch(preserved_epoch));
    EXPECT_EQ(preserved_epoch.observations.size(), 3U);
    EXPECT_NE(preserved_epoch.getObservation(
                  SatelliteId(GNSSSystem::GPS, 1), SignalType::GPS_L5),
              nullptr);
    preserved_reader.close();
    std::filesystem::remove(path);
}

TEST(BasePseudorangeCompensationTest,
     SourceExactMissMaskDropsOnlyNonfinitePcRowsAndPreservesEpochIndices) {
    using namespace libgnss::source_pseudorange_miss_mask;
    std::vector<FGOProcessor::EpochSeed> epochs(3);
    epochs[0].time = GNSSTime(2200, 100.0);
    epochs[1].time = GNSSTime(2200, 101.0);
    epochs[2].time = GNSSTime(2200, 102.0);

    const SatelliteId gps1(GNSSSystem::GPS, 1);
    const SatelliteId gps2(GNSSSystem::GPS, 2);
    FGOProcessor::PseudorangeFactor retained;
    retained.epoch_index = 2U;
    retained.satellite = gps1;
    retained.signal = SignalType::GPS_L1CA;
    retained.corrected_pseudorange_m = 100.0;
    FGOProcessor::PseudorangeFactor missing = retained;
    missing.epoch_index = 0U;
    missing.satellite = gps2;
    FGOProcessor::PseudorangeFactor outside = retained;
    outside.epoch_index = 1U;
    FGOProcessor::PseudorangeFactor nonfinite = retained;
    nonfinite.epoch_index = 0U;
    nonfinite.signal = SignalType::GPS_L5;
    std::vector<FGOProcessor::PseudorangeFactor> factors = {
        retained, missing, outside, nonfinite};

    Report report;
    ASSERT_TRUE(apply(
        factors, epochs,
        [gps1](const SatelliteId& satellite, SignalType) {
            return satellite == gps1;
        },
        [](const GNSSTime& time, const SatelliteId&, SignalType signal,
           double& correction) {
            if (time.tow == 101.0) return false;
            if (signal == SignalType::GPS_L5) {
                correction = std::numeric_limits<double>::quiet_NaN();
                return true;
            }
            correction = 2.0;
            return true;
        },
        report));

    ASSERT_EQ(factors.size(), 1U);
    EXPECT_EQ(factors.front().epoch_index, 2U);
    EXPECT_DOUBLE_EQ(factors.front().corrected_pseudorange_m, 98.0);
    EXPECT_EQ(report.original_adopted_rows, 4U);
    EXPECT_EQ(report.retained_finite_pc_rows, 1U);
    EXPECT_EQ(report.matched_exact_stream_rows, 3U);
    EXPECT_EQ(report.finite_correction_rows_among_matched, 1U);
    EXPECT_EQ(report.dropped_missing_exact_stream_rows, 1U);
    EXPECT_EQ(report.dropped_out_of_domain_rows, 1U);
    EXPECT_EQ(report.dropped_nonfinite_correction_rows, 1U);
    EXPECT_TRUE(report.factor_count_consistent);
    EXPECT_DOUBLE_EQ(report.retained_finite_pc_fraction, 1.0);
    EXPECT_DOUBLE_EQ(report.retained_over_original_fraction, 0.25);
    EXPECT_DOUBLE_EQ(report.correction_abs_p50_m, 2.0);
    EXPECT_DOUBLE_EQ(report.correction_abs_p95_m, 2.0);
    EXPECT_DOUBLE_EQ(report.correction_abs_max_m, 2.0);
}

TEST(BasePseudorangeCompensationTest, SourceExactMissMaskRejectsMissingCallbacksWithoutMutation) {
    using namespace libgnss::source_pseudorange_miss_mask;
    std::vector<FGOProcessor::EpochSeed> epochs(1);
    epochs[0].time = GNSSTime(2200, 100.0);
    FGOProcessor::PseudorangeFactor factor;
    factor.epoch_index = 0U;
    factor.corrected_pseudorange_m = 100.0;
    std::vector<FGOProcessor::PseudorangeFactor> factors = {factor};
    Report report;
    EXPECT_FALSE(apply(factors, epochs, HasStream{}, CorrectionAt{}, report));
    ASSERT_EQ(factors.size(), 1U);
    EXPECT_DOUBLE_EQ(factors.front().corrected_pseudorange_m, 100.0);
    EXPECT_FALSE(report.callback_contract_valid);
    EXPECT_EQ(report.original_adopted_rows, 1U);
}

TEST(BasePseudorangeCompensationTest, SourceCorrectionCallbackFailureDoesNotPartiallyCommit) {
    using namespace source_pseudorange_miss_mask;
    FGOProcessor::PseudorangeFactor first;
    first.satellite = SatelliteId(GNSSSystem::GPS, 1);
    first.corrected_pseudorange_m = 20000000.0;
    auto second = first;
    second.satellite = SatelliteId(GNSSSystem::GPS, 2);
    std::vector<FGOProcessor::PseudorangeFactor> factors{first, second};
    FGOProcessor::EpochSeed seed;
    seed.time = GNSSTime(2200, 100);
    std::vector<FGOProcessor::EpochSeed> epochs{seed};
    Report report;
    std::size_t calls = 0;
    EXPECT_THROW(apply(factors, epochs,
        [](const SatelliteId&, SignalType) { return true; },
        [&](const GNSSTime&, const SatelliteId&, SignalType, double& correction) {
            if (++calls == 2) throw std::runtime_error("synthetic callback failure");
            correction = 12.0;
            return true;
        }, report), std::runtime_error);
    EXPECT_EQ(calls, 2U);
    ASSERT_EQ(factors.size(), 2U);
    for (const auto& factor : factors) {
        EXPECT_DOUBLE_EQ(factor.corrected_pseudorange_m, 20000000.0);
        EXPECT_FALSE(factor.native_base_pseudorange_correction_applied);
    }
}

TEST(BasePseudorangeCompensationTest, MaskOnlyAblationPreservesMissingnessWithoutApplyingValues) {
    using namespace source_pseudorange_miss_mask;
    EXPECT_FALSE(finiteMaskOnlyAblation({}));
    CorrectionAt actual = [](const GNSSTime&, const SatelliteId& sat, SignalType, double& value) {
        value = sat.prn == 2 ? std::numeric_limits<double>::quiet_NaN() : 12.0;
        return sat.prn != 3;
    };
    FGOProcessor::EpochSeed seed;
    seed.time = GNSSTime(2200,100);
    std::vector<FGOProcessor::PseudorangeFactor> input(4);
    for (std::size_t i = 0; i < input.size(); ++i) {
        input[i].satellite = SatelliteId(GNSSSystem::GPS, i+1);
        input[i].corrected_pseudorange_m = 20000000.0;
    }
    const auto has = [](const SatelliteId& sat, SignalType) { return sat.prn != 4; };
    auto corrected = input;
    auto masked = input;
    Report real_report, mask_report;
    ASSERT_TRUE(apply(corrected, {seed}, has, actual, real_report));
    ASSERT_TRUE(apply(masked, {seed}, has, finiteMaskOnlyAblation(actual), mask_report));
    ASSERT_EQ(corrected.size(), 1U);
    ASSERT_EQ(masked.size(), corrected.size());
    EXPECT_TRUE(masked[0].satellite == corrected[0].satellite);
    EXPECT_DOUBLE_EQ(masked[0].corrected_pseudorange_m, input[0].corrected_pseudorange_m);
    EXPECT_DOUBLE_EQ(corrected[0].corrected_pseudorange_m, 19999988.0);
    EXPECT_EQ(mask_report.dropped_missing_exact_stream_rows, real_report.dropped_missing_exact_stream_rows);
    EXPECT_EQ(mask_report.dropped_out_of_domain_rows, real_report.dropped_out_of_domain_rows);
    EXPECT_EQ(mask_report.dropped_nonfinite_correction_rows, real_report.dropped_nonfinite_correction_rows);
    EXPECT_DOUBLE_EQ(mask_report.correction_abs_max_m, 0.0);
}

TEST(BasePseudorangeCompensationTest, GpsValuesOnlyPreservesFullCorrectionSupportAndWeights) {
    using namespace source_pseudorange_miss_mask;
    EXPECT_FALSE(gpsValuesOnlyAblation({}));
    CorrectionAt actual=[](const GNSSTime&,const SatelliteId& sat,SignalType,double& value) {
        value=sat.prn==2 ? std::numeric_limits<double>::quiet_NaN() : 12.0;
        return sat.prn!=3;
    };
    FGOProcessor::EpochSeed seed;seed.time=GNSSTime(2200,100);
    std::vector<FGOProcessor::PseudorangeFactor> input;
    for (auto system : {GNSSSystem::GPS,GNSSSystem::Galileo,GNSSSystem::GLONASS}) {
        for (int prn=1;prn<=4;++prn) {
            FGOProcessor::PseudorangeFactor factor;
            factor.satellite=SatelliteId(system,prn);
            factor.corrected_pseudorange_m=20000000.0;factor.sigma_m=3.5;
            input.push_back(factor);
        }
    }
    const auto has=[](const SatelliteId& sat,SignalType){return sat.prn!=4;};
    auto full=input,gps=input;
    Report a,b;
    ASSERT_TRUE(apply(full,{seed},has,actual,a));
    ASSERT_TRUE(apply(gps,{seed},has,gpsValuesOnlyAblation(actual),b));
    ASSERT_EQ(full.size(),3U);ASSERT_EQ(gps.size(),full.size());
    for (std::size_t i=0;i<gps.size();++i) {
        EXPECT_TRUE(gps[i].satellite==full[i].satellite);
        EXPECT_DOUBLE_EQ(gps[i].sigma_m,3.5);
        EXPECT_DOUBLE_EQ(gps[i].corrected_pseudorange_m,
            gps[i].satellite.system==GNSSSystem::GPS ? 19999988.0 : 20000000.0);
    }
    EXPECT_EQ(a.dropped_missing_exact_stream_rows,b.dropped_missing_exact_stream_rows);
    EXPECT_EQ(a.dropped_out_of_domain_rows,b.dropped_out_of_domain_rows);
    EXPECT_EQ(a.dropped_nonfinite_correction_rows,b.dropped_nonfinite_correction_rows);
}

TEST(BasePseudorangeCompensationTest, GpsCenteringPreservesMissesAndFailsTransactionally) {
    using namespace source_pseudorange_miss_mask;
    CorrectionAt actual=[](const GNSSTime&,const SatelliteId& sat,SignalType,double& value) {
        value=sat.prn==3 ? std::numeric_limits<double>::quiet_NaN() : 12.0;
        return sat.prn!=4;
    };
    StreamCenter center=[](const SatelliteId& sat,SignalType,double& value) {
        value=10;return sat.prn!=2;
    };
    EXPECT_FALSE(gpsCenteredValuesAblation({},center));
    EXPECT_FALSE(gpsCenteredValuesAblation(actual,{}));
    auto callback=gpsCenteredValuesAblation(actual,center);
    double value=0;const GNSSTime time(2200,100);
    ASSERT_TRUE(callback(time,{GNSSSystem::GPS,1},SignalType::GPS_L1CA,value));
    EXPECT_DOUBLE_EQ(value,2);
    ASSERT_TRUE(callback(time,{GNSSSystem::Galileo,2},SignalType::GPS_L1CA,value));
    EXPECT_DOUBLE_EQ(value,0);
    EXPECT_TRUE(callback(time,{GNSSSystem::GPS,3},SignalType::GPS_L1CA,value));
    EXPECT_TRUE(std::isnan(value));
    EXPECT_FALSE(callback(time,{GNSSSystem::GPS,4},SignalType::GPS_L1CA,value));
    FGOProcessor::EpochSeed seed;seed.time=time;
    std::vector<FGOProcessor::PseudorangeFactor> factors(2);
    for (std::size_t i=0;i<2;++i) {
        factors[i].satellite=SatelliteId(GNSSSystem::GPS,i+1);
        factors[i].corrected_pseudorange_m=20000000;
    }
    Report report;
    EXPECT_THROW(apply(factors,{seed},[](const SatelliteId&,SignalType){return true;},callback,report),std::invalid_argument);
    ASSERT_EQ(factors.size(),2U);
    for (const auto& factor:factors) {
        EXPECT_DOUBLE_EQ(factor.corrected_pseudorange_m,20000000);
        EXPECT_FALSE(factor.native_base_pseudorange_correction_applied);
    }
}

TEST(BasePseudorangeCompensationTest, RemovingMissingEpochsChangesMovingMeanWindow) {
    const double missing = std::numeric_limits<double>::quiet_NaN();
    const auto dense = centeredMovingMean({0.0, missing, missing, 90.0}, 3);
    const auto compact = centeredMovingMean({0.0, 90.0}, 3);
    ASSERT_EQ(dense.size(), 4U);
    ASSERT_EQ(compact.size(), 2U);
    EXPECT_DOUBLE_EQ(dense.front(), 0.0);
    EXPECT_DOUBLE_EQ(dense.back(), 90.0);
    EXPECT_DOUBLE_EQ(compact.front(), 45.0);
    EXPECT_DOUBLE_EQ(compact.back(), 45.0);
}

TEST(BasePseudorangeCompensationTest, DenseModelPreservesLongGapsAndExactFiniteBoundary) {
    Config c;
    c.source_complete = c.use_source_epoch_states = true;
    c.base_position_ecef = Vector3d(6378137, 0, 0);
    c.approximate_position_present = c.station_reference_verified = true;
    c.antenna_reference_is_approx_position = true;
    c.expected_interval_s = 1;
    c.moving_mean_samples = 151;
    c.use_ionosphere_model = c.use_troposphere_model = false;
    Ephemeris eph;
    eph.valid = true;
    eph.satellite = SatelliteId(GNSSSystem::SBAS, 1);
    eph.toe = GNSSTime(2200, 300);
    eph.glonass_position = Vector3d(26000000, 0, 0);
    eph.glonass_velocity.setZero();
    eph.glonass_acceleration.setZero();
    NavigationData nav;
    nav.addEphemeris(eph);
    Observation row;
    row.valid = row.has_pseudorange = true;
    row.satellite = eph.satellite;
    row.signal = SignalType::GPS_L1CA;
    row.pseudorange_observation_type = "C1C";
    row.pseudorange = 20000000;
    ObservationSeries series;
    for (int i = 0; i <= 400; ++i) {
        ObservationData epoch;
        epoch.time = GNSSTime(2200, 100 + i);
        if (i == 10 || i == 390) {
            auto sample = row;
            if (i == 390) sample.pseudorange += 90;
            epoch.observations.push_back(sample);
        }
        series.addEpoch(epoch);
    }
    Model compact, dense;
    ASSERT_TRUE(compact.build(series, nav, c));
    c.use_dense_epoch_smoothing = true;
    ASSERT_TRUE(dense.build(series, nav, c)) << dense.diagnostics().failure;
    double old_value, new_value;
    ASSERT_TRUE(compact.correctionAt(GNSSTime(2200,110), row.satellite,row.signal,old_value));
    ASSERT_TRUE(dense.correctionAt(GNSSTime(2200,110), row.satellite,row.signal,new_value));
    EXPECT_NEAR(old_value - new_value, 45.0, 1e-6);
    EXPECT_TRUE(compact.correctionAt(GNSSTime(2200,300), row.satellite,row.signal,old_value));
    EXPECT_FALSE(dense.correctionAt(GNSSTime(2200,300), row.satellite,row.signal,new_value));
    EXPECT_FALSE(dense.correctionAt(GNSSTime(2200,414.5), row.satellite,row.signal,new_value));
    EXPECT_TRUE(dense.correctionAt(GNSSTime(2200,415), row.satellite,row.signal,new_value));
    EXPECT_TRUE(dense.correctionAt(GNSSTime(2200,100), row.satellite,row.signal,new_value));
    EXPECT_TRUE(dense.correctionAt(GNSSTime(2200,500), row.satellite,row.signal,new_value));
    EXPECT_FALSE(dense.correctionAt(GNSSTime(2200,99.999), row.satellite,row.signal,new_value));
    double median_value=-123;
    ASSERT_TRUE(dense.streamMedian(row.satellite,row.signal,median_value));
    EXPECT_NEAR(median_value,old_value,1e-6); // symmetric finite arcs, omit long NaN gap
    EXPECT_FALSE(dense.streamMedian(row.satellite,SignalType::GPS_L5,median_value));
    EXPECT_NEAR(median_value,old_value,1e-6);
    c.use_source_epoch_states = false;
    EXPECT_FALSE(dense.build(series, nav, c));
    EXPECT_FALSE(dense.hasStream(row.satellite, row.signal));
    EXPECT_FALSE(dense.streamMedian(row.satellite,row.signal,median_value));
    EXPECT_NEAR(median_value,old_value,1e-6);
}

TEST(BasePseudorangeCompensationTest,
     SourceExactMissMaskReportsSignalTaxonomyAndSinglePass) {
    using namespace libgnss::source_pseudorange_miss_mask;
    std::vector<FGOProcessor::EpochSeed> epochs(3);
    epochs[0].time = GNSSTime(2200, 100.0);
    epochs[1].time = GNSSTime(2200, 101.0);
    epochs[2].time = GNSSTime(2200, 102.0);

    const SatelliteId gps1(GNSSSystem::GPS, 1);
    const auto make_factor = [gps1](std::size_t epoch_index,
                                     SignalType signal) {
        FGOProcessor::PseudorangeFactor factor;
        factor.epoch_index = epoch_index;
        factor.satellite = gps1;
        factor.signal = signal;
        factor.corrected_pseudorange_m = 100.0;
        return factor;
    };
    std::vector<FGOProcessor::PseudorangeFactor> factors = {
        make_factor(0U, SignalType::GPS_L1CA),
        make_factor(1U, SignalType::GPS_L1CA),
        make_factor(0U, SignalType::GPS_L5),
        make_factor(2U, SignalType::GPS_L5),
        make_factor(0U, SignalType::GPS_L2C),
    };

    Report report;
    ASSERT_TRUE(apply(
        factors, epochs,
        [](const SatelliteId&, SignalType signal) {
            return signal != SignalType::GPS_L2C;
        },
        [](const GNSSTime& time, const SatelliteId&, SignalType signal,
           double& correction) {
            if (signal == SignalType::GPS_L1CA && time.tow == 101.0) {
                return false;
            }
            if (signal == SignalType::GPS_L5 && time.tow == 102.0) {
                correction = std::numeric_limits<double>::quiet_NaN();
                return true;
            }
            correction = signal == SignalType::GPS_L5 ? 3.0 : 2.0;
            return true;
        },
        report));

    ASSERT_EQ(factors.size(), 2U);
    EXPECT_EQ(report.original_adopted_rows, 5U);
    EXPECT_EQ(report.retained_finite_pc_rows, 2U);
    EXPECT_EQ(report.corrected_rows, 2U);
    EXPECT_EQ(report.dropped_missing_exact_stream_rows, 1U);
    EXPECT_EQ(report.dropped_out_of_domain_rows, 1U);
    EXPECT_EQ(report.dropped_nonfinite_correction_rows, 1U);
    EXPECT_TRUE(report.factor_count_consistent);
    EXPECT_TRUE(report.signal_count_consistent);
    EXPECT_EQ(report.correction_application_passes, 1U);

    const auto& l1 = report.signal_counts.at(SignalType::GPS_L1CA);
    EXPECT_EQ(l1.original_adopted_rows, 2U);
    EXPECT_EQ(l1.retained_finite_pc_rows, 1U);
    EXPECT_EQ(l1.corrected_rows, 1U);
    EXPECT_EQ(l1.matched_exact_stream_rows, 2U);
    EXPECT_EQ(l1.dropped_out_of_domain_rows, 1U);
    EXPECT_TRUE(l1.factor_count_consistent);

    const auto& l5 = report.signal_counts.at(SignalType::GPS_L5);
    EXPECT_EQ(l5.original_adopted_rows, 2U);
    EXPECT_EQ(l5.retained_finite_pc_rows, 1U);
    EXPECT_EQ(l5.corrected_rows, 1U);
    EXPECT_EQ(l5.dropped_nonfinite_correction_rows, 1U);
    EXPECT_TRUE(l5.factor_count_consistent);

    const auto& l2 = report.signal_counts.at(SignalType::GPS_L2C);
    EXPECT_EQ(l2.original_adopted_rows, 1U);
    EXPECT_EQ(l2.retained_finite_pc_rows, 0U);
    EXPECT_EQ(l2.corrected_rows, 0U);
    EXPECT_EQ(l2.dropped_missing_exact_stream_rows, 1U);
    EXPECT_TRUE(l2.factor_count_consistent);
    EXPECT_TRUE(factors[0].native_base_pseudorange_correction_applied);
    EXPECT_TRUE(factors[1].native_base_pseudorange_correction_applied);
}

TEST(BasePseudorangeCompensationTest,
     SourceExactMissMaskRejectsDoubleApplicationWithoutMutation) {
    using namespace libgnss::source_pseudorange_miss_mask;
    std::vector<FGOProcessor::EpochSeed> epochs(1);
    epochs[0].time = GNSSTime(2200, 100.0);
    FGOProcessor::PseudorangeFactor factor;
    factor.epoch_index = 0U;
    factor.satellite = SatelliteId(GNSSSystem::GPS, 1);
    factor.signal = SignalType::GPS_L1CA;
    factor.corrected_pseudorange_m = 100.0;
    std::vector<FGOProcessor::PseudorangeFactor> factors = {factor};

    const auto has_stream = [](const SatelliteId&, SignalType) {
        return true;
    };
    const auto correction_at = [](const GNSSTime&, const SatelliteId&,
                                  SignalType, double& correction) {
        correction = 2.0;
        return true;
    };

    Report first_report;
    ASSERT_TRUE(apply(factors, epochs, has_stream, correction_at, first_report));
    ASSERT_EQ(factors.size(), 1U);
    EXPECT_DOUBLE_EQ(factors.front().corrected_pseudorange_m, 98.0);
    EXPECT_EQ(first_report.correction_application_passes, 1U);

    Report second_report;
    EXPECT_FALSE(apply(factors, epochs, has_stream, correction_at, second_report));
    EXPECT_TRUE(second_report.correction_already_applied);
    EXPECT_EQ(second_report.correction_application_passes, 0U);
    EXPECT_EQ(second_report.original_adopted_rows, 1U);
    EXPECT_EQ(factors.size(), 1U);
    EXPECT_DOUBLE_EQ(factors.front().corrected_pseudorange_m, 98.0);
}
