// Insertion-order invariance of per-satellite ephemeris ordering.
//
// Merged RINEX navigation files (and multi-stream RTCM) carry an F/NAV and an
// I/NAV copy of every Galileo record with the same toe. NavigationData must
// keep them in a fixed total order so the copy getEphemeris() returns does not
// depend on how the records were added.

#include <gtest/gtest.h>
#include <libgnss++/core/navigation.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <tuple>
#include <vector>

using namespace libgnss;

namespace {

constexpr int kFnavSource = 258;   // F/NAV E5a-I, clock E5a/E1 (bit 8 + bit 1)
constexpr int kInavE1bSource = 517;  // I/NAV E1-B, clock E5b/E1 (bit 9 + bits 2, 0)
constexpr int kInavE5bSource = 516;  // I/NAV E5b-I, clock E5b/E1 (bit 9 + bit 2)

Ephemeris makeEphemeris(const SatelliteId& sat, double toe_tow, int iode,
                        double af0, int data_source) {
    Ephemeris eph;
    eph.satellite = sat;
    eph.toe = GNSSTime(2200, toe_tow);
    eph.toc = eph.toe;
    eph.tof = eph.toe;
    eph.toes = toe_tow;
    eph.week = 2200;
    eph.sqrt_a = 5440.6;
    eph.e = 0.0003;
    eph.i0 = 0.96;
    eph.omega0 = 1.0;
    eph.omega = 0.5;
    eph.m0 = 0.25;
    eph.iode = static_cast<uint16_t>(iode);
    eph.iodc = static_cast<uint16_t>(iode);
    eph.af0 = af0;
    eph.data_source_code = data_source;
    eph.valid = true;
    return eph;
}

SatelliteId galileo(int prn) { return SatelliteId(GNSSSystem::Galileo, prn); }

// Fingerprint of the stored order for one satellite.
std::vector<std::tuple<double, int, int, double>> order(const NavigationData& nav,
                                                        const SatelliteId& sat) {
    std::vector<std::tuple<double, int, int, double>> out;
    for (const auto& eph : nav.getEphemeris(sat)) {
        out.emplace_back(eph.toe.tow, eph.data_source_code, eph.iode, eph.af0);
    }
    return out;
}

}  // namespace

TEST(EphemerisTieBreakTest, GalileoInavChosenRegardlessOfInsertionOrder) {
    const auto sat = galileo(5);
    const Ephemeris fnav = makeEphemeris(sat, 3600.0, 77, 1.0e-4, kFnavSource);
    const Ephemeris inav = makeEphemeris(sat, 3600.0, 77, 1.0e-4 + 2.0e-8, kInavE1bSource);
    const GNSSTime query(2200, 3700.0);

    NavigationData fnav_first;
    fnav_first.addEphemeris(fnav);
    fnav_first.addEphemeris(inav);
    NavigationData inav_first;
    inav_first.addEphemeris(inav);
    inav_first.addEphemeris(fnav);

    for (const NavigationData* nav : {&fnav_first, &inav_first}) {
        const Ephemeris* selected = nav->getEphemeris(sat, query);
        ASSERT_NE(selected, nullptr);
        EXPECT_NE(selected->data_source_code & galileo_data_source::kClockE5bE1, 0);
        EXPECT_EQ(selected->data_source_code, kInavE1bSource);

        // The SSR IODE-matching path scans the same list.
        const Ephemeris* by_iode = nav->getEphemeris(sat, query, 77);
        ASSERT_NE(by_iode, nullptr);
        EXPECT_EQ(by_iode->data_source_code, kInavE1bSource);
    }
    EXPECT_EQ(order(fnav_first, sat), order(inav_first, sat));

    // The cached satellite state follows the chosen record.
    Vector3d p1, v1, p2, v2;
    double b1, d1, b2, d2;
    ASSERT_TRUE(fnav_first.calculateSatelliteState(sat, query, p1, v1, b1, d1));
    ASSERT_TRUE(inav_first.calculateSatelliteState(sat, query, p2, v2, b2, d2));
    EXPECT_EQ(b1, b2);
    EXPECT_EQ(p1, p2);
}

TEST(EphemerisTieBreakTest, EveryInsertionPermutationGivesTheSameList) {
    const auto sat = galileo(11);
    std::vector<Ephemeris> records = {
        makeEphemeris(sat, 3600.0, 77, 1.0e-4, kFnavSource),
        makeEphemeris(sat, 3600.0, 77, 1.0e-4 + 3.0e-8, kInavE1bSource),
        makeEphemeris(sat, 3600.0, 77, 1.0e-4 + 1.0e-8, kInavE5bSource),
        makeEphemeris(sat, 3600.0, 78, 2.0e-4, kFnavSource),
        makeEphemeris(sat, 4200.0, 79, 1.0e-4, kFnavSource),
        makeEphemeris(sat, 4200.0, 79, 1.0e-4, kInavE1bSource),
    };
    std::vector<int> index(records.size());
    std::iota(index.begin(), index.end(), 0);

    std::vector<std::tuple<double, int, int, double>> reference;
    int permutations = 0;
    do {
        NavigationData nav;
        for (int i : index) nav.addEphemeris(records[i]);
        const auto stored = order(nav, sat);
        if (reference.empty()) reference = stored;
        EXPECT_EQ(stored, reference);
        ++permutations;

        // Equal-toe Galileo copies: all I/NAV before any F/NAV.
        const auto& list = nav.ephemeris_data.at(sat);
        ASSERT_EQ(list.size(), records.size());
        EXPECT_NE(list[0].data_source_code & galileo_data_source::kClockE5bE1, 0);
        EXPECT_NE(list[1].data_source_code & galileo_data_source::kClockE5bE1, 0);
        EXPECT_EQ(list[2].data_source_code, kFnavSource);
        for (size_t i = 1; i < list.size(); ++i) {
            EXPECT_LE(list[i - 1].toe.tow, list[i].toe.tow);
        }
        // At the shared toe, the lowest-ranked I/NAV copy (E5b-I, 516) is first.
        EXPECT_EQ(list[0].data_source_code, kInavE5bSource);
        const Ephemeris* selected = nav.getEphemeris(sat, GNSSTime(2200, 3700.0));
        ASSERT_NE(selected, nullptr);
        EXPECT_EQ(selected->data_source_code, kInavE5bSource);
    } while (std::next_permutation(index.begin(), index.end()));
    EXPECT_EQ(permutations, 720);
}

TEST(EphemerisTieBreakTest, LoneFnavRecordIsStillUsed) {
    const auto sat = galileo(7);
    const GNSSTime query(2200, 3700.0);

    NavigationData nav;
    nav.addEphemeris(makeEphemeris(sat, 3600.0, 12, 1.0e-4, kFnavSource));
    const Ephemeris* selected = nav.getEphemeris(sat, query);
    ASSERT_NE(selected, nullptr);
    EXPECT_EQ(selected->data_source_code, kFnavSource);
    EXPECT_TRUE(nav.hasEphemeris(sat, query));
    Vector3d p, v;
    double bias, drift;
    EXPECT_TRUE(nav.calculateSatelliteState(sat, query, p, v, bias, drift));

    // An I/NAV record with a different toe does not displace the F/NAV one
    // when the F/NAV record is the age-closest.
    nav.addEphemeris(makeEphemeris(sat, 5400.0, 13, 1.0e-4, kInavE1bSource));
    selected = nav.getEphemeris(sat, query);
    ASSERT_NE(selected, nullptr);
    EXPECT_EQ(selected->data_source_code, kFnavSource);
    EXPECT_EQ(selected->toe.tow, 3600.0);
}

TEST(EphemerisTieBreakTest, NonGalileoTiesAreDeterministic) {
    const GNSSTime query(2200, 3700.0);
    for (const auto system : {GNSSSystem::GPS, GNSSSystem::QZSS, GNSSSystem::BeiDou}) {
        const SatelliteId sat(system, 9);
        // Bit 9 of the data-source word means nothing outside Galileo: it must
        // not buy a GPS/QZSS/BeiDou record any preference. Ties fall to
        // data_source_code, then iode ascending.
        std::vector<Ephemeris> records = {
            makeEphemeris(sat, 3600.0, 40, 1.0e-4, 0),
            makeEphemeris(sat, 3600.0, 41, 1.1e-4, 0),
            makeEphemeris(sat, 3600.0, 40, 1.2e-4, 512),
            makeEphemeris(sat, 3600.0, 42, 1.3e-4, 512),
        };
        std::vector<int> index(records.size());
        std::iota(index.begin(), index.end(), 0);
        std::vector<std::tuple<double, int, int, double>> reference;
        do {
            NavigationData nav;
            for (int i : index) nav.addEphemeris(records[i]);
            const auto stored = order(nav, sat);
            if (reference.empty()) reference = stored;
            EXPECT_EQ(stored, reference);
            const Ephemeris* selected = nav.getEphemeris(sat, query);
            ASSERT_NE(selected, nullptr);
            EXPECT_EQ(selected->data_source_code, 0);
            EXPECT_EQ(selected->iode, 40);
            EXPECT_DOUBLE_EQ(selected->af0, 1.0e-4);
        } while (std::next_permutation(index.begin(), index.end()));
        ASSERT_EQ(reference.size(), 4u);
        EXPECT_EQ(std::get<1>(reference[0]), 0);
        EXPECT_EQ(std::get<2>(reference[0]), 40);
        EXPECT_EQ(std::get<2>(reference[1]), 41);
        EXPECT_EQ(std::get<1>(reference[2]), 512);
        EXPECT_EQ(std::get<2>(reference[2]), 40);
    }
}

TEST(EphemerisTieBreakTest, CopyAndIfNewPathsKeepTheSameOrder) {
    const auto sat = galileo(21);
    const Ephemeris fnav = makeEphemeris(sat, 3600.0, 5, 1.0e-4, kFnavSource);
    const Ephemeris inav = makeEphemeris(sat, 3600.0, 5, 1.0e-4 + 4.0e-8, kInavE1bSource);

    NavigationData a;
    EXPECT_TRUE(a.addEphemerisIfNew(fnav));
    EXPECT_TRUE(a.addEphemerisIfNew(inav));
    NavigationData b;
    EXPECT_TRUE(b.addEphemerisIfNew(inav));
    EXPECT_TRUE(b.addEphemerisIfNew(fnav));
    EXPECT_EQ(order(a, sat), order(b, sat));

    // Re-adding a sorted copy record by record (the merge/replay pattern, in
    // either direction) reproduces the same list.
    NavigationData rebuilt_forward;
    for (const auto& eph : a.getEphemeris(sat)) rebuilt_forward.addEphemeris(eph);
    NavigationData rebuilt_backward;
    const auto stored = a.getEphemeris(sat);
    for (auto it = stored.rbegin(); it != stored.rend(); ++it) rebuilt_backward.addEphemeris(*it);
    NavigationData copy = a;
    EXPECT_EQ(order(rebuilt_forward, sat), order(a, sat));
    EXPECT_EQ(order(rebuilt_backward, sat), order(a, sat));
    EXPECT_EQ(order(copy, sat), order(a, sat));

    const GNSSTime query(2200, 3650.0);
    for (const NavigationData* nav : {&a, &b, &rebuilt_forward, &rebuilt_backward, &copy}) {
        const Ephemeris* selected = nav->getEphemeris(sat, query);
        ASSERT_NE(selected, nullptr);
        EXPECT_EQ(selected->data_source_code, kInavE1bSource);
    }
}

// NavigationData::removeEphemeris erases exactly one record, identified by every field.
TEST(EphemerisRemovalTest, ErasesExactlyTheMatchingRecord) {
    const auto sat = galileo(21);
    const auto other_sat = galileo(22);
    const Ephemeris first = makeEphemeris(sat, 3600.0, 5, 1.0e-4, kInavE1bSource);
    const Ephemeris second = makeEphemeris(sat, 4200.0, 6, 2.0e-4, kInavE1bSource);
    const Ephemeris third = makeEphemeris(sat, 4800.0, 7, 3.0e-4, kInavE1bSource);
    const Ephemeris neighbour = makeEphemeris(other_sat, 3600.0, 5, 1.0e-4, kInavE1bSource);

    NavigationData nav;
    for (const auto& eph : {third, first, neighbour, second}) nav.addEphemeris(eph);
    ASSERT_EQ(nav.getEphemeris(sat).size(), 3U);

    // The query time selects `second`; once it is removed the selection moves on.
    const GNSSTime query(2200, 4210.0);
    ASSERT_NE(nav.getEphemeris(sat, query), nullptr);
    EXPECT_EQ(nav.getEphemeris(sat, query)->iode, 6);
    const std::uint64_t revision = nav.getRevision();
    EXPECT_TRUE(nav.removeEphemeris(second));
    EXPECT_GT(nav.getRevision(), revision);
    const auto remaining = nav.getEphemeris(sat);
    ASSERT_EQ(remaining.size(), 2U);
    EXPECT_EQ(remaining[0].iode, 5);
    EXPECT_EQ(remaining[1].iode, 7);
    ASSERT_NE(nav.getEphemeris(sat, query), nullptr);
    EXPECT_NE(nav.getEphemeris(sat, query)->iode, 6);       // the satellite state cache does not serve it
    EXPECT_EQ(nav.getEphemeris(other_sat).size(), 1U);       // other satellites are untouched

    // The same record is gone: a second removal finds nothing.
    const std::uint64_t after = nav.getRevision();
    EXPECT_FALSE(nav.removeEphemeris(second));
    EXPECT_EQ(nav.getRevision(), after);                      // a miss changes nothing
    EXPECT_EQ(nav.getEphemeris(sat).size(), 2U);
}

TEST(EphemerisRemovalTest, ARecordThatDiffersInAnyFieldIsLeftAlone) {
    const auto sat = galileo(21);
    const Ephemeris stored = makeEphemeris(sat, 3600.0, 5, 1.0e-4, kInavE1bSource);
    NavigationData nav;
    nav.addEphemeris(stored);
    const std::uint64_t revision = nav.getRevision();

    // Same satellite, issue and toe; one other field differs.
    Ephemeris af0 = stored;
    af0.af0 = std::nextafter(stored.af0, 1.0);
    Ephemeris omega0 = stored;
    omega0.omega0 += 1e-12;
    Ephemeris health = stored;
    health.health = 1;
    Ephemeris ln = stored;
    ln.glonass_ln = 1;
    Ephemeris tof = stored;
    tof.tof = GNSSTime(2200, 3601.0);
    Ephemeris source = stored;
    source.data_source_code = kFnavSource;
    Ephemeris toc = stored;
    toc.toc = GNSSTime(2200, 3601.0);
    Ephemeris position = stored;
    position.glonass_position.x() = 1.0;
    for (const Ephemeris* probe : {&af0, &omega0, &health, &ln, &tof, &source, &toc, &position}) {
        EXPECT_FALSE(nav.removeEphemeris(*probe));
    }
    // Same fields, other satellite.
    Ephemeris other_sat = stored;
    other_sat.satellite = galileo(22);
    EXPECT_FALSE(nav.removeEphemeris(other_sat));
    EXPECT_EQ(nav.getRevision(), revision);
    EXPECT_EQ(nav.getEphemeris(sat).size(), 1U);

    // The exact record goes; an empty satellite leaves no entry behind.
    EXPECT_TRUE(nav.removeEphemeris(stored));
    EXPECT_TRUE(nav.getEphemeris(sat).empty());
    EXPECT_TRUE(nav.isEmpty());
}

TEST(EphemerisRemovalTest, OnlyOneOfTwoIdenticalRecordsIsErased) {
    const auto sat = galileo(21);
    const Ephemeris eph = makeEphemeris(sat, 3600.0, 5, 1.0e-4, kInavE1bSource);
    NavigationData nav;
    nav.addEphemeris(eph);
    nav.addEphemeris(eph);       // addEphemeris does not de-duplicate
    ASSERT_EQ(nav.getEphemeris(sat).size(), 2U);
    EXPECT_TRUE(nav.removeEphemeris(eph));
    EXPECT_EQ(nav.getEphemeris(sat).size(), 1U);
    EXPECT_TRUE(nav.removeEphemeris(eph));
    EXPECT_EQ(nav.getEphemeris(sat).size(), 0U);
    EXPECT_FALSE(nav.removeEphemeris(eph));
}
