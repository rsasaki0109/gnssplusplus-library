#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/io/madoca_l6d.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace libgnss::io;

TEST(MadocaL6dSnapshotTest, MaterializesPersistentStoreForReceiver) {
    auto region = std::make_unique<MadocaIonoRegion>();
    region->rvalid = 1;
    region->narea = 1;
    auto& area = region->area[3];
    area.avalid = 1;
    area.sid = 1;
    area.type = 0;
    area.ref[0] = 35.0;
    area.ref[1] = 139.0;
    area.span[0] = 1000.0;
    area.sat[0].t0.time = 1'700'000'000;
    area.sat[0].coef[0] = 10.0;
    area.sat[0].sqi = 1;

    auto store = std::make_unique<MadocaIonoStore>();
    store->update(7, *region);
    const double receiver_ecef[3] = {-3'947'450.0, 3'431'468.0, 3'636'866.0};
    const MadocaGtime decode_time{1'700'000'000, 0.0};
    std::vector<MadocaIonoSnapshot> snapshots;

    ASSERT_TRUE(appendMadocaIonoSnapshot(
        *store, receiver_ecef, decode_time, 7, snapshots));
    ASSERT_EQ(snapshots.size(), 1u);
    EXPECT_EQ(snapshots[0].decode_time.time, decode_time.time);
    EXPECT_EQ(snapshots[0].updated_region_id, 7);
    EXPECT_EQ(snapshots[0].correction.rid, 7);
    EXPECT_EQ(snapshots[0].correction.anum, 3);
    EXPECT_EQ(snapshots[0].correction.t0[0].time, area.sat[0].t0.time);
    EXPECT_TRUE(std::isfinite(snapshots[0].correction.dly[0]));
    EXPECT_GT(snapshots[0].correction.dly[0], 0.0);
    EXPECT_GT(snapshots[0].correction.std[0], 0.0);
}

TEST(MadocaL6dSnapshotTest, RejectsMissingAndIncompleteFiles) {
    const double reference_epoch[6] = {2025, 4, 1, 0, 0, 0};
    const double receiver_ecef[3] = {-3'947'450.0, 3'431'468.0, 3'636'866.0};
    std::vector<MadocaIonoSnapshot> snapshots;
    std::string error;

    EXPECT_FALSE(decodeMadocaL6dFileToSnapshots(
        "missing-madoca-l6d-file.l6", reference_epoch, receiver_ecef,
        snapshots, &error));
    EXPECT_NE(error.find("failed to open"), std::string::npos);

    const auto path = std::filesystem::temp_directory_path() /
        "gnsspp_incomplete_madoca_l6d.l6";
    {
        std::ofstream output(path, std::ios::binary);
        const unsigned char bytes[] = {0x1A, 0xCF, 0xFC, 0x1D, 0x00};
        output.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
    }
    error.clear();
    EXPECT_FALSE(decodeMadocaL6dFileToSnapshots(
        path.string(), reference_epoch, receiver_ecef, snapshots, &error));
    EXPECT_NE(error.find("no complete L6D messages"), std::string::npos);
    std::filesystem::remove(path);
}

namespace {

// Packs fields MSB-first into one 218-byte L6 message (no R-S parity).
class L6BitWriter {
public:
    L6BitWriter() : bytes_(218, 0) {}
    void put(std::uint64_t value, int bits) {
        for (int i = bits - 1; i >= 0; --i, ++pos_) {
            if ((value >> i) & 1u) {
                bytes_[pos_ / 8] |= static_cast<std::uint8_t>(0x80u >> (pos_ % 8));
            }
        }
    }
    void putSigned(int value, int bits) {
        put(static_cast<std::uint64_t>(value) & ((std::uint64_t{1} << bits) - 1), bits);
    }
    const std::vector<std::uint8_t>& bytes() const { return bytes_; }

private:
    std::vector<std::uint8_t> bytes_;
    std::size_t pos_ = 0;
};

// One complete single-frame L6D message: MT1 coverage with one rectangular
// area (0.1 deg units) followed by an MT2 correction message for that area.
std::vector<std::uint8_t> makeL6dMessage(double ref_lat_deg, double ref_lon_deg,
                                         double span_deg) {
    L6BitWriter w;
    w.put(0x1ACFFC1Du, 32);
    w.put(200, 8);        // PRN
    w.put(2, 3);          // vendor ID
    w.put(0, 2);          // message generation facility
    w.put(0, 1);          // correction service ID
    w.put(0, 1);          // applicable nav. message extension
    w.put(1, 1);          // subframe indicator: first data
    w.put(0, 1);          // alert flag
    w.put(1, 12);         // MT1 coverage
    w.put(0, 4);
    w.put(100, 20);       // epoch time
    w.put(0, 4);
    w.put(0, 1);
    w.put(0, 4);          // IOD
    w.put(5, 8);          // region ID
    w.put(0, 1);          // region alert
    w.put(77, 16);        // length of the MT2 message that follows
    w.put(1, 5);          // number of areas
    w.put(0, 5);          // area number
    w.put(0, 1);          // rectangle
    w.putSigned(static_cast<int>(std::lround(ref_lat_deg * 10.0)), 11);
    w.put(static_cast<std::uint64_t>(std::lround(ref_lon_deg * 10.0)), 12);
    w.put(static_cast<std::uint64_t>(std::lround(span_deg * 10.0)), 8);
    w.put(static_cast<std::uint64_t>(std::lround(span_deg * 10.0)), 8);
    w.put(2, 12);         // MT2 correction
    w.put(0, 4);
    w.put(100, 12);
    w.put(0, 4);
    w.put(0, 1);
    w.put(0, 4);          // IOD
    w.put(5, 8);
    w.put(0, 5);          // area number
    w.put(0, 2);          // correction type
    w.put(0, 25);         // no satellites
    return w.bytes();
}

std::string writeL6dFile(const std::string& name,
                         const std::vector<std::uint8_t>& bytes) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream output(path, std::ios::binary);
    output.write(reinterpret_cast<const char*>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    return path.string();
}

// Receiver near Tokyo (about 35.5N, 139.0E).
const double kTokyoEcef[3] = {-3'947'450.0, 3'431'468.0, 3'636'866.0};
const double kReferenceEpoch[6] = {2025, 4, 1, 0, 0, 0};

}  // namespace

TEST(MadocaL6dLoadStatusTest, ClassifiesFailureOutsideCoverageAndLoaded) {
    EXPECT_EQ(classifyMadocaL6dLoad(false, 0), MadocaL6dLoadStatus::kFailed);
    EXPECT_EQ(classifyMadocaL6dLoad(false, 12), MadocaL6dLoadStatus::kFailed);
    EXPECT_EQ(classifyMadocaL6dLoad(true, 0), MadocaL6dLoadStatus::kOutsideCoverage);
    EXPECT_EQ(classifyMadocaL6dLoad(true, 1), MadocaL6dLoadStatus::kLoaded);
}

TEST(MadocaL6dLoadStatusTest, SyntheticFileDecodesWithAndWithoutCoverage) {
    const auto inside = writeL6dFile(
        "gnsspp_l6d_inside.l6", makeL6dMessage(35.5, 139.0, 5.0));
    const auto outside = writeL6dFile(
        "gnsspp_l6d_outside.l6", makeL6dMessage(-30.0, 130.0, 5.0));
    std::vector<MadocaIonoSnapshot> snapshots;
    std::string error;

    ASSERT_TRUE(decodeMadocaL6dFileToSnapshots(
        inside, kReferenceEpoch, kTokyoEcef, snapshots, &error)) << error;
    EXPECT_EQ(snapshots.size(), 1u);
    EXPECT_EQ(snapshots[0].updated_region_id, 5);

    // Decoded fine, but no area covers the receiver: success with 0 snapshots.
    ASSERT_TRUE(decodeMadocaL6dFileToSnapshots(
        outside, kReferenceEpoch, kTokyoEcef, snapshots, &error)) << error;
    EXPECT_TRUE(snapshots.empty());

    std::filesystem::remove(inside);
    std::filesystem::remove(outside);
}

TEST(MadocaL6dLoadStatusTest, ProcessorLoadSeparatesOutsideCoverageFromFailure) {
    const auto inside = writeL6dFile(
        "gnsspp_l6d_proc_inside.l6", makeL6dMessage(35.5, 139.0, 5.0));
    const auto outside = writeL6dFile(
        "gnsspp_l6d_proc_outside.l6", makeL6dMessage(-30.0, 130.0, 5.0));
    const auto truncated = writeL6dFile(
        "gnsspp_l6d_proc_truncated.l6", {0x1A, 0xCF, 0xFC, 0x1D, 0x00});
    const auto empty = writeL6dFile("gnsspp_l6d_proc_empty.l6", {});

    const auto load = [&](const std::vector<std::string>& files,
                          bool* apply_after, bool* loaded_after) {
        libgnss::PPPProcessor::PPPConfig config;
        config.apply_madoca_l6d_ionosphere = true;
        libgnss::PPPProcessor processor(config);
        const auto status =
            processor.loadMadocaL6dProducts(files, kReferenceEpoch, kTokyoEcef);
        *apply_after = processor.getPPPConfig().apply_madoca_l6d_ionosphere;
        *loaded_after = processor.hasLoadedMadocaL6dProducts();
        return status;
    };
    bool apply = false;
    bool loaded = false;

    EXPECT_EQ(load({inside}, &apply, &loaded), MadocaL6dLoadStatus::kLoaded);
    EXPECT_TRUE(apply);
    EXPECT_TRUE(loaded);

    // Outside the service area: not an error, and the constraint is switched
    // off so the run matches one without L6D input.
    EXPECT_EQ(load({outside}, &apply, &loaded),
              MadocaL6dLoadStatus::kOutsideCoverage);
    EXPECT_FALSE(apply);
    EXPECT_FALSE(loaded);

    // Any decodable file with a snapshot keeps L6D active.
    EXPECT_EQ(load({outside, inside}, &apply, &loaded),
              MadocaL6dLoadStatus::kLoaded);
    EXPECT_TRUE(apply);

    // Real load failures stay failures.
    EXPECT_EQ(load({"missing-madoca-l6d-file.l6"}, &apply, &loaded),
              MadocaL6dLoadStatus::kFailed);
    EXPECT_EQ(load({truncated}, &apply, &loaded), MadocaL6dLoadStatus::kFailed);
    EXPECT_EQ(load({empty}, &apply, &loaded), MadocaL6dLoadStatus::kFailed);
    EXPECT_EQ(load({}, &apply, &loaded), MadocaL6dLoadStatus::kFailed);
    EXPECT_EQ(load({outside, truncated}, &apply, &loaded),
              MadocaL6dLoadStatus::kFailed);
    EXPECT_FALSE(loaded);

    for (const auto& path : {inside, outside, truncated, empty}) {
        std::filesystem::remove(path);
    }
}

namespace {

MadocaIonoSnapshot makeSnapshot(std::int64_t time, double sec, int region,
                                double delay) {
    MadocaIonoSnapshot snapshot;
    snapshot.decode_time = {time, sec};
    snapshot.updated_region_id = region;
    snapshot.correction.rid = region;
    snapshot.correction.dly[0] = delay;
    return snapshot;
}

}  // namespace

TEST(MadocaIonoProductsTest, OrdersMultipleSourcesAndNeverSelectsFuture) {
    MadocaIonoProducts products;
    const std::vector<MadocaIonoSnapshot> second_source = {
        makeSnapshot(1'700'000'030, 0.0, 8, 3.0),
        makeSnapshot(1'700'000'010, 0.0, 8, 1.0),
    };
    const std::vector<MadocaIonoSnapshot> first_source = {
        makeSnapshot(1'700'000'020, 0.0, 7, 2.0),
    };
    EXPECT_EQ(products.addSnapshots(second_source), 2u);
    EXPECT_EQ(products.addSnapshots(first_source), 1u);
    ASSERT_EQ(products.size(), 3u);
    EXPECT_EQ(products.snapshots()[0].decode_time.time, 1'700'000'010);
    EXPECT_EQ(products.snapshots()[1].decode_time.time, 1'700'000'020);
    EXPECT_EQ(products.snapshots()[2].decode_time.time, 1'700'000'030);

    EXPECT_EQ(products.latestAtOrBefore({1'700'000'009, 0.9}), nullptr);
    const auto* selected = products.latestAtOrBefore({1'700'000'025, 0.0});
    ASSERT_NE(selected, nullptr);
    EXPECT_DOUBLE_EQ(selected->correction.dly[0], 2.0);
}

TEST(MadocaIonoProductsTest, ReplacesDuplicateAndAppliesFreshnessGate) {
    MadocaIonoProducts products;
    ASSERT_TRUE(products.addSnapshot(makeSnapshot(1'700'000'000, 0.5, 7, 1.0)));
    ASSERT_TRUE(products.addSnapshot(makeSnapshot(1'700'000'000, 0.5, 7, 4.0)));
    EXPECT_EQ(products.size(), 1u);

    const auto* fresh = products.latestAtOrBefore({1'700'000'005, 0.5}, 5.0);
    ASSERT_NE(fresh, nullptr);
    EXPECT_DOUBLE_EQ(fresh->correction.dly[0], 4.0);
    EXPECT_EQ(products.latestAtOrBefore({1'700'000'005, 0.6}, 5.0), nullptr);
    EXPECT_NE(products.latestAtOrBefore({1'700'100'000, 0.0}), nullptr);
}

TEST(MadocaIonoProductsTest, RejectsInvalidSnapshotKeysAndQueries) {
    MadocaIonoProducts products;
    EXPECT_FALSE(products.addSnapshot(makeSnapshot(0, 0.0, 7, 1.0)));
    EXPECT_FALSE(products.addSnapshot(makeSnapshot(1'700'000'000, 1.0, 7, 1.0)));
    EXPECT_FALSE(products.addSnapshot(makeSnapshot(1'700'000'000, 0.0, -1, 1.0)));
    EXPECT_TRUE(products.empty());
    EXPECT_EQ(products.latestAtOrBefore({1'700'000'000, 0.0}), nullptr);
}
