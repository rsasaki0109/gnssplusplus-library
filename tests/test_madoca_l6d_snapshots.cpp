#include <gtest/gtest.h>

#include <libgnss++/io/madoca_l6d.hpp>

#include <cmath>
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
