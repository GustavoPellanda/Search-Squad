#include <cmath>
#include <gtest/gtest.h>
#include <explorer/room_geometry.hpp>

// Construct a synthetic closed room with an entrance, independently of the implementation:
nav_msgs::msg::OccupancyGrid make_room()
{
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = 60;
    map.info.height = 50;
    map.info.resolution = 0.1;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(3000, -1);
    for(int row = 10; row <= 40; ++row) {
        for(int column = 5; column <= 50; ++column) {
            const bool wall = row == 10 || row == 40 || column == 5 || column == 50;
            map.data[row * 60 + column] = wall ? 100 : 0;
        }
    }
    for(int column = 23; column <= 32; ++column) map.data[10 * 60 + column] = 0;
    return map;
}


// A measured doorway provides a normal, not true room dimensions:
geometry_msgs::msg::PoseStamped doorway()
{
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 2.75;
    pose.pose.position.y = 1.05;
    pose.pose.orientation.z = std::sqrt(0.5);
    pose.pose.orientation.w = std::sqrt(0.5);
    return pose;
}


TEST(RoomGeometry, ClosedObservedRoomHasNoFrontiers)
{
    auto map = make_room();
    explorer::RoomGeometry geometry(map, doorway(), 0.22, 0.20);
    geometry_msgs::msg::Point seed;
    seed.x = 2.75;
    seed.y = 1.65;
    const auto analysis = geometry.analyze(seed);
    EXPECT_TRUE(analysis.seed_valid);
    EXPECT_FALSE(analysis.touches_extent);
    EXPECT_TRUE(analysis.frontiers.empty());
    EXPECT_GT(analysis.known_cells, 1000U);
}


TEST(RoomGeometry, UnobservedAreaRemainsAFrontier)
{
    auto map = make_room();
    for(int row = 30; row < 40; ++row) {
        for(int column = 6; column < 20; ++column) map.data[row * 60 + column] = -1;
    }
    explorer::RoomGeometry geometry(map, doorway(), 0.22, 0.20);
    geometry_msgs::msg::Point seed;
    seed.x = 2.75;
    seed.y = 1.65;
    const auto analysis = geometry.analyze(seed);
    ASSERT_FALSE(analysis.frontiers.empty());
    geometry_msgs::msg::Point target;
    ASSERT_TRUE(geometry.choose_target(analysis, seed, {}, target));
    EXPECT_EQ(map.data[geometry.index(target.x, target.y)], 0);
    EXPECT_TRUE(geometry.inside(target.x, target.y));
    EXPECT_TRUE(analysis.safe[geometry.index(target.x, target.y)]);
}


TEST(RoomGeometry, UnknownSeedCannotProduceAValidRegion)
{
    auto map = make_room();
    explorer::RoomGeometry geometry(map, doorway(), 0.22, 0.20);
    geometry_msgs::msg::Point seed;
    seed.x = 0.1;
    seed.y = 0.1;
    EXPECT_FALSE(geometry.analyze(seed).seed_valid);
}


TEST(RoomGeometry, ThinWallPinholeDoesNotBecomeUnseenFloor)
{
    auto map = make_room();
    map.data[40 * 60 + 12] = -1;
    explorer::RoomGeometry geometry(map, doorway(), 0.22, 0.05);
    geometry_msgs::msg::Point seed;
    seed.x = 2.75;
    seed.y = 1.65;
    EXPECT_TRUE(geometry.analyze(seed).frontiers.empty());
    EXPECT_EQ(map.data[40 * 60 + 12], -1);
}


TEST(RoomGeometry, UnobservedWideOpeningRemainsUnresolved)
{
    auto map = make_room();
    for(int column = 20; column <= 35; ++column) map.data[40 * 60 + column] = -1;
    explorer::RoomGeometry geometry(map, doorway(), 0.22, 0.20);
    geometry_msgs::msg::Point seed;
    seed.x = 2.75;
    seed.y = 1.65;
    EXPECT_FALSE(geometry.analyze(seed).frontiers.empty());
}


TEST(RoomGeometry, MaskNeverModifiesTheStructuralDoorway)
{
    auto map = make_room();
    const auto original = map.data;
    explorer::RoomGeometry geometry(map, doorway(), 0.22, 0.20);
    geometry_msgs::msg::Point seed;
    seed.x = 2.75;
    seed.y = 1.65;
    const auto analysis = geometry.analyze(seed);
    EXPECT_EQ(map.data, original);
    EXPECT_EQ(analysis.region[geometry.index(2.75, 1.05)], 0);
    EXPECT_EQ(map.data[geometry.index(2.75, 1.05)], 0);
}
