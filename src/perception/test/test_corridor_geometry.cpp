#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "perception/corridor_geometry.hpp"

namespace
{
// Add measured wall support and a missing-return separator in scan order:
void add_wall(std::vector<perception::Point> & points, double start, double end, double side)
{
    for(int index = 0; index <= 60; ++index) {
        const double fraction = index / 60.0;
        points.push_back({start + fraction * (end - start), side});
    }
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    points.push_back({unavailable, unavailable});
}


// Construct a corridor with no supplied centerline or expected width in the estimator:
std::vector<perception::Point> corridor(double half_width = 0.925)
{
    std::vector<perception::Point> points;
    add_wall(points, -3.0, 3.0, half_width);
    add_wall(points, -3.0, 3.0, -half_width);
    return points;
}

TEST(CorridorGeometry, MeasuresWidthAndSignedLateralError)
{
    perception::CorridorGeometry geometry({});
    const auto estimate = geometry.estimate(corridor(), {{0.0, 0.2}, 0.0}, 1.0);
    ASSERT_TRUE(estimate.axis_valid);
    EXPECT_NEAR(estimate.width, 1.85, 0.001);
    EXPECT_NEAR(estimate.lateral_error, 0.2, 0.001);
    EXPECT_NEAR(estimate.left_distance, 0.725, 0.001);
    EXPECT_NEAR(estimate.right_distance, 1.125, 0.001);
}

TEST(CorridorGeometry, EstimatesHeadingRatherThanUsingRobotHeadingAsTruth)
{
    perception::CorridorGeometry geometry({});
    perception::CorridorEstimate estimate;
    for(int index = 1; index <= 25; ++index) {
        estimate = geometry.estimate(corridor(), {{0.0, 0.0}, 0.1}, index * 0.1);
    }
    ASSERT_TRUE(estimate.axis_valid);
    EXPECT_NEAR(estimate.heading, 0.0, 0.001);
    EXPECT_NEAR(estimate.heading_error, 0.1, 0.001);
}

TEST(CorridorGeometry, OppositeDoorsDoNotSelectWallsInsideRooms)
{
    perception::CorridorGeometry geometry({});
    std::vector<perception::Point> points;
    for(double side : {-0.925, 0.925}) {
        add_wall(points, -3.0, -0.6, side);
        add_wall(points, 0.6, 3.0, side);
        add_wall(points, -0.5, 0.5, side * 3.0);
    }
    const auto estimate = geometry.estimate(points, {{0.0, 0.0}, 0.0}, 1.0);
    ASSERT_TRUE(estimate.axis_valid);
    EXPECT_NEAR(estimate.width, 1.85, 0.001);
    ASSERT_EQ(estimate.left_openings.size(), 1U);
    ASSERT_EQ(estimate.right_openings.size(), 1U);
    EXPECT_NEAR(estimate.left_openings[0].length, 1.2, 0.001);
}

TEST(CorridorGeometry, MissingWallIsPredictedAndExpires)
{
    perception::CorridorGeometry geometry({});
    geometry.estimate(corridor(), {{0.0, 0.0}, 0.0}, 1.0);
    std::vector<perception::Point> right_only;
    add_wall(right_only, -3.0, 3.0, -0.925);
    const auto predicted = geometry.estimate(right_only, {{0.0, 0.0}, 0.0}, 2.0);
    EXPECT_TRUE(predicted.axis_valid);
    EXPECT_FALSE(predicted.left_measured);
    EXPECT_TRUE(predicted.left_predicted);
    EXPECT_NEAR(predicted.left_age, 1.0, 0.001);
    const auto expired = geometry.estimate(right_only, {{0.0, 0.0}, 0.0}, 22.0);
    EXPECT_FALSE(expired.axis_valid);
    EXPECT_TRUE(std::isnan(expired.left_distance));
}

TEST(CorridorGeometry, PredictionExpiresByDistanceAndRepeatedScansAreNotNewEvidence)
{
    perception::CorridorGeometry geometry({});
    geometry.estimate(corridor(), {{0.0, 0.0}, 0.0}, 1.0);
    EXPECT_FALSE(geometry.estimate(corridor(), {{0.0, 0.0}, 0.0}, 1.0).axis_valid);
    EXPECT_FALSE(geometry.estimate({}, {{2.0, 0.0}, 0.0}, 2.0).axis_valid);
}

TEST(CorridorGeometry, TransverseEvidenceRequiresCorridorCoverage)
{
    perception::CorridorGeometry geometry({});
    auto points = corridor();
    for(int index = 0; index <= 40; ++index) {
        points.push_back({2.0, -0.2 + index * 0.01});
    }
    EXPECT_FALSE(geometry.estimate(points, {{0.0, 0.0}, 0.0}, 1.0).terminal_candidate);
    points = corridor();
    for(int index = 0; index <= 80; ++index) {
        points.push_back({2.0, -0.925 + index * 1.85 / 80.0});
    }
    EXPECT_TRUE(geometry.estimate(points, {{0.0, 0.0}, 0.0}, 2.0).terminal_candidate);
}

TEST(CorridorGeometry, SupportsDifferentWidthsAndRejectsInvalidConfiguration)
{
    perception::CorridorGeometry geometry({});
    EXPECT_NEAR(geometry.estimate(corridor(0.6), {{0.0, 0.0}, 0.0}, 1.0).width, 1.2, 0.001);
    perception::GeometryConfiguration invalid;
    invalid.max_width = invalid.min_width;
    EXPECT_THROW(perception::CorridorGeometry estimator(invalid), std::invalid_argument);
}
TEST(CorridorGeometry, TerminalWallCanStraddleTheScanAngularOrigin)
{
    perception::CorridorGeometry geometry({});
    auto points = corridor();
    for(int index = 0; index <= 40; ++index) points.push_back({2.0, index * 0.925 / 40.0});
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    points.push_back({unavailable, unavailable});
    for(int index = 0; index <= 40; ++index) points.push_back({2.0, -index * 0.925 / 40.0});
    EXPECT_TRUE(geometry.estimate(points, {{0.0, 0.0}, 0.0}, 1.0).terminal_candidate);
}
}  // namespace
