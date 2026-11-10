#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <lidar_safety/safety_evaluator.hpp>

using lidar_safety::Configuration;
using lidar_safety::SafetyEvaluator;

TEST(SafetyEvaluator, ClearSpacePreservesBothCommandComponents)
{
    const auto decision = SafetyEvaluator().evaluate({}, {0.12, 0.4}, {}, 0.1);
    EXPECT_EQ(decision.state, "CLEAR");
    EXPECT_DOUBLE_EQ(decision.output.linear, 0.12);
    EXPECT_DOUBLE_EQ(decision.output.angular, 0.4);
}

TEST(SafetyEvaluator, SlowdownPreservesCurvature)
{
    const auto decision = SafetyEvaluator().evaluate({{0.3, 0.0}}, {0.12, 0.2}, {}, 0.0);
    ASSERT_EQ(decision.state, "SLOW");
    EXPECT_NEAR(decision.output.linear, 0.048, 1e-9);
    EXPECT_NEAR(decision.output.angular / decision.output.linear, 0.2 / 0.12, 1e-9);
}

TEST(SafetyEvaluator, FrontRiskStopsForwardButPermitsRetreat)
{
    SafetyEvaluator safety;
    EXPECT_EQ(safety.evaluate({{0.18, 0.0}}, {0.15, 0.0}, {}, 0.0).state, "STOP");
    EXPECT_EQ(safety.evaluate({{0.18, 0.0}}, {-0.15, 0.0}, {}, 0.0).state, "CLEAR");
}

TEST(SafetyEvaluator, RearRiskStopsReverseButPermitsForwardEscape)
{
    SafetyEvaluator safety;
    EXPECT_EQ(safety.evaluate({{-0.18, 0.0}}, {-0.15, 0.0}, {}, 0.0).state, "STOP");
    EXPECT_EQ(safety.evaluate({{-0.18, 0.0}}, {0.15, 0.0}, {}, 0.0).state, "CLEAR");
}

TEST(SafetyEvaluator, MeasuredMomentumCannotBeHiddenByAReversal)
{
    const auto decision = SafetyEvaluator().evaluate(
        {{0.18, 0.0}}, {-0.15, 0.0}, {0.15, 0.0}, 0.0);
    EXPECT_EQ(decision.state, "STOP");
}

TEST(SafetyEvaluator, RectangleCornersDistinguishRotationDirection)
{
    SafetyEvaluator safety;
    EXPECT_EQ(safety.evaluate({{0.17, 0.06}}, {0.0, -0.8}, {}, 0.0).state, "STOP");
    EXPECT_NE(safety.evaluate({{0.17, 0.06}}, {0.0, 0.8}, {}, 0.0).state, "STOP");
}

TEST(SafetyEvaluator, CurvedMotionChecksBeyondTheForwardSector)
{
    SafetyEvaluator safety;
    EXPECT_EQ(safety.evaluate({{0.14, 0.16}}, {0.15, 0.8}, {}, 0.0).state, "STOP");
    EXPECT_NE(safety.evaluate({{0.14, 0.16}}, {0.15, 0.0}, {}, 0.0).state, "STOP");
}

TEST(SafetyEvaluator, SpeedAndSensorAgeIncreaseStoppingDistance)
{
    SafetyEvaluator safety;
    const auto slow = safety.evaluate({}, {0.05, 0.0}, {}, 0.0);
    const auto fast = safety.evaluate({}, {0.15, 0.0}, {}, 0.0);
    const auto delayed = safety.evaluate({}, {0.15, 0.0}, {}, 0.4);
    EXPECT_LT(slow.stopping_distance, fast.stopping_distance);
    EXPECT_LT(fast.stopping_distance, delayed.stopping_distance);
    EXPECT_NEAR(delayed.stopping_distance, 0.15 * 0.5 + 0.15 * 0.15, 1e-9);
    EXPECT_NE(safety.evaluate({{0.25, 0.0}}, {0.15, 0.0}, {}, 0.0).state, "STOP");
    EXPECT_EQ(safety.evaluate({{0.25, 0.0}}, {0.15, 0.0}, {}, 0.6).state, "STOP");
}

TEST(SafetyEvaluator, FootprintMarginAndBrakingAreConfigurable)
{
    Configuration config;
    config.front = 0.2;
    EXPECT_EQ(SafetyEvaluator(config).evaluate({{0.25, 0.0}}, {0.15, 0.0}, {}, 0.0).state,
        "STOP");
    config.front = 0.11;
    config.linear_deceleration = 0.1;
    EXPECT_EQ(SafetyEvaluator(config).evaluate({{0.25, 0.0}}, {0.15, 0.0}, {}, 0.0).state,
        "STOP");
}

TEST(SafetyEvaluator, MalformedInputsAndPhysicalOverlapFailClosed)
{
    SafetyEvaluator safety;
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(safety.evaluate({}, {invalid, 0.0}, {}, 0.0).state, "STOP");
    EXPECT_EQ(safety.evaluate({{invalid, 0.0}}, {0.1, 0.0}, {}, 0.0).state, "STOP");
    EXPECT_EQ(safety.evaluate({{0.05, 0.0}}, {0.1, 0.0}, {}, 0.0).state, "STOP");
    EXPECT_THROW(SafetyEvaluator(Configuration{0.0}), std::invalid_argument);
}
