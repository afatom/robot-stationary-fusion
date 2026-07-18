#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "udemy_ros2_pkg/static_stationary_detector.hpp"

class StaticStationaryDetectorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        detector = std::make_unique<StaticStationaryDetector>(
            0.5,
            0.1,
            0.5,
			0.20,
			0.05,
			0.20,
            32,
            32);
    }

    void TearDown() override
    {
        detector.reset();
    }

    void addFreshStillSamples(double base_time)
    {
        detector->updateOdometry(0.01, base_time + 0.01);
        detector->updateOdometry(0.02, base_time + 0.02);
        detector->updateOdometry(0.03, base_time + 0.03);

        detector->updateRange(1.00, base_time + 0.01);
        detector->updateRange(1.01, base_time + 0.02);
        detector->updateRange(1.00, base_time + 0.03);
    }

    std::unique_ptr<StaticStationaryDetector> detector;
};

TEST_F(StaticStationaryDetectorTest, ReturnsFalseWhenNoSamplesExist)
{
	EXPECT_FALSE(detector->isStationary(10.0));
}

TEST_F(StaticStationaryDetectorTest, ReturnsFalseWithOnlyOdometrySamples)
{
	detector->updateOdometry(0.01, 1.0);
	detector->updateOdometry(0.02, 1.1);

	EXPECT_FALSE(detector->isStationary(1.2));
}

TEST_F(StaticStationaryDetectorTest, ReturnsFalseWithOnlyRangeSamples)
{
	detector->updateRange(1.0, 1.0);
	detector->updateRange(1.0, 1.1);
	detector->updateRange(1.0, 1.2);

	EXPECT_FALSE(detector->isStationary(1.3));
}

TEST_F(StaticStationaryDetectorTest, ReturnsTrueWhenBothSensorsAreStill)
{
	addFreshStillSamples(1.0);

	EXPECT_TRUE(detector->isStationary(1.2));
}

TEST_F(StaticStationaryDetectorTest, ReturnsFalseWhenOdometryExceedRatioIsAboveMovingThreshold)
{
	// 2/4 samples above threshold => 50% > 20% moving ratio threshold.
	detector->updateOdometry(0.05, 1.0);
	detector->updateOdometry(0.12, 1.1);
	detector->updateOdometry(0.15, 1.2);
	detector->updateOdometry(0.03, 1.3);

	detector->updateRange(1.0, 1.0);
	detector->updateRange(1.0, 1.1);
	detector->updateRange(1.0, 1.2);
	detector->updateRange(1.0, 1.3);

	EXPECT_FALSE(detector->isStationary(1.4));
}

TEST_F(StaticStationaryDetectorTest, ReturnsFalseWhenRangeVarianceExceedsThreshold)
{
	detector->updateOdometry(0.01, 1.0);
	detector->updateOdometry(0.02, 1.1);
	detector->updateOdometry(0.03, 1.2);

	detector->updateRange(0.0, 1.0);
	detector->updateRange(1.0, 1.1);
	detector->updateRange(2.0, 1.2);

	EXPECT_FALSE(detector->isStationary(1.3));
}

TEST_F(StaticStationaryDetectorTest, IgnoresSamplesOutsideTheTimeWindow)
{
	detector->updateOdometry(1.0, 0.1);
	detector->updateRange(10.0, 0.1);
	detector->updateRange(10.0, 0.2);

	EXPECT_FALSE(detector->isStationary(1.2));
}

TEST_F(StaticStationaryDetectorTest, UsesFreshSamplesEvenWhenOlderSamplesAreMoving)
{
	detector->updateOdometry(1.0, 0.1);
	detector->updateRange(10.0, 0.1);

	addFreshStillSamples(1.0);

	EXPECT_TRUE(detector->isStationary(1.2));
}

TEST_F(StaticStationaryDetectorTest, ReturnsTrueWhenExceedRatioIsBelowMovingThreshold)
{
	// 1/6 samples above threshold => 16.7% <= 20%, should still be stationary.
	detector->updateOdometry(0.11, 1.0);
	detector->updateOdometry(0.01, 1.1);
	detector->updateOdometry(0.02, 1.2);
	detector->updateOdometry(0.03, 1.3);
	detector->updateOdometry(0.01, 1.4);
	detector->updateOdometry(0.02, 1.5);

	detector->updateRange(1.0, 1.0);
	detector->updateRange(1.0, 1.1);
	detector->updateRange(1.0, 1.2);
	detector->updateRange(1.0, 1.3);
	detector->updateRange(1.0, 1.4);
	detector->updateRange(1.0, 1.5);

	EXPECT_TRUE(detector->isStationary(1.6));
}

TEST_F(StaticStationaryDetectorTest, HysteresisKeepsStationaryInMiddleBand)
{
	addFreshStillSamples(1.0);
	ASSERT_TRUE(detector->isStationary(1.2));

	// 2/10 above threshold => exactly 20%, between 15% and 25% band.
	for (int i = 0; i < 10; ++i) {
		double v = (i == 1 || i == 5) ? 0.12 : 0.01;
		detector->updateOdometry(v, 2.0 + i * 0.01);
		detector->updateRange(1.0, 2.0 + i * 0.01);
	}

	EXPECT_TRUE(detector->isStationary(2.2));
}

TEST_F(StaticStationaryDetectorTest, HysteresisKeepsMovingInMiddleBand)
{
	// First force moving with clear evidence.
	for (int i = 0; i < 10; ++i) {
		double v = (i < 4) ? 0.12 : 0.01;  // 40% above threshold
		detector->updateOdometry(v, 1.0 + i * 0.01);
		detector->updateRange(1.0, 1.0 + i * 0.01);
	}
	ASSERT_FALSE(detector->isStationary(1.2));

	// Move into hysteresis middle band (20% above threshold): should stay moving.
	for (int i = 0; i < 10; ++i) {
		double v = (i == 1 || i == 5) ? 0.12 : 0.01;
		detector->updateOdometry(v, 2.0 + i * 0.01);
		detector->updateRange(1.0, 2.0 + i * 0.01);
	}

	EXPECT_FALSE(detector->isStationary(2.2));
}

TEST_F(StaticStationaryDetectorTest, VelocityThresholdBoundaryCountsAsExceed)
{
	detector->updateOdometry(0.1, 1.0);
	detector->updateOdometry(0.05, 1.1);
	detector->updateOdometry(0.03, 1.2);

	detector->updateRange(1.0, 1.0);
	detector->updateRange(1.0, 1.1);
	detector->updateRange(1.0, 1.2);

	EXPECT_FALSE(detector->isStationary(1.3));
}

TEST_F(StaticStationaryDetectorTest, RangeVarianceThresholdIsStrictlyLessThan)
{
	detector = std::make_unique<StaticStationaryDetector>(0.5, 0.1, 0.0, 32, 32);
	detector->updateOdometry(0.01, 1.0);
	detector->updateOdometry(0.02, 1.1);
	detector->updateOdometry(0.03, 1.2);

	detector->updateRange(1.0, 1.0);
	detector->updateRange(1.0, 1.1);
	detector->updateRange(1.0, 1.2);

	EXPECT_FALSE(detector->isStationary(1.3));
}

