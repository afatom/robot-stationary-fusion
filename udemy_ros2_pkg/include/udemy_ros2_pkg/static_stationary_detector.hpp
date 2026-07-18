#pragma once

#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

#include "SPSCQueue.hpp"

struct StaticTimeStampedValue {
    double value = 0.0;
    double timestamp = 0.0;
};

class StaticStationaryDetector {
public:
    // Sized generously to handle spikes (e.g., up to 2.0 seconds of history)
    StaticStationaryDetector(double window_duration_sec, 
                             double velocity_threshold, 
                             double range_variance_threshold,
                             double moving_ratio_threshold = 0.20,
                             double moving_ratio_hysteresis = 0.05,
                             double variance_hysteresis_ratio = 0.20,
                             size_t max_odom_capacity = 100, 
                             size_t max_range_capacity = 100)
        : window_duration_(window_duration_sec)
        , velocity_th_(velocity_threshold)
        , variance_th_(range_variance_threshold)
        , moving_ratio_threshold_(std::clamp(moving_ratio_threshold, 0.0, 1.0))
        , moving_ratio_hysteresis_(std::clamp(moving_ratio_hysteresis, 0.0, 0.49))
        , variance_hysteresis_ratio_(std::max(0.0, variance_hysteresis_ratio))
        , odom_buffer_(max_odom_capacity)
        , range_buffer_(max_range_capacity) { }

    bool updateOdometry(double velocity_magnitude, double timestamp) {
        return odom_buffer_.push(StaticTimeStampedValue{velocity_magnitude, timestamp}); // O(1) Zero Allocation
    }

    bool updateRange(double range, double timestamp) {
        return range_buffer_.push(StaticTimeStampedValue{range, timestamp});      // O(1) Zero Allocation
    }

    bool isStationary(double current_timestamp) {
        double cutoff_time = current_timestamp - window_duration_;

        // Keep queues bounded to the active decision window.
        pruneOlderThan(odom_buffer_, cutoff_time);
        pruneOlderThan(range_buffer_, cutoff_time);

        // --- 1. Evaluate Kinematics (Odometry) ---
        size_t valid_odom_count = 0;
        size_t odom_above_threshold_count = 0;

        const size_t odom_count = odom_buffer_.size();
        for (size_t i = 0; i < odom_count; ++i) {
            auto sample = odom_buffer_.get_relative(i);
            // Skip data points older than our time window
            if (sample.timestamp < cutoff_time) continue;

            valid_odom_count++;
            if (sample.value >= velocity_th_) {
                odom_above_threshold_count++;
            }
        }

        // If no fresh odometry packets exist, fall back to moving/unsafe state
        if (valid_odom_count == 0) {
            has_last_decision_ = false;
            last_stationary_ = false;
            return false;
        }


        // --- 2. Evaluate Spatial Profile (Range Variance) ---
        size_t valid_range_count = 0;
        double range_sum = 0.0;
        const size_t range_count = range_buffer_.size();

        // Pass 1: Calculate Mean of valid range points in current window
        for (size_t i = 0; i < range_count; ++i) {
            auto sample = range_buffer_.get_relative(i);
            if (sample.timestamp < cutoff_time) continue;

            range_sum += sample.value;
            valid_range_count++;
        }

        // Require a baseline minimum population profile to confidently calculate variance
        if (valid_range_count < 3) {
            has_last_decision_ = false;
            last_stationary_ = false;
            return false;
        }
        double range_mean = range_sum / valid_range_count;

        // Pass 2: Calculate Variance
        double variance_sum = 0.0;
        for (size_t i = 0; i < range_count; ++i) {
            auto sample = range_buffer_.get_relative(i);
            if (sample.timestamp < cutoff_time) continue;

            variance_sum += (sample.value - range_mean) * (sample.value - range_mean);
        }
        double range_variance = variance_sum / valid_range_count;
        const double odom_moving_ratio = static_cast<double>(odom_above_threshold_count) /
            static_cast<double>(valid_odom_count);

        // Hysteresis thresholds for robust state transitions.
        const double enter_stationary_ratio_max = std::max(0.0, moving_ratio_threshold_ - moving_ratio_hysteresis_);
        const double exit_stationary_ratio_min = std::min(1.0, moving_ratio_threshold_ + moving_ratio_hysteresis_);
        const double enter_stationary_variance_max = variance_th_ * std::max(0.0, 1.0 - variance_hysteresis_ratio_);
        const double exit_stationary_variance_min = variance_th_ * (1.0 + variance_hysteresis_ratio_);

        const bool strict_stationary =
            (odom_moving_ratio <= enter_stationary_ratio_max) &&
            (range_variance < enter_stationary_variance_max);
        const bool strict_moving =
            (odom_moving_ratio > exit_stationary_ratio_min) ||
            (range_variance >= exit_stationary_variance_min);

        bool decision = false;
        if (!has_last_decision_) {
            // Conservative initialization: require clear stillness before first YES.
            decision = strict_stationary;
        } else if (last_stationary_) {
            // Stay stationary unless there is clear moving evidence.
            decision = !strict_moving;
        } else {
            // Stay moving until there is clear stationary evidence.
            decision = strict_stationary;
        }

        has_last_decision_ = true;
        last_stationary_ = decision;
        return decision;
    }

private:
    double window_duration_;
    double velocity_th_;
    double variance_th_;
    double moving_ratio_threshold_;
    double moving_ratio_hysteresis_;
    double variance_hysteresis_ratio_;
    bool has_last_decision_ = false;
    bool last_stationary_ = false;

    static void pruneOlderThan(SPSCQueue<StaticTimeStampedValue>& buffer, double cutoff_time) {
        while (!buffer.empty()) {
            const auto oldest = buffer.get_relative(0);
            if (oldest.timestamp >= cutoff_time) {
                break;
            }

            StaticTimeStampedValue discarded;
            if (!buffer.pop(discarded)) {
                break;
            }
        }
    }

    SPSCQueue<StaticTimeStampedValue> odom_buffer_;
    SPSCQueue<StaticTimeStampedValue> range_buffer_;
};
