#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

#include "static_stationary_detector.hpp"


// PreallocatedRingBuffer::PreallocatedRingBuffer() : capacity_(0), head_(0), size_(0) {}

// // Explicit reserve step executed exclusively at initialization
// void PreallocatedRingBuffer::init(size_t capacity) {
//     capacity_ = capacity;
//     buffer_.resize(capacity); // Allocate memory ONCE at boot
//     head_ = 0;
//     size_ = 0;
// }

// void PreallocatedRingBuffer::push(double value, double timestamp) {
//     if (capacity_ == 0) return;

//     buffer_[head_] = {value, timestamp};
//     head_ = (head_ + 1) % capacity_; // Circular wrap

//     if (size_ < capacity_) {
//         size_++;
//     }
// }

// // Safe indexed access relative to the oldest active sample in the queue
// StaticTimeStampedValue PreallocatedRingBuffer::get_relative(size_t index) const {
//     if (index >= size_) return StaticTimeStampedValue();

//     // Locate chronological slot based on whether the buffer has wrapped or not
//     size_t start_idx = 0;
//     if (size_ == capacity_) {
//         start_idx = head_; // When full, head points to the oldest un-overwritten item
//     }

//     size_t target_idx = (start_idx + index) % capacity_;
//     return buffer_[target_idx];
// }



// Sized generously to handle spikes (e.g., up to 2.0 seconds of history)
StaticStationaryDetector::StaticStationaryDetector(double window_duration_sec, 
                        double velocity_threshold, 
                        double range_variance_threshold,
                        size_t max_odom_capacity, 
                        size_t max_range_capacity)
    : window_duration_(window_duration_sec)
    , velocity_th_(velocity_threshold)
    , variance_th_(range_variance_threshold) 
{
    // Allocation occurs strictly inside constructor context during initialization
    odom_buffer_.init(max_odom_capacity);
    range_buffer_.init(max_range_capacity);
}

void StaticStationaryDetector::updateOdometry(double velocity_magnitude, double timestamp) {
    odom_buffer_.push(velocity_magnitude, timestamp); // O(1) Zero Allocation
}

void StaticStationaryDetector::updateRange(double range, double timestamp) {
    range_buffer_.push(range, timestamp);             // O(1) Zero Allocation
}

bool StaticStationaryDetector::isStationary(double current_timestamp) {
    double cutoff_time = current_timestamp - window_duration_;

    // --- 1. Evaluate Kinematics (Odometry) ---
    bool odom_indicates_still = true;
    size_t valid_odom_count = 0;

    for (size_t i = 0; i < odom_buffer_.size(); ++i) {
        auto sample = odom_buffer_.get_relative(i);
        // Skip data points older than our time window
        if (sample.timestamp < cutoff_time) continue;

        valid_odom_count++;
        if (sample.value >= velocity_th_) {
            odom_indicates_still = false;
            break; // Break early if moving
        }
    }

    // If no fresh odometry packets exist, fall back to moving/unsafe state
    if (valid_odom_count == 0) return false;


    // --- 2. Evaluate Spatial Profile (Range Variance) ---
    size_t valid_range_count = 0;
    double range_sum = 0.0;

    // Pass 1: Calculate Mean of valid range points in current window
    for (size_t i = 0; i < range_buffer_.size(); ++i) {
        auto sample = range_buffer_.get_relative(i);
        if (sample.timestamp < cutoff_time) continue;

        range_sum += sample.value;
        valid_range_count++;
    }

    // Require a baseline minimum population profile to confidently calculate variance
    if (valid_range_count < 3) return false;
    double range_mean = range_sum / valid_range_count;

    // Pass 2: Calculate Variance
    double variance_sum = 0.0;
    for (size_t i = 0; i < range_buffer_.size(); ++i) {
        auto sample = range_buffer_.get_relative(i);
        if (sample.timestamp < cutoff_time) continue;

        variance_sum += (sample.value - range_mean) * (sample.value - range_mean);
    }
    double range_variance = variance_sum / valid_range_count;
    bool range_indicates_still = (range_variance < variance_th_);


    // --- 3. Strict Fusion Evaluation ---
    return (odom_indicates_still && range_indicates_still);
}
