#pragma once

#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

struct StaticTimeStampedValue {
    double value = 0.0;
    double timestamp = 0.0;
};

// Fixed-capacity Ring Buffer using a contiguous preallocated vector
class PreallocatedRingBuffer {
public:
    PreallocatedRingBuffer();

    // Explicit reserve step executed exclusively at initialization
    void init(size_t capacity);

    void push(double value, double timestamp);

    size_t size() const { return size_; }
    size_t capacity() const { return capacity_; }

    // Safe indexed access relative to the oldest active sample in the queue
    StaticTimeStampedValue get_relative(size_t index) const;

private:
    std::vector<StaticTimeStampedValue> buffer_;
    size_t capacity_;
    size_t head_;
    size_t size_;
};

class StaticStationaryDetector {
public:
    // Sized generously to handle spikes (e.g., up to 2.0 seconds of history)
    StaticStationaryDetector(double window_duration_sec, 
                             double velocity_threshold, 
                             double range_variance_threshold,
                             size_t max_odom_capacity = 100, 
                             size_t max_range_capacity = 100);

    void updateOdometry(double velocity_magnitude, double timestamp);

    void updateRange(double range, double timestamp);

    bool isStationary(double current_timestamp);

private:
    double window_duration_;
    double velocity_th_;
    double variance_th_;

    PreallocatedRingBuffer odom_buffer_;
    PreallocatedRingBuffer range_buffer_;
};
