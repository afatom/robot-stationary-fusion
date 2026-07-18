#pragma once

#include <vector>
#include <atomic>
#include <cstddef>
#include <utility>
#include <new>
#include <bit> // C++20 Required for std::bit_ceil

template <typename T>
class SPSCQueue {
public:
    /**
     * @brief Constructor. Automatically rounds up requested capacity to the nearest power of 2.
     */
    explicit SPSCQueue(size_t requested_capacity) 
        : capacity_(ensure_power_of_two(requested_capacity)), 
          mask_(capacity_ - 1)
    {
        // Pre-allocate continuous memory array up front. No heap changes occur after this.
        buffer_.resize(capacity_);
    }

    // Rule of 5: Prevent accidental copies or moves affecting thread safety boundaries
    SPSCQueue(const SPSCQueue&) = delete;
    SPSCQueue& operator=(const SPSCQueue&) = delete;
    SPSCQueue(SPSCQueue&&) = delete;
    SPSCQueue& operator=(SPSCQueue&&) = delete;
    ~SPSCQueue() = default;

    /**
     * @brief Inserts an element by copying. Drops the item and returns false if the queue is full.
     * @note Producer Thread Only.
     */
    bool push(const T& item) {
        // memory_order_relaxed: Guarantees only that the modification to the variable itself is atomic
        const auto current_tail = tail_.load(std::memory_order_relaxed);
        const auto current_head = head_.load(std::memory_order_acquire);

        // Standard power of 2 boundary block check: If full, do NOT overwrite.
        if ((current_tail - current_head) == capacity_) {
            return false; 
        }

        buffer_[current_tail & mask_] = item;
        // release store ensures all memory writes made here are visible to the consumer thread that does an acquire load
        tail_.store(current_tail + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Inserts an element by moving. Drops the item and returns false if the queue is full.
     * @note Producer Thread Only.
     */
    bool push(T&& item) {
        const auto current_tail = tail_.load(std::memory_order_relaxed);
        const auto current_head = head_.load(std::memory_order_acquire);

        if ((current_tail - current_head) == capacity_) {
            return false;
        }

        buffer_[current_tail & mask_] = std::move(item);
        tail_.store(current_tail + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Extracts the oldest item by copying it out into a local destination block.
     * @return true if an item was successfully popped, false if empty.
     * @note Consumer Thread Only.
     */
    bool pop(T& out_value) {
        const auto current_head = head_.load(std::memory_order_relaxed);
        const auto current_tail = tail_.load(std::memory_order_acquire);

        if (current_head == current_tail) {
            return false; // Queue is entirely empty
        }

        out_value = std::move(buffer_[current_head & mask_]);
        head_.store(current_head + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Returns the item at the given index relative to the head of the queue.
     * @param index The index of the item to retrieve, where 0 is the oldest item, 1 is the next oldest, and so on.
     * @note The index must be less than the current size of the queue.
     * @return The item at the specified index, or a default-constructed T if out of bounds.
     * @note Consumer Thread Only.
     */
    T get_relative(size_t index) const {
        const auto current_head = head_.load(std::memory_order_acquire);
        const auto current_tail = tail_.load(std::memory_order_acquire);
        uint64_t active_size = current_tail - current_head;
        if (index >= active_size) return T();

        // Calculate physical index using fast bitwise masking
        size_t physical_index = static_cast<size_t>((current_head + index) & mask_);
        return buffer_[physical_index];
    }

    bool empty() const noexcept {
        return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
    }

    size_t size() const noexcept {
        const auto current_tail = tail_.load(std::memory_order_relaxed);
        const auto current_head = head_.load(std::memory_order_relaxed);
        return current_tail - current_head;
    }

    size_t capacity() const noexcept {
        return capacity_;
    }

private:
    static size_t ensure_power_of_two(size_t n) noexcept {
        if (n <= 1) return 2;
        return std::bit_ceil(n);
    }

    std::vector<T> buffer_;
    const size_t capacity_;
    const size_t mask_;

#if defined(__cpp_lib_hardware_interference_size)
    static constexpr size_t CacheLineSize = std::hardware_destructive_interference_size;
#else
    static constexpr size_t CacheLineSize = 64; 
#endif

    // Strict cache isolation blocks completely eliminate multi-core memory contention loop thrashing
    alignas(CacheLineSize) std::atomic<size_t> head_{0}; 
    alignas(CacheLineSize) std::atomic<size_t> tail_{0}; 
};
