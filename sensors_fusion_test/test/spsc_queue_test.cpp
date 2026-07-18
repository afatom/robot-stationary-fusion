#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "udemy_ros2_pkg/SPSCQueue.hpp"

namespace
{

template<typename T>
void expect_queue_size(const SPSCQueue<T> & queue, size_t expected_size)
{
	EXPECT_EQ(queue.size(), expected_size);
	EXPECT_EQ(queue.empty(), expected_size == 0U);
}

}  // namespace

TEST(SPSCQueueTest, ConstructorRoundsCapacityToPowerOfTwo)
{
	SPSCQueue<int> queue_small(0);
	EXPECT_EQ(queue_small.capacity(), 2U);

	SPSCQueue<int> queue_exact(8);
	EXPECT_EQ(queue_exact.capacity(), 8U);

	SPSCQueue<int> queue_rounded(9);
	EXPECT_EQ(queue_rounded.capacity(), 16U);
}

TEST(SPSCQueueTest, StartsEmpty)
{
	SPSCQueue<int> queue(4);
	expect_queue_size(queue, 0U);
}

TEST(SPSCQueueTest, PushPopSingleElement)
{
	SPSCQueue<int> queue(4);

	EXPECT_TRUE(queue.push(42));
	expect_queue_size(queue, 1U);

	int value = 0;
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 42);
	expect_queue_size(queue, 0U);
}

TEST(SPSCQueueTest, FifoOrderIsPreserved)
{
	SPSCQueue<int> queue(4);

	EXPECT_TRUE(queue.push(1));
	EXPECT_TRUE(queue.push(2));
	EXPECT_TRUE(queue.push(3));

	int value = 0;
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 1);
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 2);
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 3);
	EXPECT_FALSE(queue.pop(value));
}

TEST(SPSCQueueTest, PushFailsWhenQueueIsFull)
{
	SPSCQueue<int> queue(2);

	EXPECT_TRUE(queue.push(10));
	EXPECT_TRUE(queue.push(20));
	EXPECT_FALSE(queue.push(30));

	int value = 0;
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 10);
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 20);
	EXPECT_FALSE(queue.pop(value));
}

TEST(SPSCQueueTest, PopFailsWhenQueueIsEmpty)
{
	SPSCQueue<int> queue(4);

	int value = 0;
	EXPECT_FALSE(queue.pop(value));
	EXPECT_TRUE(queue.empty());
	EXPECT_EQ(queue.size(), 0U);
}

TEST(SPSCQueueTest, RelativeAccessReturnsChronologicalOrder)
{
	SPSCQueue<int> queue(4);

	EXPECT_TRUE(queue.push(10));
	EXPECT_TRUE(queue.push(20));
	EXPECT_TRUE(queue.push(30));

	int value = 0;
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 10);

	EXPECT_TRUE(queue.push(40));

	EXPECT_EQ(queue.get_relative(0), 20);
	EXPECT_EQ(queue.get_relative(1), 30);
	EXPECT_EQ(queue.get_relative(2), 40);
	EXPECT_EQ(queue.get_relative(3), 0);
}

TEST(SPSCQueueTest, WrapAroundMaintainsOrder)
{
	SPSCQueue<int> queue(2);

	EXPECT_TRUE(queue.push(1));
	EXPECT_TRUE(queue.push(2));

	int value = 0;
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 1);

	EXPECT_TRUE(queue.push(3));
	EXPECT_FALSE(queue.push(4));

	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 2);
	EXPECT_TRUE(queue.pop(value));
	EXPECT_EQ(value, 3);
	EXPECT_FALSE(queue.pop(value));
}

TEST(SPSCQueueTest, SupportsMoveOnlyTypes)
{
	SPSCQueue<std::unique_ptr<int>> queue(2);

	auto input = std::make_unique<int>(77);
	EXPECT_TRUE(queue.push(std::move(input)));
	EXPECT_EQ(input, nullptr);

	std::unique_ptr<int> output;
	EXPECT_TRUE(queue.pop(output));
	ASSERT_NE(output, nullptr);
	EXPECT_EQ(*output, 77);
}

TEST(SPSCQueueTest, ConcurrentSingleProducerSingleConsumerPreservesOrder)
{
	constexpr int item_count = 5000;
	SPSCQueue<int> queue(128);
	std::vector<int> received;
	received.reserve(item_count);

	std::atomic<bool> producer_done{false};

	std::thread producer([&]() {
		for (int i = 0; i < item_count; ++i) {
			while (!queue.push(i)) {
				std::this_thread::yield();
			}
		}
		producer_done.store(true, std::memory_order_release);
	});

	std::thread consumer([&]() {
		int value = 0;
		while (!producer_done.load(std::memory_order_acquire) || !queue.empty()) {
			if (queue.pop(value)) {
				received.push_back(value);
			} else {
				std::this_thread::yield();
			}
		}
	});

	producer.join();
	consumer.join();

	ASSERT_EQ(received.size(), static_cast<size_t>(item_count));
	for (int i = 0; i < item_count; ++i) {
		EXPECT_EQ(received[static_cast<size_t>(i)], i);
	}
}


