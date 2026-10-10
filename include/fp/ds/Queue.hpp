// Queue.hpp
//
// A first-in, first-out queue built on a growable circular buffer.
//
// Who uses it: BFS (include/fp/algo/BFS.hpp) and the hub analysis
// (src/analysis/HubAnalysis.cpp). BFS is the "ripple" algorithm: airports
// are processed in the exact order they were discovered, which is what a
// FIFO queue gives you.
//
// Why a circular buffer and not a linked list:
//   * One contiguous block of memory, so it is cache friendly.
//   * No allocation per push. We only allocate when the buffer is full,
//     and then we double it, so push is O(1) amortised.
//   * pop is O(1): we just move the "head" index forward and wrap around.
//
// Picture with capacity 8, size 3, head at index 6:
//
//     index:  0   1   2   3   4   5   6   7
//     data : [c] [ ] [ ] [ ] [ ] [ ] [a] [b]
//                                     ^head
//   front() is 'a'. The next push goes to (head + size) % capacity = 1.
//
// Requirement on T: default constructible and move assignable (true for the
// ints we store). This keeps the buffer a plain array.

#pragma once

#include <cstddef>
#include <memory>
#include <utility>

#include "fp/Errors.hpp"  // EmptyContainer

namespace fp {

template <typename T>
class Queue {
public:
    Queue() = default;

    // The buffer is owned by a unique_ptr, so copying is disabled on purpose:
    // a silent deep copy of a big BFS frontier would be a performance bug.
    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;

    // Moving is cheap: steal the buffer and leave `other` empty but valid.
    Queue(Queue&& other) noexcept
        : data_(std::move(other.data_)),
          capacity_(std::exchange(other.capacity_, 0)),
          head_(std::exchange(other.head_, 0)),
          size_(std::exchange(other.size_, 0)) {}

    Queue& operator=(Queue&& other) noexcept {
        if (this != &other) {
            data_ = std::move(other.data_);
            capacity_ = std::exchange(other.capacity_, 0);
            head_ = std::exchange(other.head_, 0);
            size_ = std::exchange(other.size_, 0);
        }
        return *this;
    }

    // Add an element at the back.
    void push(T value) {
        if (size_ == capacity_) {
            grow(capacity_ == 0 ? kInitialCapacity : capacity_ * 2);
        }
        data_[(head_ + size_) % capacity_] = std::move(value);
        ++size_;
    }

    // Remove and return the element at the front.
    T pop() {
        if (size_ == 0) {
            throw EmptyContainer("Queue::pop");
        }
        T value = std::move(data_[head_]);
        head_ = (head_ + 1) % capacity_;
        --size_;
        return value;
    }

    // Look at the front element without removing it.
    const T& front() const {
        if (size_ == 0) {
            throw EmptyContainer("Queue::front");
        }
        return data_[head_];
    }

    bool empty() const { return size_ == 0; }
    std::size_t size() const { return size_; }
    std::size_t capacity() const { return capacity_; }

    // Make room for at least `n` elements up front (avoids repeated growth
    // when the caller knows roughly how many items are coming).
    void reserve(std::size_t n) {
        if (n > capacity_) {
            grow(n);
        }
    }

    // Forget all elements but keep the buffer for reuse.
    void clear() {
        head_ = 0;
        size_ = 0;
    }

private:
    static constexpr std::size_t kInitialCapacity = 16;

    // Allocate a bigger buffer and copy the elements over in queue order,
    // so in the new buffer the front sits at index 0 again ("unwrapping").
    void grow(std::size_t newCapacity) {
        std::unique_ptr<T[]> bigger(new T[newCapacity]);
        for (std::size_t i = 0; i < size_; ++i) {
            bigger[i] = std::move(data_[(head_ + i) % capacity_]);
        }
        data_ = std::move(bigger);
        capacity_ = newCapacity;
        head_ = 0;
    }

    std::unique_ptr<T[]> data_;
    std::size_t capacity_ = 0;
    std::size_t head_ = 0;  // index of the front element
    std::size_t size_ = 0;  // number of stored elements
};

}  // namespace fp
