// MinHeap.hpp
//
// A binary heap: a priority queue where top() is always the "smallest"
// element according to a comparator you choose.
//
// Who uses it: Dijkstra (include/fp/algo/Dijkstra.hpp) and the stop-limited
// search (include/fp/algo/KStopSearch.hpp). Their whole loop is "give me the
// cheapest unsettled airport", and a heap answers that in O(log n) instead of
// scanning every airport in O(n).
//
// How a binary heap is stored: as a plain array where the children of index
// i are at 2i+1 and 2i+2, and the parent of i is at (i-1)/2.
//
//                 [0]            heap rule: every parent is <= its children,
//               /     \          so the smallest element is always at [0].
//            [1]       [2]
//           /   \     /   \      array: [0][1][2][3][4][5][6]
//         [3]   [4] [5]   [6]
//
//   push: put the new element at the end, then "sift up" (swap with its
//         parent while it is smaller).                       O(log n)
//   pop:  move the last element to [0], then "sift down" (swap with the
//         smaller child while a child is smaller).           O(log n)
//   top:  read [0].                                          O(1)
//
// The comparator: Compare(a, b) returns true if `a` must come out BEFORE `b`.
// std::less<T> (the default) gives a min-heap. The algorithms pass their own
// comparator built from a cost policy (include/fp/algo/CostPolicies.hpp),
// which is how one Dijkstra handles "shortest", "fewest flights" and
// "shortest, then fewest flights" without being rewritten.
//
// No decrease-key: Dijkstra uses "lazy deletion" instead. When it finds a
// better cost for an airport it just pushes a new entry and later skips the
// stale one when it pops it. Simpler code, same O(m log n) bound, because
// log(m) <= 2 log(n) for a graph with m edges and n nodes.

#pragma once

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "fp/Errors.hpp"  // EmptyContainer

namespace fp {

template <typename T, typename Compare = std::less<T>>
class MinHeap {
public:
    MinHeap() = default;
    explicit MinHeap(Compare compare) : before_(std::move(compare)) {}

    void push(T value) {
        items_.push_back(std::move(value));
        siftUp(items_.size() - 1);
    }

    // Remove and return the element that comes out first.
    T pop() {
        if (items_.empty()) {
            throw EmptyContainer("MinHeap::pop");
        }
        T result = std::move(items_.front());
        // Move the last element into the hole at the root, shrink, and
        // restore the heap rule by sifting it down.
        items_.front() = std::move(items_.back());
        items_.pop_back();
        if (!items_.empty()) {
            siftDown(0);
        }
        return result;
    }

    const T& top() const {
        if (items_.empty()) {
            throw EmptyContainer("MinHeap::top");
        }
        return items_.front();
    }

    bool empty() const { return items_.empty(); }
    std::size_t size() const { return items_.size(); }
    void reserve(std::size_t n) { items_.reserve(n); }
    void clear() { items_.clear(); }

private:
    static std::size_t parent(std::size_t i) { return (i - 1) / 2; }
    static std::size_t left(std::size_t i) { return 2 * i + 1; }

    // Bubble the element at i upwards until its parent is not "after" it.
    void siftUp(std::size_t i) {
        while (i > 0) {
            std::size_t p = parent(i);
            if (!before_(items_[i], items_[p])) {
                break;  // heap rule already holds
            }
            std::swap(items_[i], items_[p]);
            i = p;
        }
    }

    // Push the element at i downwards, always swapping with the child that
    // comes out first, until neither child should come out before it.
    void siftDown(std::size_t i) {
        const std::size_t n = items_.size();
        while (true) {
            std::size_t best = i;
            std::size_t l = left(i);
            std::size_t r = l + 1;
            if (l < n && before_(items_[l], items_[best])) best = l;
            if (r < n && before_(items_[r], items_[best])) best = r;
            if (best == i) {
                break;
            }
            std::swap(items_[i], items_[best]);
            i = best;
        }
    }

    // std::vector is only the storage here; all the heap logic is above.
    std::vector<T> items_;
    Compare before_{};
};

}  // namespace fp
