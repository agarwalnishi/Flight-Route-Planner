// test_ds.cpp
//
// Tests for the hand-written data structures in include/fp/ds/:
// Queue.hpp, MinHeap.hpp, AVLMap.hpp. Where possible each one is compared
// against the standard library version doing the same job, on thousands of
// random operations, because "it works on 3 hand-picked inputs" is weak
// evidence for a data structure.

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "TestHarness.hpp"
#include "fp/Errors.hpp"
#include "fp/ds/AVLMap.hpp"
#include "fp/ds/MinHeap.hpp"
#include "fp/ds/Queue.hpp"

using namespace fp;

// ------------------------------------------------------------------- Queue

TEST_CASE(queue_is_fifo) {
    Queue<int> q;
    CHECK(q.empty());
    for (int i = 0; i < 5; ++i) q.push(i);
    CHECK_EQ(q.size(), 5u);
    CHECK_EQ(q.front(), 0);
    for (int i = 0; i < 5; ++i) CHECK_EQ(q.pop(), i);
    CHECK(q.empty());
}

TEST_CASE(queue_wraps_around_and_grows_in_order) {
    // Interleave pushes and pops so head moves past the end of the buffer
    // before the buffer has to grow. Growth must "unwrap" in queue order.
    Queue<int> q;
    std::deque<int> reference;
    int next = 0;
    for (int round = 0; round < 200; ++round) {
        for (int i = 0; i < 7; ++i) {
            q.push(next);
            reference.push_back(next++);
        }
        for (int i = 0; i < 5; ++i) {
            CHECK_EQ(q.pop(), reference.front());
            reference.pop_front();
        }
    }
    CHECK_EQ(q.size(), reference.size());
    while (!q.empty()) {
        CHECK_EQ(q.pop(), reference.front());
        reference.pop_front();
    }
}

TEST_CASE(queue_empty_access_throws) {
    Queue<int> q;
    CHECK_THROWS_AS(q.pop(), EmptyContainer);
    CHECK_THROWS_AS(q.front(), EmptyContainer);
    q.push(1);
    q.pop();
    CHECK_THROWS_AS(q.pop(), EmptyContainer);
}

TEST_CASE(queue_move_leaves_source_empty) {
    Queue<int> a;
    for (int i = 0; i < 40; ++i) a.push(i);
    Queue<int> b = std::move(a);
    CHECK(a.empty());  // NOLINT: moved-from is specified as empty
    CHECK_EQ(b.size(), 40u);
    CHECK_EQ(b.pop(), 0);
    a.push(7);  // moved-from queue must still be usable
    CHECK_EQ(a.pop(), 7);
}

// ----------------------------------------------------------------- MinHeap

TEST_CASE(heap_pops_in_sorted_order_random) {
    std::mt19937 rng(12345);
    for (int trial = 0; trial < 50; ++trial) {
        MinHeap<int> heap;
        std::vector<int> values;
        const int count = static_cast<int>(rng() % 300);
        for (int i = 0; i < count; ++i) {
            const int v = static_cast<int>(rng() % 100) - 50;  // duplicates on purpose
            heap.push(v);
            values.push_back(v);
        }
        std::sort(values.begin(), values.end());
        for (int expected : values) CHECK_EQ(heap.pop(), expected);
        CHECK(heap.empty());
    }
}

TEST_CASE(heap_interleaved_push_pop_matches_std_priority_queue_order) {
    std::mt19937 rng(777);
    MinHeap<int> heap;
    std::multiset<int> reference;
    for (int step = 0; step < 5000; ++step) {
        if (reference.empty() || rng() % 3 != 0) {
            const int v = static_cast<int>(rng() % 1000);
            heap.push(v);
            reference.insert(v);
        } else {
            CHECK_EQ(heap.top(), *reference.begin());
            CHECK_EQ(heap.pop(), *reference.begin());
            reference.erase(reference.begin());
        }
        CHECK_EQ(heap.size(), reference.size());
    }
}

TEST_CASE(heap_custom_comparator_makes_max_heap) {
    MinHeap<int, std::greater<int>> heap;
    for (int v : {3, 9, 1, 7, 5}) heap.push(v);
    CHECK_EQ(heap.pop(), 9);
    CHECK_EQ(heap.pop(), 7);
    CHECK_EQ(heap.pop(), 5);
}

TEST_CASE(heap_empty_access_throws) {
    MinHeap<int> heap;
    CHECK_THROWS_AS(heap.pop(), EmptyContainer);
    CHECK_THROWS_AS(heap.top(), EmptyContainer);
}

// ------------------------------------------------------------------ AVLMap

TEST_CASE(avl_basic_insert_find) {
    AVLMap<std::string, int> m;
    CHECK(m.insert("DEL", 1));
    CHECK(m.insert("BHO", 2));
    CHECK(!m.insert("DEL", 99));  // duplicate rejected...
    CHECK_EQ(m.at("DEL"), 1);     // ...and the old value is kept
    m.insertOrAssign("DEL", 5);
    CHECK_EQ(m.at("DEL"), 5);
    CHECK_EQ(m.size(), 2u);
    CHECK(m.find("XYZ") == nullptr);
    CHECK_THROWS_AS(m.at("XYZ"), KeyNotFound);
    CHECK(m.checkInvariants());
}

TEST_CASE(avl_sorted_inserts_stay_logarithmic) {
    // Sorted input turns a plain BST into a linked list of height n.
    // An AVL tree must stay below about 1.44 * log2(n + 2).
    AVLMap<int, int> m;
    const int n = 100000;
    for (int i = 0; i < n; ++i) m.insert(i, i);
    CHECK(m.checkInvariants());
    const double bound = 1.4405 * std::log2(n + 2.0);
    CHECK(m.height() <= static_cast<int>(bound));
    CHECK_EQ(m.size(), static_cast<std::size_t>(n));
}

TEST_CASE(avl_random_matches_std_map) {
    std::mt19937 rng(2024);
    AVLMap<int, int> m;
    std::map<int, int> reference;
    for (int step = 0; step < 20000; ++step) {
        const int k = static_cast<int>(rng() % 5000);
        const int v = static_cast<int>(rng());
        if (rng() % 2) {
            CHECK_EQ(m.insert(k, v), reference.emplace(k, v).second);
        } else {
            m.insertOrAssign(k, v);
            reference[k] = v;
        }
    }
    CHECK(m.checkInvariants());
    CHECK_EQ(m.size(), reference.size());

    // In-order traversal must equal std::map's order exactly.
    std::vector<std::pair<int, int>> walked;
    m.forEach([&](int k, int v) {
        walked.emplace_back(k, v);
        return true;
    });
    const std::vector<std::pair<int, int>> expected(reference.begin(), reference.end());
    CHECK(walked == expected);
}

TEST_CASE(avl_for_each_from_supports_prefix_search) {
    AVLMap<std::string, int> m;
    for (const char* code : {"DEN", "DEL", "BHO", "DED", "DFW", "BOM", "DXB"}) m.insert(code, 0);

    std::vector<std::string> found;
    m.forEachFrom("DE", [&](const std::string& k, int) {
        if (k.compare(0, 2, "DE") != 0) return false;  // left the prefix range
        found.push_back(k);
        return true;
    });
    CHECK(found == (std::vector<std::string>{"DED", "DEL", "DEN"}));

    // Starting past every key visits nothing.
    int visited = 0;
    m.forEachFrom("ZZZ", [&](const std::string&, int) {
        ++visited;
        return true;
    });
    CHECK_EQ(visited, 0);
}
