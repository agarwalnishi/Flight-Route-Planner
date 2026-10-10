// KStopSearch.hpp
//
// Cheapest route that uses AT MOST K stops (K + 1 flights).
// This is a "constrained shortest path" problem, and plain Dijkstra gets it
// wrong. The tiny example that shows why (limit: 1 stop):
//
//     A -> B  100
//     B -> C  100
//     A -> C  500
//     C -> D  100
//
//   Plain Dijkstra settles C at cost 200 (via B) and never looks at A -> C
//   again. But A -> B -> C -> D has 2 stops, which breaks the limit, and the
//   only legal route, A -> C -> D (600), needed the "worse" way into C that
//   Dijkstra threw away. Result: it wrongly reports "no route".
//   (This exact graph is a unit test in tests/test_algo.cpp.)
//
// The fix: stop treating "being at C" as one situation. Being at C after
// 1 flight and being at C after 2 flights are DIFFERENT states, because they
// leave different amounts of the stop budget. So we run the same Dijkstra on
// states (airport, flights used), with flights used in 0 .. K + 1.
//
// Pruning that keeps it fast in practice ("dominance"):
// states are popped in cost order. If airport v was already settled using f'
// flights, any later state (v, f) with f >= f' costs at least as much AND has
// no more budget left, so nothing it can do beats what (v, f') can do. We skip
// it. Each airport is therefore expanded only when a new pop reaches it with
// strictly fewer flights than before.
//
// Complexity, with F = K + 1:
//   states:  n * (F + 1)               -> O(n K) memory for cost/parent arrays
//   time:    O(K m log(K m)) worst case, usually far less thanks to dominance
//
// Alternative implemented as a cross-check: Bellman-Ford limited to K + 1
// rounds (BellmanFord.hpp), O(K m) with no heap. Simpler, but it always does
// all the rounds over all edges, while this search stops at the target.

#pragma once

#include <algorithm>
#include <climits>
#include <cstddef>
#include <optional>
#include <vector>

#include "fp/Errors.hpp"
#include "fp/algo/CostPolicies.hpp"
#include "fp/algo/Dijkstra.hpp"   // dijkstra (fallback when the limit cannot bind)
#include "fp/algo/Itinerary.hpp"
#include "fp/ds/MinHeap.hpp"
#include "fp/graph/FlightGraph.hpp"

namespace fp {

namespace detail {

template <typename Policy>
struct StateEntry {
    typename Policy::Cost cost;
    int node;
    int flights;  // flights used to reach `node` in this state
};

template <typename Policy>
struct CheaperStateFirst {
    bool operator()(const StateEntry<Policy>& a, const StateEntry<Policy>& b) const {
        return Policy::less(a.cost, b.cost);
    }
};

}  // namespace detail

// Cheapest route from src to dst with at most `maxStops` stops, or
// std::nullopt if no such route exists. Throws InvalidQuery if maxStops < 0.
template <typename Policy>
std::optional<Itinerary> cheapestWithMaxStops(const FlightGraph& graph, int src, int dst,
                                              int maxStops, const SearchOptions& options = {}) {
    using Cost = typename Policy::Cost;
    using Entry = detail::StateEntry<Policy>;

    graph.requireValidId(src, "origin");
    graph.requireValidId(dst, "destination");
    validateOptions(graph, options);
    if (maxStops < 0) {
        throw InvalidQuery("max stops must be >= 0");
    }
    if (isBlocked(options, src) || isBlocked(options, dst)) return std::nullopt;
    if (src == dst) return makeItinerary(graph, {src});

    const long long n = static_cast<long long>(graph.airportCount());
    // A best route never needs to visit an airport twice (costs are
    // non-negative, so a loop can only add cost), so it has at most n - 1
    // flights, i.e. n - 2 stops. A larger limit cannot change the answer, so
    // just run ordinary Dijkstra and skip the bigger state space.
    if (maxStops >= n - 2) {
        return dijkstra<Policy>(graph, src, dst, options);
    }

    const int maxFlights = maxStops + 1;
    const std::size_t width = static_cast<std::size_t>(maxFlights) + 1;  // flights 0..maxFlights
    auto stateOf = [width](int node, int flights) {
        return static_cast<std::size_t>(node) * width + static_cast<std::size_t>(flights);
    };

    const std::size_t nodes = graph.airportCount();
    std::vector<Cost> cost(nodes * width, Policy::infinity());
    std::vector<int> parentState(nodes * width, -1);  // previous STATE index, -1 = start
    // fewestFlightsSettled[v] = smallest flight count with which v was
    // settled so far. Used for the dominance pruning described above.
    std::vector<int> fewestFlightsSettled(nodes, INT_MAX);

    MinHeap<Entry, detail::CheaperStateFirst<Policy>> heap;
    cost[stateOf(src, 0)] = Policy::zero();
    heap.push({Policy::zero(), src, 0});
    if (options.stats) ++options.stats->heapPushes;

    while (!heap.empty()) {
        const Entry top = heap.pop();
        const std::size_t s = stateOf(top.node, top.flights);
        const std::size_t u = static_cast<std::size_t>(top.node);

        if (Policy::less(cost[s], top.cost)) continue;          // stale entry (lazy deletion)
        if (top.flights >= fewestFlightsSettled[u]) continue;   // dominated, see header
        fewestFlightsSettled[u] = top.flights;
        if (options.stats) ++options.stats->nodesSettled;

        if (top.node == dst) {
            // First time the destination is popped: cheapest legal route.
            // Walk the state parents back to the start and keep the airports.
            std::vector<int> path;
            for (long long st = static_cast<long long>(s); st != -1;
                 st = parentState[static_cast<std::size_t>(st)]) {
                path.push_back(static_cast<int>(static_cast<std::size_t>(st) / width));
            }
            std::reverse(path.begin(), path.end());
            return makeItinerary(graph, std::move(path));
        }
        if (top.flights == maxFlights) continue;  // budget used up, cannot fly further

        const int nextFlights = top.flights + 1;
        for (const Edge& e : graph.neighbors(top.node)) {
            if (options.stats) ++options.stats->edgesRelaxed;
            if (isBlocked(options, e.to)) continue;
            // Already settled with no more flights and no more cost? Then this
            // state would be dominated anyway, so do not even push it.
            if (nextFlights >= fewestFlightsSettled[static_cast<std::size_t>(e.to)]) continue;

            const Cost candidate = Policy::extend(top.cost, e);
            const std::size_t t = stateOf(e.to, nextFlights);
            if (Policy::less(candidate, cost[t])) {
                cost[t] = candidate;
                parentState[t] = static_cast<int>(s);
                heap.push({candidate, e.to, nextFlights});
                if (options.stats) ++options.stats->heapPushes;
            }
        }
    }
    return std::nullopt;  // destination not reachable within the stop limit
}

}  // namespace fp
