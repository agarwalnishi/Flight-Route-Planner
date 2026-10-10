// BellmanFord.hpp
//
// Bellman-Ford limited to a number of rounds (R. Bellman 1958, L. Ford 1956).
//
// Role in this project: an independent second opinion. It solves the same
// "cheapest with at most K stops" problem as KStopSearch.hpp in a completely
// different way, so if both agree on thousands of random graphs
// (tests/test_random_oracle.cpp), we can trust both. The CLI also exposes it
// through `route ... --verify` (src/main.cpp).
//
// The idea in one picture. Round r answers "cheapest cost using at most r
// flights", built only from round r - 1:
//
//     round 0: origin = 0, everything else = infinity
//     round 1: every direct flight from the origin
//     round 2: extend every round-1 answer by one more flight
//     ...
//     round K + 1: cheapest with at most K + 1 flights = at most K stops
//
// The detail that makes the limit work: each round relaxes edges using a
// COPY of the previous round's costs. Classic Bellman-Ford updates in place,
// so one round can chain several flights together and quietly break the
// flight limit. Using the copy guarantees one extra flight per round.
//
// Complexity: O((K + 1) * m) time, O(n) space. No heap, no early exit.
// That is the trade-off versus the state-space Dijkstra: simpler and
// predictable, but it always scans every edge in every round.

#pragma once

#include <algorithm>
#include <optional>
#include <vector>

#include "fp/Errors.hpp"
#include "fp/algo/CostPolicies.hpp"
#include "fp/algo/Itinerary.hpp"     // SearchOptions, isBlocked
#include "fp/graph/FlightGraph.hpp"

namespace fp {

// Cheapest COST (no path, it is an oracle) from src to dst with at most
// maxStops stops, or std::nullopt if unreachable within the limit.
template <typename Policy>
std::optional<typename Policy::Cost> bellmanFordMaxStops(const FlightGraph& graph, int src,
                                                         int dst, int maxStops,
                                                         const SearchOptions& options = {}) {
    using Cost = typename Policy::Cost;

    graph.requireValidId(src, "origin");
    graph.requireValidId(dst, "destination");
    validateOptions(graph, options);
    if (maxStops < 0) {
        throw InvalidQuery("max stops must be >= 0");
    }
    if (isBlocked(options, src) || isBlocked(options, dst)) return std::nullopt;

    const std::size_t n = graph.airportCount();
    std::vector<Cost> previous(n, Policy::infinity());
    previous[static_cast<std::size_t>(src)] = Policy::zero();

    // More than n - 1 rounds can never help (see KStopSearch.hpp for why a
    // best route has at most n - 1 flights).
    const long long rounds = std::min<long long>(static_cast<long long>(maxStops) + 1,
                                                 static_cast<long long>(n) - 1);

    for (long long round = 0; round < rounds; ++round) {
        std::vector<Cost> current = previous;  // the copy that enforces "one more flight"
        bool changed = false;
        for (std::size_t u = 0; u < n; ++u) {
            if (!isReached<Policy>(previous[u]) || isBlocked(options, static_cast<int>(u))) continue;
            for (const Edge& e : graph.neighbors(static_cast<int>(u))) {
                if (options.stats) ++options.stats->edgesRelaxed;
                if (isBlocked(options, e.to)) continue;
                const Cost candidate = Policy::extend(previous[u], e);  // from PREVIOUS round
                const std::size_t v = static_cast<std::size_t>(e.to);
                if (Policy::less(candidate, current[v])) {
                    current[v] = candidate;
                    changed = true;
                }
            }
        }
        previous = std::move(current);
        if (!changed) break;  // nothing improved, later rounds cannot either
    }

    const Cost& answer = previous[static_cast<std::size_t>(dst)];
    if (!isReached<Policy>(answer)) return std::nullopt;
    return answer;
}

}  // namespace fp
