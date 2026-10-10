// Dijkstra.hpp
//
// Dijkstra's algorithm (E. W. Dijkstra, 1959): cheapest path when every
// flight has a non-negative cost.
//
// The one-line idea: always continue from the cheapest airport you have not
// settled yet. When an airport is popped from the heap as the cheapest
// unsettled one, its cost is final. Any other way to reach it would have to
// go through some airport that is at least as expensive, and costs never go
// down along a path (rule 1 in CostPolicies.hpp), so it cannot be cheaper.
//
// Why Dijkstra and not the alternatives, for "shortest route":
//   * BFS:            ignores distances. Only right when every flight costs 1.
//   * Bellman-Ford:   O(n * m). Its strength is negative edges, which flights
//                     never have. We still use it as a test oracle
//                     (BellmanFord.hpp).
//   * Floyd-Warshall: all pairs at once, O(n^3). With ~6,000 airports that is
//                     about 2 * 10^11 steps to answer one question.
//   * A*:             needs a heuristic that never overestimates. Would work
//                     for distance (straight-line distance is a valid lower
//                     bound), but not for the other policies. Possible extension.
//
// Heap strategy: "lazy deletion". When we find a cheaper cost for an airport
// we push a new heap entry instead of updating the old one; when an outdated
// entry is popped later we skip it. The heap can hold up to m entries, so the
// running time is O(m log m) = O(m log n), because m < n^2 means
// log m < 2 log n.
//
// Space: O(n) for the cost and parent arrays, O(m) worst case for the heap.
//
// This file is header-only because it is a template: the compiler needs the
// full code to generate one version per cost policy.

#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "fp/algo/CostPolicies.hpp"  // DistancePolicy, isReached, ...
#include "fp/algo/Itinerary.hpp"     // Itinerary, SearchOptions, walkParents
#include "fp/ds/MinHeap.hpp"         // MinHeap (our binary heap)
#include "fp/graph/FlightGraph.hpp"

namespace fp {

// Result of a full Dijkstra run: best cost and parent for every airport.
template <typename Policy>
struct ShortestPathTree {
    std::vector<typename Policy::Cost> cost;  // Policy::infinity() if unreached
    std::vector<int> parent;                  // previous airport on a best path, -1 if none

    bool reached(int id) const { return isReached<Policy>(cost[static_cast<std::size_t>(id)]); }
};

namespace detail {

// One heap entry: "airport `node` can be reached for `cost`".
template <typename Policy>
struct DijkstraEntry {
    typename Policy::Cost cost;
    int node;
};

// Heap order: cheaper cost comes out first. This is where the policy's
// comparator plugs into our MinHeap.
template <typename Policy>
struct CheaperFirst {
    bool operator()(const DijkstraEntry<Policy>& a, const DijkstraEntry<Policy>& b) const {
        return Policy::less(a.cost, b.cost);
    }
};

}  // namespace detail

// Run Dijkstra from `src`. If `target` is not -1, stop as soon as the target
// is settled (its answer cannot change after that point).
template <typename Policy>
ShortestPathTree<Policy> dijkstraTree(const FlightGraph& graph, int src, int target = -1,
                                      const SearchOptions& options = {}) {
    using Cost = typename Policy::Cost;
    using Entry = detail::DijkstraEntry<Policy>;

    graph.requireValidId(src, "origin");
    if (target != -1) graph.requireValidId(target, "target");
    validateOptions(graph, options);

    const std::size_t n = graph.airportCount();
    ShortestPathTree<Policy> tree;
    tree.cost.assign(n, Policy::infinity());
    tree.parent.assign(n, -1);
    if (isBlocked(options, src)) return tree;

    std::vector<bool> settled(n, false);
    MinHeap<Entry, detail::CheaperFirst<Policy>> heap;

    tree.cost[static_cast<std::size_t>(src)] = Policy::zero();
    heap.push({Policy::zero(), src});
    if (options.stats) ++options.stats->heapPushes;

    while (!heap.empty()) {
        const Entry top = heap.pop();
        const std::size_t u = static_cast<std::size_t>(top.node);

        // Lazy deletion: skip entries for airports already settled with a
        // cost at least as good.
        if (settled[u]) continue;
        settled[u] = true;
        if (options.stats) ++options.stats->nodesSettled;
        if (top.node == target) break;  // its cost is now final

        for (const Edge& e : graph.neighbors(top.node)) {
            if (options.stats) ++options.stats->edgesRelaxed;
            const std::size_t v = static_cast<std::size_t>(e.to);
            if (settled[v] || isBlocked(options, e.to)) continue;

            // "Relaxation": is going through u a better way to reach v?
            const Cost candidate = Policy::extend(top.cost, e);
            if (Policy::less(candidate, tree.cost[v])) {
                tree.cost[v] = candidate;
                tree.parent[v] = top.node;
                heap.push({candidate, e.to});
                if (options.stats) ++options.stats->heapPushes;
            }
        }
    }
    return tree;
}

// Cheapest route from src to dst under `Policy`, or std::nullopt if there is
// none. Example:  dijkstra<DistancePolicy>(graph, bho, jfk)
template <typename Policy>
std::optional<Itinerary> dijkstra(const FlightGraph& graph, int src, int dst,
                                  const SearchOptions& options = {}) {
    graph.requireValidId(src, "origin");
    graph.requireValidId(dst, "destination");
    validateOptions(graph, options);
    if (isBlocked(options, src) || isBlocked(options, dst)) return std::nullopt;
    if (src == dst) return makeItinerary(graph, {src});

    ShortestPathTree<Policy> tree = dijkstraTree<Policy>(graph, src, dst, options);
    if (!tree.reached(dst)) return std::nullopt;
    return makeItinerary(graph, walkParents(tree.parent, src, dst));
}

}  // namespace fp
