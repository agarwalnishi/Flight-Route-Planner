// BFS.cpp
//
// Implementation of include/fp/algo/BFS.hpp. Complexity: O(n + m) time,
// O(n) extra space (the three arrays in BfsTree plus the queue).

#include "fp/algo/BFS.hpp"

#include "fp/Errors.hpp"    // InvalidQuery
#include "fp/ds/Queue.hpp"  // Queue (our circular-buffer FIFO)

namespace fp {

BfsTree bfsTree(const FlightGraph& graph, int src, int maxHops, int target,
                const SearchOptions& options) {
    graph.requireValidId(src, "origin");
    if (target != -1) graph.requireValidId(target, "target");
    if (maxHops < -1) {
        throw InvalidQuery("maxHops must be -1 (no limit) or >= 0");
    }
    validateOptions(graph, options);  // Itinerary.hpp

    const std::size_t n = graph.airportCount();
    BfsTree tree;
    tree.hops.assign(n, -1);
    tree.parent.assign(n, -1);

    // A closed origin reaches nothing (not even itself).
    if (isBlocked(options, src)) return tree;

    Queue<int> queue;
    queue.reserve(n);  // never holds more than n airports, so no regrowth
    tree.hops[static_cast<std::size_t>(src)] = 0;
    tree.order.push_back(src);
    queue.push(src);

    while (!queue.empty()) {
        const int u = queue.pop();
        const int hopsU = tree.hops[static_cast<std::size_t>(u)];
        if (options.stats) ++options.stats->nodesSettled;
        if (maxHops != -1 && hopsU >= maxHops) continue;  // reached the ripple limit

        for (const Edge& e : graph.neighbors(u)) {
            if (options.stats) ++options.stats->edgesRelaxed;
            const std::size_t v = static_cast<std::size_t>(e.to);
            if (tree.hops[v] != -1 || isBlocked(options, e.to)) continue;  // seen or closed

            // First time we see v, so this is its fewest-flight distance.
            tree.hops[v] = hopsU + 1;
            tree.parent[v] = u;
            tree.order.push_back(e.to);
            if (e.to == target) return tree;  // early exit: target's answer is final
            queue.push(e.to);
            if (options.stats) ++options.stats->heapPushes;
        }
    }
    return tree;
}

std::optional<Itinerary> fewestFlights(const FlightGraph& graph, int src, int dst,
                                       const SearchOptions& options) {
    graph.requireValidId(src, "origin");
    graph.requireValidId(dst, "destination");
    validateOptions(graph, options);
    if (isBlocked(options, src) || isBlocked(options, dst)) return std::nullopt;
    if (src == dst) return makeItinerary(graph, {src});

    BfsTree tree = bfsTree(graph, src, /*maxHops=*/-1, /*target=*/dst, options);
    if (tree.hops[static_cast<std::size_t>(dst)] == -1) return std::nullopt;
    return makeItinerary(graph, walkParents(tree.parent, src, dst));  // Itinerary.hpp
}

}  // namespace fp
