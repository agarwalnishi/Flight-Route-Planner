// Itinerary.hpp
//
// What every route-finding algorithm returns, plus the small helpers they
// all share. Included by BFS.hpp, Dijkstra.hpp, KStopSearch.hpp and
// BellmanFord.hpp in this folder.
//
// Vocabulary used everywhere in this project:
//   flights = number of edges on the path      BHO -> DEL -> JFK = 2 flights
//   stops   = airports you land at in between  BHO -> DEL -> JFK = 1 stop
//   so stops = flights - 1 (and a direct flight has 0 stops).

#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

#include "fp/Errors.hpp"             // InvalidQuery
#include "fp/graph/FlightGraph.hpp"  // FlightGraph, Edge, Meters

namespace fp {

// A concrete answer: the airports to fly through, in order, and the totals.
struct Itinerary {
    std::vector<int> path;  // airport ids; path.front() = origin, path.back() = destination
    Meters totalMeters = 0;

    int flights() const { return path.empty() ? 0 : static_cast<int>(path.size()) - 1; }
    int stops() const { return std::max(0, flights() - 1); }
};

// Optional counters so the benchmark (bench/bench_queries.cpp) can show how
// much work each algorithm actually did, not just how long it took.
struct SearchStats {
    std::size_t nodesSettled = 0;   // pops that were not stale or dominated
    std::size_t edgesRelaxed = 0;   // edges looked at
    std::size_t heapPushes = 0;     // or queue pushes, for BFS
};

// Knobs shared by every search.
struct SearchOptions {
    // If set, blocked[id] == true means "pretend this airport is closed".
    // Used by the hub-failure analysis (src/analysis/HubAnalysis.cpp) to ask
    // "what if Delhi shuts down?" without copying or editing the graph.
    const std::vector<bool>* blocked = nullptr;

    // If set, the search adds its work counters here.
    SearchStats* stats = nullptr;
};

inline bool isBlocked(const SearchOptions& opt, int id) {
    return opt.blocked && (*opt.blocked)[static_cast<std::size_t>(id)];
}

// Every search calls this first, so a blocked list of the wrong size is
// reported as a clear error instead of an out-of-bounds read.
inline void validateOptions(const FlightGraph& graph, const SearchOptions& opt) {
    if (opt.blocked && opt.blocked->size() != graph.airportCount()) {
        throw InvalidQuery("blocked list must have one entry per airport");
    }
}

// Follow parent pointers back from `dst` to `src` and return the path in
// forward order. parent[src] must be -1. Shared by BFS and Dijkstra.
inline std::vector<int> walkParents(const std::vector<int>& parent, int src, int dst) {
    std::vector<int> path;
    for (int v = dst; v != -1; v = parent[static_cast<std::size_t>(v)]) {
        path.push_back(v);
    }
    std::reverse(path.begin(), path.end());
    if (path.empty() || path.front() != src) {
        throw InvalidQuery("internal error: parent chain does not reach the origin");
    }
    return path;
}

// Turn a list of airport ids into an Itinerary with its total distance,
// reading each leg from the graph (FlightGraph::edgeBetween). Throws if two
// consecutive airports are not connected, which would be an algorithm bug;
// the tests rely on this to validate every returned path.
inline Itinerary makeItinerary(const FlightGraph& graph, std::vector<int> path) {
    Itinerary it;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const Edge* e = graph.edgeBetween(path[i - 1], path[i]);
        if (!e) {
            throw InvalidQuery("path uses a flight that does not exist");
        }
        it.totalMeters += e->meters;
    }
    it.path = std::move(path);
    return it;
}

// Cost of a given path under any cost policy (include/fp/algo/CostPolicies.hpp).
// Lets tests check that the path an algorithm returns really costs what the
// algorithm claims is optimal.
template <typename Policy>
typename Policy::Cost pathCost(const FlightGraph& graph, const std::vector<int>& path) {
    typename Policy::Cost cost = Policy::zero();
    for (std::size_t i = 1; i < path.size(); ++i) {
        const Edge* e = graph.edgeBetween(path[i - 1], path[i]);
        if (!e) {
            throw InvalidQuery("path uses a flight that does not exist");
        }
        cost = Policy::extend(cost, *e);
    }
    return cost;
}

}  // namespace fp
