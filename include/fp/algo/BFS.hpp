// BFS.hpp
//
// Breadth-first search: the "ripples in water" algorithm.
//
//   ripple 0: the origin
//   ripple 1: every airport with a direct flight from the origin
//   ripple 2: every NEW airport one flight beyond ripple 1
//   ...
//
// The ripple an airport first appears in is the fewest number of flights
// needed to reach it. That is guaranteed because every closer ripple was
// finished before this one started, which is exactly what a first-in,
// first-out queue (include/fp/ds/Queue.hpp) enforces.
//
// Why BFS and not Dijkstra for "fewest flights": every flight counts as 1,
// so the queue is already in cost order and a heap would only add a log
// factor. BFS is O(n + m); Dijkstra would be O(m log n) for the same answer.
//
// Used by:
//   * the CLI `route --by flights`              (src/main.cpp)
//   * reachability and hub analysis             (src/analysis/HubAnalysis.cpp)
// Implementation: src/algo/BFS.cpp

#pragma once

#include <optional>
#include <vector>

#include "fp/algo/Itinerary.hpp"     // Itinerary, SearchOptions
#include "fp/graph/FlightGraph.hpp"

namespace fp {

// Everything one BFS run learns, indexed by airport id.
struct BfsTree {
    std::vector<int> hops;    // flights needed from the origin, or -1 if not reached
    std::vector<int> parent;  // previous airport on one fewest-flight path, -1 if none
    std::vector<int> order;   // reached airports in the order BFS visited them,
                              // which is sorted by `hops` (Brandes needs this order)
};

// Run BFS from `src`.
//   maxHops: stop expanding after this many flights (-1 = no limit).
//   target:  stop as soon as this airport is discovered (-1 = explore everything).
// Throws InvalidQuery for bad ids or maxHops < -1.
BfsTree bfsTree(const FlightGraph& graph, int src, int maxHops = -1, int target = -1,
                const SearchOptions& options = {});

// Fewest-flight route from src to dst, or std::nullopt if unreachable.
std::optional<Itinerary> fewestFlights(const FlightGraph& graph, int src, int dst,
                                       const SearchOptions& options = {});

}  // namespace fp
