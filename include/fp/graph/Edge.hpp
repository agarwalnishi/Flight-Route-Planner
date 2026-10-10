// Edge.hpp
//
// One directed edge of the flight graph: "there is at least one scheduled
// flight from the owning airport to `to`".
//
// Edges live in FlightGraph's adjacency lists (include/fp/graph/FlightGraph.hpp).
// The airport the edge starts from is implied by which list it sits in, so
// it is not stored here. That keeps an edge at 16 bytes.
//
// Why distance is an integer number of metres instead of a double of km:
// costs get added up and compared all the time (Dijkstra, Bellman-Ford,
// the "shortest, then fewest flights" tie-break). With integers, a + b + c
// gives the same answer in any order, so two correct algorithms always agree
// exactly and the tests can compare them with ==. With doubles, rounding can
// make two equally short routes look different by 1e-12.

#pragma once

#include <cstdint>

namespace fp {

using Meters = std::int64_t;  // all distances and path costs use this type

struct Edge {
    int to = -1;               // internal id of the destination airport
    std::int32_t airlines = 0; // how many airline routes were merged into this edge
    Meters meters = 0;         // great-circle distance (see geo/Haversine.hpp)
};

}  // namespace fp
