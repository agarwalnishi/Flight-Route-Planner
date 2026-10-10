// FlightGraph.hpp
//
// The map of the world's flights: airports are nodes, routes are directed,
// weighted edges. Every algorithm in include/fp/algo/ and the hub analysis
// in include/fp/analysis/ takes one of these as input.
//
// Two layers, on purpose:
//
//   1. Name layer (used rarely, at the edges of the program)
//      An AVLMap from airport code to internal id: "BHO" -> 1432.
//      See include/fp/ds/AVLMap.hpp for why it is a balanced tree.
//
//   2. Number layer (used in every inner loop)
//      Airports are numbered 0 .. n-1. The adjacency list is
//      std::vector<std::vector<Edge>>, so "who can I fly to from airport 7"
//      is a direct array index. No hashing, no tree, no strings.
//
// Memory: O(n + m) for n airports and m distinct directed routes.
//
// Lifecycle:
//   addAirport(...) for every airport   -> returns its id
//   addRoute(...)   for every flight    -> duplicates allowed
//   finalize()                          -> sort + merge duplicates
// After finalize(), every adjacency list is sorted by destination id and
// holds at most one edge per destination. edgeBetween() relies on that.

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "fp/ds/AVLMap.hpp"     // AVLMap (code -> id)
#include "fp/graph/Airport.hpp" // Airport
#include "fp/graph/Edge.hpp"    // Edge, Meters

namespace fp {

class FlightGraph {
public:
    // Add an airport and return its new id (ids are assigned 0, 1, 2, ...).
    // Throws InvalidQuery if the code is empty or already used.
    int addAirport(Airport airport);

    // Add a directed route from -> to with the given length.
    // Duplicates (several airlines flying the same pair) are merged later by
    // finalize(). Throws InvalidQuery for unknown ids or a negative length,
    // because Dijkstra is only correct with non-negative costs.
    void addRoute(int from, int to, Meters meters);

    // Sort each adjacency list by destination and merge duplicate edges
    // (keeping the shortest length and adding up the airline counts).
    // Safe to call more than once.
    void finalize();

    // Name layer --------------------------------------------------------

    // Code -> id. Accepts any case ("bho" works). Throws UnknownAirport.
    int idOf(const std::string& code) const;

    // Code -> id, or std::nullopt if unknown. Never throws.
    std::optional<int> findId(const std::string& code) const;

    // All airport ids whose code starts with `prefix`, in code order.
    // Uses AVLMap::forEachFrom, so it costs O(log n + matches).
    std::vector<int> idsWithPrefix(const std::string& prefix, std::size_t limit) const;

    // Number layer ------------------------------------------------------

    const Airport& airport(int id) const;
    const std::vector<Edge>& neighbors(int id) const;

    // The edge from -> to, or nullptr if there is no direct flight.
    // Binary search in the sorted adjacency list: O(log degree).
    // Used to rebuild itinerary totals (include/fp/algo/Itinerary.hpp).
    const Edge* edgeBetween(int from, int to) const;

    std::size_t airportCount() const { return airports_.size(); }
    std::size_t routeCount() const;  // distinct directed edges
    bool isFinalized() const { return finalized_; }

    // Throws InvalidQuery if id is not in [0, airportCount()).
    void requireValidId(int id, const char* what) const;

private:
    std::vector<Airport> airports_;            // indexed by id
    std::vector<std::vector<Edge>> adjacency_; // adjacency_[id] = outgoing edges
    AVLMap<std::string, int> idByCode_;        // "BHO" -> id
    bool finalized_ = true;                    // an empty graph is trivially finalized
};

// Upper-case a code so lookups are case-insensitive ("del" == "DEL").
// Defined in src/graph/FlightGraph.cpp.
std::string normalizeCode(const std::string& code);

}  // namespace fp
