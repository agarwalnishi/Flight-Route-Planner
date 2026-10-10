// CostPolicies.hpp
//
// "What does cheapest mean?" lives here, so the search algorithms do not
// have to know.
//
// Dijkstra (Dijkstra.hpp), the stop-limited search (KStopSearch.hpp) and
// Bellman-Ford (BellmanFord.hpp) are all templates over a Policy. A policy
// is a tiny struct with:
//
//   using Cost = ...;                         the type of a path's cost
//   static Cost zero();                       cost of the empty path
//   static Cost infinity();                   "not reached yet"
//   static Cost extend(Cost c, const Edge&);  cost after taking one more flight
//   static bool less(Cost a, Cost b);         is a strictly better than b?
//   static constexpr const char* name;        for printing
//
// The compiler stamps out a separate, fully inlined copy of Dijkstra for
// each policy, so this flexibility costs nothing at run time (unlike passing
// a function pointer or a virtual comparator).
//
// The two rules a policy MUST follow, or Dijkstra gives wrong answers:
//   1. Never decreasing: extend(c, e) is never less than c.
//      (This is the "no negative edges" condition. A path can only get
//      worse as it gets longer, which is what lets Dijkstra lock in an
//      airport's cost the first time it pops it.)
//   2. Order preserving: if a is not worse than b, then extend(a, e) is not
//      worse than extend(b, e). (A better prefix never becomes a worse path.)
// All three policies below are sums of non-negative numbers, so both hold.

#pragma once

#include <limits>
#include <ostream>

#include "fp/graph/Edge.hpp"  // Edge, Meters

namespace fp {

// Shortest total distance flown.
struct DistancePolicy {
    using Cost = Meters;
    static constexpr const char* name = "distance";

    static Cost zero() { return 0; }
    static Cost infinity() { return std::numeric_limits<Meters>::max(); }
    static Cost extend(Cost c, const Edge& e) { return c + e.meters; }
    static bool less(Cost a, Cost b) { return a < b; }
};

// Fewest flights, every edge costs 1. Dijkstra with this policy gives the
// same answer as BFS, just slower (O(m log n) instead of O(n + m)). It exists
// so the tests can check BFS against Dijkstra, and so the stop-limited search
// can be run "by flights" for completeness.
struct FlightsPolicy {
    using Cost = int;
    static constexpr const char* name = "flights";

    static Cost zero() { return 0; }
    static Cost infinity() { return std::numeric_limits<int>::max(); }
    static Cost extend(Cost c, const Edge&) { return c + 1; }
    static bool less(Cost a, Cost b) { return a < b; }
};

// Shortest distance, and among equally short routes, fewer flights.
// Compares (meters, flights) pairs lexicographically, like comparing words
// in a dictionary: look at the first component, use the second only on a tie.
// Because both components are sums of non-negative numbers, the pair is
// still "never decreasing" and Dijkstra stays correct.
struct DistanceThenFlightsPolicy {
    struct Cost {
        Meters meters = 0;
        int flights = 0;
        bool operator==(const Cost& o) const { return meters == o.meters && flights == o.flights; }
        bool operator!=(const Cost& o) const { return !(*this == o); }
    };
    static constexpr const char* name = "distance-then-flights";

    static Cost zero() { return {0, 0}; }
    static Cost infinity() {
        return {std::numeric_limits<Meters>::max(), std::numeric_limits<int>::max()};
    }
    static Cost extend(Cost c, const Edge& e) { return {c.meters + e.meters, c.flights + 1}; }
    static bool less(Cost a, Cost b) {
        if (a.meters != b.meters) return a.meters < b.meters;
        return a.flights < b.flights;
    }
};

// Printing support so test failures show readable values.
inline std::ostream& operator<<(std::ostream& os, const DistanceThenFlightsPolicy::Cost& c) {
    return os << "{" << c.meters << " m, " << c.flights << " flights}";
}

// True if `c` is a real cost and not the "unreached" sentinel.
template <typename Policy>
bool isReached(const typename Policy::Cost& c) {
    return Policy::less(c, Policy::infinity());
}

}  // namespace fp
