// HubAnalysis.hpp
//
// The project's "why": how connected is a city, and which big airports is it
// quietly depending on?
//
// Three questions, all built on BFS (include/fp/algo/BFS.hpp):
//
//   1. reachability(): what can I reach in 1, 2, 3 ... flights?
//      One BFS, grouped by ripple.
//
//   2. hubFailureImpact(): what do I lose if airport H shuts down?
//      Run BFS normally, run it again with H blocked (SearchOptions::blocked),
//      compare. Airports that vanish are "lost", airports that now need more
//      flights are "slower". This is the HARD measure: it is exactly what
//      happens if H closes.
//
//   3. rankHubs(): which airports carry most of my fewest-flight routes?
//      For every destination t, look at ALL fewest-flight routes from the
//      origin to t, and give each airport v on them a share equal to the
//      fraction of those routes that pass through v. Add the shares up over
//      all destinations. This is the "single-source dependency" from
//      Brandes' betweenness-centrality algorithm (U. Brandes, "A Faster
//      Algorithm for Betweenness Centrality", 2001) and needs only one BFS
//      plus one backwards pass, O(n + m), instead of enumerating paths.
//      This is the SOFT measure: how much traffic would naturally route
//      through v. The top few by share then get the hard measure as well.
//
// Implementation: src/analysis/HubAnalysis.cpp

#pragma once

#include <cstddef>
#include <vector>

#include "fp/graph/FlightGraph.hpp"

namespace fp {

// layers[h] = airports whose fewest-flight distance from the origin is h.
// layers[0] is just the origin.
struct ReachLayers {
    std::vector<std::vector<int>> layers;

    // Number of destinations (every reached airport except the origin).
    std::size_t destinations() const;
};

// maxHops >= 1 required (-1 means no limit).
ReachLayers reachability(const FlightGraph& graph, int origin, int maxHops);

struct HubImpact {
    int hub = -1;
    // Counts exclude the origin and the hub itself, so they answer "how many
    // OTHER airports can I still reach?".
    std::size_t reachableBefore = 0;
    std::size_t reachableAfter = 0;
    std::vector<int> lost;    // reachable before, unreachable (within maxHops) after
    std::vector<int> slower;  // still reachable, but needs more flights now
};

// Throws InvalidQuery if hub == origin.
HubImpact hubFailureImpact(const FlightGraph& graph, int origin, int hub, int maxHops);

struct HubReliance {
    int hub = -1;
    double dependentDestinations = 0.0;  // Brandes dependency (fractional count)
    double share = 0.0;                  // dependentDestinations / all destinations
    std::size_t lostIfClosed = 0;        // from hubFailureImpact
    std::size_t slowerIfClosed = 0;
};

// Per-airport Brandes dependency of `origin`, restricted to destinations
// within maxHops. result[v] = expected number of destinations whose
// fewest-flight route connects through v. result[origin] = 0.
std::vector<double> fewestFlightDependency(const FlightGraph& graph, int origin, int maxHops);

// Top `topN` connecting airports for `origin`, by dependency share, each with
// its failure impact filled in.
std::vector<HubReliance> rankHubs(const FlightGraph& graph, int origin, int maxHops,
                                  std::size_t topN);

}  // namespace fp
