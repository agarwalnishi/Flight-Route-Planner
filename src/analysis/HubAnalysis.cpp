// HubAnalysis.cpp
//
// Implementation of include/fp/analysis/HubAnalysis.hpp. Every function here
// is BFS (src/algo/BFS.cpp) plus some bookkeeping.

#include "fp/analysis/HubAnalysis.hpp"

#include <algorithm>

#include "fp/Errors.hpp"       // InvalidQuery
#include "fp/algo/BFS.hpp"     // bfsTree, BfsTree
#include "fp/algo/Itinerary.hpp"  // SearchOptions

namespace fp {
namespace {

void requireHopLimit(int maxHops) {
    if (maxHops != -1 && maxHops < 1) {
        throw InvalidQuery("max hops must be at least 1 (or -1 for no limit)");
    }
}

}  // namespace

std::size_t ReachLayers::destinations() const {
    std::size_t total = 0;
    for (std::size_t h = 1; h < layers.size(); ++h) total += layers[h].size();
    return total;
}

ReachLayers reachability(const FlightGraph& graph, int origin, int maxHops) {
    requireHopLimit(maxHops);
    BfsTree tree = bfsTree(graph, origin, maxHops);

    // tree.order is already sorted by hops (BFS visits ripple by ripple),
    // so we can append to the right layer as we go.
    ReachLayers result;
    for (int id : tree.order) {
        const std::size_t h = static_cast<std::size_t>(tree.hops[static_cast<std::size_t>(id)]);
        if (result.layers.size() <= h) result.layers.resize(h + 1);
        result.layers[h].push_back(id);
    }
    return result;
}

HubImpact hubFailureImpact(const FlightGraph& graph, int origin, int hub, int maxHops) {
    requireHopLimit(maxHops);
    graph.requireValidId(origin, "origin");
    graph.requireValidId(hub, "hub");
    if (hub == origin) {
        throw InvalidQuery("the hub to close cannot be the origin itself");
    }

    // Normal world.
    BfsTree before = bfsTree(graph, origin, maxHops);

    // Same world with the hub closed. Blocking is a flag, not a deletion, so
    // the graph is untouched and the next query sees the normal world again.
    std::vector<bool> blocked(graph.airportCount(), false);
    blocked[static_cast<std::size_t>(hub)] = true;
    SearchOptions options;
    options.blocked = &blocked;
    BfsTree after = bfsTree(graph, origin, maxHops, /*target=*/-1, options);

    HubImpact impact;
    impact.hub = hub;
    for (int id : before.order) {
        if (id == origin || id == hub) continue;
        ++impact.reachableBefore;
        const int hopsBefore = before.hops[static_cast<std::size_t>(id)];
        const int hopsAfter = after.hops[static_cast<std::size_t>(id)];
        if (hopsAfter == -1) {
            impact.lost.push_back(id);
        } else if (hopsAfter > hopsBefore) {
            impact.slower.push_back(id);
        }
    }
    for (int id : after.order) {
        if (id != origin) ++impact.reachableAfter;  // hub is blocked, so never in here
    }
    return impact;
}

std::vector<double> fewestFlightDependency(const FlightGraph& graph, int origin, int maxHops) {
    requireHopLimit(maxHops);
    BfsTree tree = bfsTree(graph, origin, maxHops);
    const std::size_t n = graph.airportCount();
    const std::vector<int>& hops = tree.hops;

    // An edge u -> w lies on some fewest-flight route exactly when
    // hops[w] == hops[u] + 1. Those edges form a layered "shortest path DAG".
    auto onShortestRoute = [&hops](int u, int w) {
        const int hw = hops[static_cast<std::size_t>(w)];
        return hw != -1 && hw == hops[static_cast<std::size_t>(u)] + 1;
    };

    // Pass 1 (forward, in BFS order): sigma[v] = how many different
    // fewest-flight routes reach v. Doubles, because counts grow
    // exponentially with the number of layers and would overflow integers.
    std::vector<double> sigma(n, 0.0);
    sigma[static_cast<std::size_t>(origin)] = 1.0;
    for (int u : tree.order) {
        for (const Edge& e : graph.neighbors(u)) {
            if (onShortestRoute(u, e.to)) {
                sigma[static_cast<std::size_t>(e.to)] += sigma[static_cast<std::size_t>(u)];
            }
        }
    }

    // Pass 2 (backward, reverse BFS order): Brandes' recurrence
    //     delta[u] = sum over shortest-route edges u -> w of
    //                (sigma[u] / sigma[w]) * (1 + delta[w])
    // sigma[u] / sigma[w] is the fraction of routes into w that come from u.
    // The "1" counts w itself as a destination; delta[w] counts everything
    // further downstream that routes through w. Processing in reverse order
    // guarantees delta[w] is complete before u needs it.
    std::vector<double> delta(n, 0.0);
    for (auto it = tree.order.rbegin(); it != tree.order.rend(); ++it) {
        const int u = *it;
        const std::size_t ui = static_cast<std::size_t>(u);
        for (const Edge& e : graph.neighbors(u)) {
            if (!onShortestRoute(u, e.to)) continue;
            const std::size_t wi = static_cast<std::size_t>(e.to);
            delta[ui] += (sigma[ui] / sigma[wi]) * (1.0 + delta[wi]);
        }
    }
    // The origin is on every route by definition; reporting that is useless.
    delta[static_cast<std::size_t>(origin)] = 0.0;
    return delta;
}

std::vector<HubReliance> rankHubs(const FlightGraph& graph, int origin, int maxHops,
                                  std::size_t topN) {
    requireHopLimit(maxHops);
    const std::vector<double> delta = fewestFlightDependency(graph, origin, maxHops);
    const std::size_t destinations = reachability(graph, origin, maxHops).destinations();

    std::vector<int> candidates;
    for (std::size_t v = 0; v < delta.size(); ++v) {
        if (delta[v] > 0.0) candidates.push_back(static_cast<int>(v));
    }
    // Highest dependency first; ties broken by code so output is stable.
    std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
        const double da = delta[static_cast<std::size_t>(a)];
        const double db = delta[static_cast<std::size_t>(b)];
        if (da != db) return da > db;
        return graph.airport(a).code < graph.airport(b).code;
    });
    if (candidates.size() > topN) candidates.resize(topN);

    std::vector<HubReliance> ranked;
    for (int hub : candidates) {
        HubReliance r;
        r.hub = hub;
        r.dependentDestinations = delta[static_cast<std::size_t>(hub)];
        r.share = destinations == 0 ? 0.0 : r.dependentDestinations / static_cast<double>(destinations);
        // The soft measure picked the candidates; now apply the hard one.
        const HubImpact impact = hubFailureImpact(graph, origin, hub, maxHops);
        r.lostIfClosed = impact.lost.size();
        r.slowerIfClosed = impact.slower.size();
        ranked.push_back(r);
    }
    return ranked;
}

}  // namespace fp
