// test_analysis.cpp
//
// Tests for the connectivity questions in src/analysis/HubAnalysis.cpp:
// reachability by ripple, the "what if this hub closes" impact, and the
// Brandes dependency score. The Brandes score is checked against a brute
// force that literally lists every fewest-flight route, because the
// sigma/delta recurrence is easy to get subtly wrong.

#include <cmath>
#include <functional>
#include <random>
#include <vector>

#include "GraphBuilders.hpp"
#include "TestHarness.hpp"
#include "fp/Errors.hpp"
#include "fp/analysis/HubAnalysis.hpp"

using namespace fp;
using testutil::makeGraph;

namespace {

// A small "tier-2 city" shaped network:
//
//   ORIGIN(0) -> HUB1(1) -> X(3), Y(4)
//   ORIGIN(0) -> HUB2(2) -> Y(4), Z(5)
//   X(3) -> FAR(6)
//
// X and FAR are only reachable through HUB1, Z only through HUB2,
// Y through either.
FlightGraph tierTwoCity() {
    return makeGraph(7, {{0, 1, 1}, {0, 2, 1}, {1, 3, 1}, {1, 4, 1}, {2, 4, 1}, {2, 5, 1}, {3, 6, 1}});
}

// Independent fewest-flight distances (repeated relaxation, no BFS code).
std::vector<int> bruteHops(const FlightGraph& g, int src, int maxHops) {
    const int n = static_cast<int>(g.airportCount());
    std::vector<int> hops(static_cast<std::size_t>(n), -1);
    hops[static_cast<std::size_t>(src)] = 0;
    for (int round = 0; round < n; ++round) {
        for (int u = 0; u < n; ++u) {
            const int hu = hops[static_cast<std::size_t>(u)];
            if (hu == -1 || (maxHops != -1 && hu >= maxHops)) continue;
            for (const Edge& e : g.neighbors(u)) {
                int& hv = hops[static_cast<std::size_t>(e.to)];
                if (hv == -1 || hv > hu + 1) hv = hu + 1;
            }
        }
    }
    return hops;
}

// Brute-force dependency: list every fewest-flight route to every target and
// count, for each airport, the fraction of a target's routes passing through it.
std::vector<double> bruteDependency(const FlightGraph& g, int src, int maxHops) {
    const std::size_t n = g.airportCount();
    const std::vector<int> hops = bruteHops(g, src, maxHops);
    std::vector<double> delta(n, 0.0);

    for (std::size_t t = 0; t < n; ++t) {
        if (static_cast<int>(t) == src || hops[t] == -1) continue;
        std::vector<std::vector<int>> routes;
        std::vector<int> path{src};
        // Depth-first walk that only takes edges going exactly one ripple out.
        std::function<void(int)> walk = [&](int u) {
            if (u == static_cast<int>(t)) {
                routes.push_back(path);
                return;
            }
            for (const Edge& e : g.neighbors(u)) {
                const int hv = hops[static_cast<std::size_t>(e.to)];
                if (hv == hops[static_cast<std::size_t>(u)] + 1 && hv <= hops[t]) {
                    path.push_back(e.to);
                    walk(e.to);
                    path.pop_back();
                }
            }
        };
        walk(src);
        for (const std::vector<int>& r : routes) {
            for (std::size_t i = 1; i + 1 < r.size(); ++i) {  // intermediate airports only
                delta[static_cast<std::size_t>(r[i])] += 1.0 / static_cast<double>(routes.size());
            }
        }
    }
    return delta;
}

}  // namespace

TEST_CASE(reachability_groups_by_ripple) {
    const FlightGraph g = tierTwoCity();
    const ReachLayers r = reachability(g, 0, -1);
    REQUIRE(r.layers.size() == 4u);
    CHECK_EQ(r.layers[0].size(), 1u);  // origin
    CHECK_EQ(r.layers[1].size(), 2u);  // HUB1, HUB2
    CHECK_EQ(r.layers[2].size(), 3u);  // X, Y, Z
    CHECK_EQ(r.layers[3].size(), 1u);  // FAR
    CHECK_EQ(r.destinations(), 6u);

    CHECK_EQ(reachability(g, 0, 2).destinations(), 5u);  // FAR is 3 flights away
    CHECK_THROWS_AS(reachability(g, 0, 0), InvalidQuery);
}

TEST_CASE(hub_failure_lists_lost_airports) {
    const FlightGraph g = tierTwoCity();
    const HubImpact closeHub1 = hubFailureImpact(g, 0, 1, -1);
    CHECK_EQ(closeHub1.reachableBefore, 5u);  // 6 destinations minus the hub itself
    CHECK_EQ(closeHub1.reachableAfter, 3u);   // HUB2, Y, Z
    CHECK(closeHub1.lost == (std::vector<int>{3, 6}));  // X and FAR
    CHECK(closeHub1.slower.empty());                    // Y still 2 flights via HUB2

    const HubImpact closeHub2 = hubFailureImpact(g, 0, 2, -1);
    CHECK(closeHub2.lost == (std::vector<int>{5}));     // only Z

    CHECK_THROWS_AS(hubFailureImpact(g, 0, 0, -1), InvalidQuery);
}

TEST_CASE(hub_failure_detects_slower_routes) {
    // 0 -> 1 -> 2 directly, or the long way 0 -> 3 -> 4 -> 2.
    const FlightGraph g = makeGraph(5, {{0, 1, 1}, {1, 2, 1}, {0, 3, 1}, {3, 4, 1}, {4, 2, 1}});
    const HubImpact impact = hubFailureImpact(g, 0, 1, -1);
    CHECK(impact.lost.empty());
    CHECK(impact.slower == (std::vector<int>{2}));

    // With a 2-flight budget, the 3-flight detour does not count: lost.
    const HubImpact tight = hubFailureImpact(g, 0, 1, 2);
    CHECK(tight.lost == (std::vector<int>{2}));
}

TEST_CASE(dependency_on_tier_two_city_by_hand) {
    const FlightGraph g = tierTwoCity();
    const std::vector<double> delta = fewestFlightDependency(g, 0, -1);
    // HUB1 carries: X (1), FAR (1), half of Y (0.5)  = 2.5
    // HUB2 carries: Z (1), half of Y (0.5)           = 1.5
    // X carries:    FAR (1)
    CHECK(std::fabs(delta[1] - 2.5) < 1e-9);
    CHECK(std::fabs(delta[2] - 1.5) < 1e-9);
    CHECK(std::fabs(delta[3] - 1.0) < 1e-9);
    CHECK(std::fabs(delta[4]) < 1e-9);
    CHECK(std::fabs(delta[0]) < 1e-9);  // origin reported as 0
}

TEST_CASE(dependency_matches_brute_force_on_random_graphs) {
    std::mt19937 rng(4242);
    for (int trial = 0; trial < 300; ++trial) {
        const int n = 2 + static_cast<int>(rng() % 7);
        const FlightGraph g = testutil::randomGraph(rng, n, 0.35, 1);
        const int src = static_cast<int>(rng() % static_cast<unsigned>(n));
        for (int maxHops : {-1, 1, 2, 3}) {
            const std::vector<double> fast = fewestFlightDependency(g, src, maxHops);
            const std::vector<double> slow = bruteDependency(g, src, maxHops);
            for (int v = 0; v < n; ++v) {
                if (v == src) continue;
                CHECK(std::fabs(fast[static_cast<std::size_t>(v)] - slow[static_cast<std::size_t>(v)]) < 1e-9);
            }
        }
    }
}

TEST_CASE(rank_hubs_orders_by_share) {
    const FlightGraph g = tierTwoCity();
    const std::vector<HubReliance> ranked = rankHubs(g, 0, -1, 10);
    REQUIRE(ranked.size() == 3u);  // HUB1, HUB2, X carry traffic
    CHECK_EQ(ranked[0].hub, 1);
    CHECK_EQ(ranked[0].lostIfClosed, 2u);
    CHECK(std::fabs(ranked[0].share - 2.5 / 6.0) < 1e-9);
    CHECK_EQ(ranked[1].hub, 2);
    CHECK_EQ(rankHubs(g, 0, -1, 1).size(), 1u);  // topN respected
}
