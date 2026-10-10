// test_random_oracle.cpp
//
// The strongest tests in the project. On thousands of small random graphs,
// three completely different methods must agree on the cheapest cost:
//
//   1. Brute force: list EVERY simple path and take the cheapest. Obviously
//      correct, hopelessly slow, fine for 8 airports.
//   2. Bellman-Ford limited to K + 1 rounds (BellmanFord.hpp).
//   3. The fast algorithms: BFS, Dijkstra, state-space Dijkstra.
//
// Why only simple paths in the brute force: with non-negative costs a loop
// can only add cost and flights, so some cheapest route never repeats an
// airport. (Same argument as in KStopSearch.hpp.)
//
// Every returned itinerary is also validated: it must start and end at the
// right airports, use only real flights, respect the stop limit, and its
// recomputed cost must equal the optimum.

#include <functional>
#include <optional>
#include <random>
#include <vector>

#include "GraphBuilders.hpp"
#include "TestHarness.hpp"
#include "fp/algo/BFS.hpp"
#include "fp/algo/BellmanFord.hpp"
#include "fp/algo/CostPolicies.hpp"
#include "fp/algo/Dijkstra.hpp"
#include "fp/algo/KStopSearch.hpp"

using namespace fp;

namespace {

// best[t][f] = cheapest cost to reach t using EXACTLY f flights on a simple
// path (infinity if impossible). One DFS per origin covers every target and
// every flight count at once.
template <typename Policy>
std::vector<std::vector<typename Policy::Cost>> bruteForce(const FlightGraph& g, int src,
                                                           const std::vector<bool>& blocked) {
    using Cost = typename Policy::Cost;
    const std::size_t n = g.airportCount();
    std::vector<std::vector<Cost>> best(n, std::vector<Cost>(n, Policy::infinity()));
    if (blocked[static_cast<std::size_t>(src)]) return best;

    std::vector<bool> onPath(n, false);
    std::function<void(int, int, Cost)> dfs = [&](int u, int flights, Cost cost) {
        Cost& slot = best[static_cast<std::size_t>(u)][static_cast<std::size_t>(flights)];
        if (Policy::less(cost, slot)) slot = cost;
        onPath[static_cast<std::size_t>(u)] = true;
        for (const Edge& e : g.neighbors(u)) {
            const std::size_t v = static_cast<std::size_t>(e.to);
            if (!onPath[v] && !blocked[v]) dfs(e.to, flights + 1, Policy::extend(cost, e));
        }
        onPath[static_cast<std::size_t>(u)] = false;
    };
    dfs(src, 0, Policy::zero());
    return best;
}

// Cheapest using AT MOST maxFlights flights, from the brute-force table.
template <typename Policy>
typename Policy::Cost bestWithin(const std::vector<typename Policy::Cost>& row, int maxFlights) {
    typename Policy::Cost result = Policy::infinity();
    for (int f = 0; f <= maxFlights && f < static_cast<int>(row.size()); ++f) {
        if (Policy::less(row[static_cast<std::size_t>(f)], result)) result = row[static_cast<std::size_t>(f)];
    }
    return result;
}

// Checks one algorithm answer against the brute-force optimum.
template <typename Policy>
void checkAnswer(const FlightGraph& g, const std::optional<Itinerary>& it, int src, int dst,
                 typename Policy::Cost expected, int maxFlights, const std::vector<bool>& blocked) {
    const bool reachable = isReached<Policy>(expected);
    CHECK_EQ(it.has_value(), reachable);
    if (!it || !reachable) return;
    REQUIRE(!it->path.empty());
    CHECK_EQ(it->path.front(), src);
    CHECK_EQ(it->path.back(), dst);
    CHECK(it->flights() <= maxFlights);
    for (int id : it->path) CHECK(!blocked[static_cast<std::size_t>(id)]);
    // pathCost throws if any leg is not a real flight.
    CHECK(pathCost<Policy>(g, it->path) == expected);
}

template <typename Policy>
void runPolicy(std::mt19937& rng, int trials) {
    for (int trial = 0; trial < trials; ++trial) {
        const int n = 1 + static_cast<int>(rng() % 8);
        const double density = 0.15 + 0.6 * static_cast<double>(rng() % 100) / 100.0;
        const FlightGraph g = testutil::randomGraph(rng, n, density, 20);

        // Sometimes close a random airport to exercise SearchOptions::blocked.
        std::vector<bool> blocked(static_cast<std::size_t>(n), false);
        if (n > 2 && rng() % 3 == 0) blocked[rng() % static_cast<unsigned>(n)] = true;
        SearchOptions options;
        options.blocked = &blocked;

        const int src = static_cast<int>(rng() % static_cast<unsigned>(n));
        const auto table = bruteForce<Policy>(g, src, blocked);

        for (int dst = 0; dst < n; ++dst) {
            const auto& row = table[static_cast<std::size_t>(dst)];
            const bool dstBlocked = blocked[static_cast<std::size_t>(dst)];

            // Unconstrained Dijkstra vs brute force over all simple paths.
            auto unconstrained = bestWithin<Policy>(row, n);
            if (dstBlocked) unconstrained = Policy::infinity();
            checkAnswer<Policy>(g, dijkstra<Policy>(g, src, dst, options), src, dst, unconstrained, n,
                                blocked);

            // Every stop limit from 0 up to "cannot bind".
            for (int k = 0; k <= n; ++k) {
                auto expected = bestWithin<Policy>(row, k + 1);
                if (dstBlocked) expected = Policy::infinity();

                checkAnswer<Policy>(g, cheapestWithMaxStops<Policy>(g, src, dst, k, options), src, dst,
                                    expected, k + 1, blocked);

                const auto bf = bellmanFordMaxStops<Policy>(g, src, dst, k, options);
                CHECK_EQ(bf.has_value(), isReached<Policy>(expected));
                if (bf && isReached<Policy>(expected)) CHECK(*bf == expected);
            }
        }
    }
}

}  // namespace

TEST_CASE(oracle_distance_policy) {
    std::mt19937 rng(1);
    runPolicy<DistancePolicy>(rng, 1500);
}

TEST_CASE(oracle_distance_then_flights_policy) {
    std::mt19937 rng(2);
    runPolicy<DistanceThenFlightsPolicy>(rng, 1000);
}

TEST_CASE(oracle_flights_policy) {
    std::mt19937 rng(3);
    runPolicy<FlightsPolicy>(rng, 800);
}

TEST_CASE(oracle_bfs_equals_dijkstra_with_flights_policy) {
    std::mt19937 rng(4);
    for (int trial = 0; trial < 1000; ++trial) {
        const int n = 1 + static_cast<int>(rng() % 10);
        const FlightGraph g = testutil::randomGraph(rng, n, 0.3, 50);
        const int src = static_cast<int>(rng() % static_cast<unsigned>(n));
        const BfsTree tree = bfsTree(g, src);
        const ShortestPathTree<FlightsPolicy> spt = dijkstraTree<FlightsPolicy>(g, src);
        for (int v = 0; v < n; ++v) {
            const int bfsHops = tree.hops[static_cast<std::size_t>(v)];
            if (bfsHops == -1) {
                CHECK(!spt.reached(v));
            } else {
                CHECK_EQ(spt.cost[static_cast<std::size_t>(v)], bfsHops);
                std::optional<Itinerary> it = fewestFlights(g, src, v);
                REQUIRE(it.has_value());
                CHECK_EQ(it->flights(), bfsHops);
            }
        }
    }
}
