// test_algo.cpp
//
// Hand-built cases for the route algorithms in include/fp/algo/:
// BFS, Dijkstra (all three cost policies), the stop-limited search and
// Bellman-Ford. Each graph here is small enough to check by eye, and each
// one targets a specific way an implementation can be wrong.

#include <optional>
#include <vector>

#include "GraphBuilders.hpp"
#include "TestHarness.hpp"
#include "fp/Errors.hpp"
#include "fp/algo/BFS.hpp"
#include "fp/algo/BellmanFord.hpp"
#include "fp/algo/CostPolicies.hpp"
#include "fp/algo/Dijkstra.hpp"
#include "fp/algo/KStopSearch.hpp"

using namespace fp;
using testutil::makeGraph;

namespace {
// A=0, B=1, C=2, D=3, E=4 for readability.
constexpr int A = 0, B = 1, C = 2, D = 3, E = 4;
}  // namespace

// The counterexample from KStopSearch.hpp and the README:
//   A -> B 100, B -> C 100, A -> C 500, C -> D 100, limit 1 stop.
// Plain Dijkstra keeps only the 200 way into C, which needs too many stops.
TEST_CASE(kstop_counterexample_where_plain_dijkstra_fails) {
    const FlightGraph g = makeGraph(4, {{A, B, 100}, {B, C, 100}, {A, C, 500}, {C, D, 100}});

    // Unconstrained: A B C D, 300.
    std::optional<Itinerary> best = dijkstra<DistancePolicy>(g, A, D);
    REQUIRE(best.has_value());
    CHECK(best->path == (std::vector<int>{A, B, C, D}));
    CHECK_EQ(best->totalMeters, Meters{300});
    CHECK_EQ(best->stops(), 2);

    // At most 1 stop: must find A C D, 600.
    std::optional<Itinerary> limited = cheapestWithMaxStops<DistancePolicy>(g, A, D, 1);
    REQUIRE(limited.has_value());
    CHECK(limited->path == (std::vector<int>{A, C, D}));
    CHECK_EQ(limited->totalMeters, Meters{600});

    // Bellman-Ford agrees on both.
    CHECK_EQ(*bellmanFordMaxStops<DistancePolicy>(g, A, D, 1), Meters{600});
    CHECK_EQ(*bellmanFordMaxStops<DistancePolicy>(g, A, D, 2), Meters{300});

    // 0 stops: there is no direct A -> D flight.
    CHECK(!cheapestWithMaxStops<DistancePolicy>(g, A, D, 0).has_value());
    CHECK(!bellmanFordMaxStops<DistancePolicy>(g, A, D, 0).has_value());
}

TEST_CASE(bfs_finds_fewest_flights_not_shortest_distance) {
    // A -> D direct is long (1000) but 1 flight; A -> B -> C -> D is short (30).
    const FlightGraph g = makeGraph(4, {{A, D, 1000}, {A, B, 10}, {B, C, 10}, {C, D, 10}});
    std::optional<Itinerary> fewest = fewestFlights(g, A, D);
    REQUIRE(fewest.has_value());
    CHECK_EQ(fewest->flights(), 1);
    CHECK_EQ(fewest->totalMeters, Meters{1000});

    std::optional<Itinerary> shortest = dijkstra<DistancePolicy>(g, A, D);
    REQUIRE(shortest.has_value());
    CHECK_EQ(shortest->flights(), 3);
    CHECK_EQ(shortest->totalMeters, Meters{30});
}

TEST_CASE(bfs_layers_and_hop_limit) {
    //   A -> B -> C -> D, and A -> E
    const FlightGraph g = makeGraph(5, {{A, B, 1}, {B, C, 1}, {C, D, 1}, {A, E, 1}});
    const BfsTree full = bfsTree(g, A);
    CHECK(full.hops == (std::vector<int>{0, 1, 2, 3, 1}));
    CHECK_EQ(full.parent[static_cast<std::size_t>(D)], C);

    const BfsTree limited = bfsTree(g, A, /*maxHops=*/2);
    CHECK_EQ(limited.hops[static_cast<std::size_t>(C)], 2);
    CHECK_EQ(limited.hops[static_cast<std::size_t>(D)], -1);  // beyond the limit

    // Visit order is sorted by hops (Brandes relies on this).
    for (std::size_t i = 1; i < full.order.size(); ++i) {
        CHECK(full.hops[static_cast<std::size_t>(full.order[i - 1])] <=
              full.hops[static_cast<std::size_t>(full.order[i])]);
    }
}

TEST_CASE(lexicographic_policy_breaks_distance_ties_by_flights) {
    // Two routes of equal length 200: A B C D (3 flights) and A E D (2 flights).
    // Edges added in an order that tempts a naive tie-break to pick the
    // longer one first.
    const FlightGraph g =
        makeGraph(5, {{A, B, 50}, {B, C, 50}, {C, D, 100}, {A, E, 100}, {E, D, 100}});
    std::optional<Itinerary> it = dijkstra<DistanceThenFlightsPolicy>(g, A, D);
    REQUIRE(it.has_value());
    CHECK_EQ(it->totalMeters, Meters{200});
    CHECK_EQ(it->flights(), 2);
    CHECK(it->path == (std::vector<int>{A, E, D}));

    const DistanceThenFlightsPolicy::Cost expected{200, 2};
    CHECK_EQ(pathCost<DistanceThenFlightsPolicy>(g, it->path), expected);
}

TEST_CASE(zero_length_edges_are_fine) {
    const FlightGraph g = makeGraph(3, {{A, B, 0}, {B, C, 0}, {A, C, 5}});
    std::optional<Itinerary> it = dijkstra<DistancePolicy>(g, A, C);
    REQUIRE(it.has_value());
    CHECK_EQ(it->totalMeters, Meters{0});
}

TEST_CASE(same_origin_and_destination_is_an_empty_trip) {
    const FlightGraph g = makeGraph(2, {{A, B, 7}});
    for (const auto& it : {fewestFlights(g, A, A), dijkstra<DistancePolicy>(g, A, A),
                           cheapestWithMaxStops<DistancePolicy>(g, A, A, 0)}) {
        REQUIRE(it.has_value());
        CHECK_EQ(it->flights(), 0);
        CHECK_EQ(it->totalMeters, Meters{0});
    }
}

TEST_CASE(unreachable_returns_nullopt_not_exception) {
    // B -> A exists, A -> B does not: direction matters.
    const FlightGraph g = makeGraph(2, {{B, A, 10}});
    CHECK(!fewestFlights(g, A, B).has_value());
    CHECK(!dijkstra<DistancePolicy>(g, A, B).has_value());
    CHECK(!cheapestWithMaxStops<DistancePolicy>(g, A, B, 3).has_value());
    CHECK(!bellmanFordMaxStops<DistancePolicy>(g, A, B, 3).has_value());
}

TEST_CASE(blocked_airport_is_avoided) {
    //   A -> B -> D  (short, through B)
    //   A -> C -> D  (long, through C)
    const FlightGraph g = makeGraph(4, {{A, B, 10}, {B, D, 10}, {A, C, 50}, {C, D, 50}});
    std::vector<bool> blocked(4, false);
    blocked[static_cast<std::size_t>(B)] = true;
    SearchOptions options;
    options.blocked = &blocked;

    std::optional<Itinerary> it = dijkstra<DistancePolicy>(g, A, D, options);
    REQUIRE(it.has_value());
    CHECK(it->path == (std::vector<int>{A, C, D}));

    std::optional<Itinerary> k = cheapestWithMaxStops<DistancePolicy>(g, A, D, 1, options);
    REQUIRE(k.has_value());
    CHECK_EQ(k->totalMeters, Meters{100});
    CHECK_EQ(*bellmanFordMaxStops<DistancePolicy>(g, A, D, 1, options), Meters{100});

    // A blocked destination is simply unreachable.
    blocked[static_cast<std::size_t>(D)] = true;
    CHECK(!fewestFlights(g, A, D, options).has_value());

    // A blocked list of the wrong size is a caller error.
    std::vector<bool> wrongSize(2, false);
    options.blocked = &wrongSize;
    CHECK_THROWS_AS(dijkstra<DistancePolicy>(g, A, D, options), InvalidQuery);
}

TEST_CASE(invalid_queries_throw) {
    const FlightGraph g = makeGraph(2, {{A, B, 1}});
    CHECK_THROWS_AS(fewestFlights(g, A, 9), InvalidQuery);
    CHECK_THROWS_AS(dijkstra<DistancePolicy>(g, -1, B), InvalidQuery);
    CHECK_THROWS_AS(cheapestWithMaxStops<DistancePolicy>(g, A, B, -1), InvalidQuery);
    CHECK_THROWS_AS(bellmanFordMaxStops<DistancePolicy>(g, A, B, -1), InvalidQuery);
    CHECK_THROWS_AS(bfsTree(g, A, -5), InvalidQuery);
}

TEST_CASE(large_stop_limit_falls_back_to_plain_dijkstra) {
    const FlightGraph g = makeGraph(4, {{A, B, 100}, {B, C, 100}, {A, C, 500}, {C, D, 100}});
    // n = 4, so any limit >= 2 cannot bind; both must give the 300 route.
    for (int k : {2, 3, 1000000}) {
        std::optional<Itinerary> it = cheapestWithMaxStops<DistancePolicy>(g, A, D, k);
        REQUIRE(it.has_value());
        CHECK_EQ(it->totalMeters, Meters{300});
    }
}

TEST_CASE(search_stats_count_work) {
    const FlightGraph g = makeGraph(4, {{A, B, 1}, {B, C, 1}, {C, D, 1}});
    SearchStats stats;
    SearchOptions options;
    options.stats = &stats;
    (void)dijkstra<DistancePolicy>(g, A, D, options);
    CHECK_EQ(stats.nodesSettled, 4u);
    CHECK(stats.edgesRelaxed >= 3u);
}
