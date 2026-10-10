// bench_queries.cpp
//
// Measures every algorithm on the real OpenFlights network, so the numbers
// quoted in the README come from running code, not from guessing.
//
//   ./build/fp_bench [data-dir] [queries]
//
// What it reports per algorithm, averaged over the same random
// origin/destination pairs (fixed seed, so runs are repeatable):
//   * time per query in microseconds
//   * airports settled and edges relaxed per query (SearchStats in
//     include/fp/algo/Itinerary.hpp), which shows WHY one algorithm is
//     faster than another, independent of the machine.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <optional>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "fp/Errors.hpp"
#include "fp/algo/BFS.hpp"
#include "fp/algo/BellmanFord.hpp"
#include "fp/algo/CostPolicies.hpp"
#include "fp/algo/Dijkstra.hpp"
#include "fp/algo/KStopSearch.hpp"
#include "fp/analysis/HubAnalysis.hpp"
#include "fp/io/DataLoader.hpp"

#ifndef FP_DEFAULT_DATA_DIR
#define FP_DEFAULT_DATA_DIR "data"
#endif

using namespace fp;
using Clock = std::chrono::steady_clock;

namespace {

// Runs `query` for every pair and prints one table row.
template <typename Query>
void measure(const std::string& name, const std::vector<std::pair<int, int>>& pairs, Query query) {
    SearchStats stats;
    std::size_t found = 0;
    const auto start = Clock::now();
    for (const auto& [src, dst] : pairs) {
        if (query(src, dst, stats)) ++found;
    }
    const double totalUs = std::chrono::duration<double, std::micro>(Clock::now() - start).count();
    const double q = static_cast<double>(pairs.size());
    // Bellman-Ford never "settles" an airport, so it prints "-" in that column.
    std::ostringstream settled;
    if (stats.nodesSettled == 0) {
        settled << "-";
    } else {
        settled << std::fixed << std::setprecision(1) << static_cast<double>(stats.nodesSettled) / q;
    }
    std::cout << "  " << std::left << std::setw(36) << name << std::right << std::fixed
              << std::setprecision(1) << std::setw(10) << totalUs / q << std::setw(12)
              << settled.str() << std::setw(13)
              << static_cast<double>(stats.edgesRelaxed) / q << std::setw(9) << found << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::string dataDir = argc > 1 ? argv[1] : FP_DEFAULT_DATA_DIR;
        const int queries = argc > 2 ? std::atoi(argv[2]) : 300;

        const auto loadStart = Clock::now();
        LoadStats load;
        const FlightGraph g = loadOpenFlightsDir(dataDir, &load);
        const double loadMs = std::chrono::duration<double, std::milli>(Clock::now() - loadStart).count();
        std::cout << "Loaded " << g.airportCount() << " airports, " << g.routeCount()
                  << " distinct routes in " << std::fixed << std::setprecision(1) << loadMs << " ms\n\n";

        // Only airports that have at least one flight are interesting endpoints.
        std::vector<int> active;
        for (int id = 0; id < static_cast<int>(g.airportCount()); ++id) {
            if (!g.neighbors(id).empty()) active.push_back(id);
        }
        std::mt19937 rng(20261009);
        std::vector<std::pair<int, int>> pairs;
        for (int i = 0; i < queries; ++i) {
            pairs.emplace_back(active[rng() % active.size()], active[rng() % active.size()]);
        }

        std::cout << "  " << std::left << std::setw(36) << "algorithm" << std::right << std::setw(10)
                  << "us/query" << std::setw(12) << "settled" << std::setw(13) << "relaxed"
                  << std::setw(9) << "found" << "\n";

        measure("BFS (fewest flights)", pairs, [&](int s, int d, SearchStats& st) {
            SearchOptions o;
            o.stats = &st;
            return fewestFlights(g, s, d, o).has_value();
        });
        measure("Dijkstra (distance)", pairs, [&](int s, int d, SearchStats& st) {
            SearchOptions o;
            o.stats = &st;
            return dijkstra<DistancePolicy>(g, s, d, o).has_value();
        });
        measure("Dijkstra (distance-then-flights)", pairs, [&](int s, int d, SearchStats& st) {
            SearchOptions o;
            o.stats = &st;
            return dijkstra<DistanceThenFlightsPolicy>(g, s, d, o).has_value();
        });
        for (int k : {0, 1, 2, 3}) {
            measure("state-space Dijkstra, max " + std::to_string(k) + " stops", pairs,
                    [&](int s, int d, SearchStats& st) {
                        SearchOptions o;
                        o.stats = &st;
                        return cheapestWithMaxStops<DistancePolicy>(g, s, d, k, o).has_value();
                    });
        }
        // Same pairs for Bellman-Ford, so the rows are directly comparable.
        // "no limit" = n - 2 stops; it still stops early once a round changes
        // nothing, which on this network happens after about a dozen rounds.
        const int noLimit = static_cast<int>(g.airportCount()) - 2;
        for (int k : {1, 2, 3, noLimit}) {
            const std::string name = k == noLimit ? std::string("Bellman-Ford, no limit")
                                                  : "Bellman-Ford, max " + std::to_string(k) + " stops";
            measure(name, pairs, [&](int s, int d, SearchStats& st) {
                SearchOptions o;
                o.stats = &st;
                return bellmanFordMaxStops<DistancePolicy>(g, s, d, k, o).has_value();
            });
        }

        // Hub analysis for Bhopal, if it is in the data.
        if (std::optional<int> bho = g.findId("BHO")) {
            const auto start = Clock::now();
            const std::vector<HubReliance> hubs = rankHubs(g, *bho, 3, 5);
            const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            std::cout << "\n  rankHubs(BHO, 3 flights, top 5): " << std::setprecision(2) << ms << " ms ("
                      << hubs.size() << " hubs)\n";
        }
        return 0;
    } catch (const PlannerError& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
