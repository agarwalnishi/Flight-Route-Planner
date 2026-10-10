// GraphBuilders.hpp
//
// Helpers shared by the algorithm tests (test_algo.cpp, test_analysis.cpp,
// test_random_oracle.cpp) for building small graphs by hand or at random,
// without going through the CSV loader.

#pragma once

#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "fp/graph/FlightGraph.hpp"

namespace testutil {

// Airport i gets the code "A", "B", "C", ... (then "A1", "B1", ... beyond 26).
inline std::string codeFor(int i) {
    std::string code(1, static_cast<char>('A' + i % 26));
    if (i >= 26) code += std::to_string(i / 26);
    return code;
}

// edges: (from, to, meters). The graph is finalized before returning.
inline fp::FlightGraph makeGraph(int n, const std::vector<std::tuple<int, int, fp::Meters>>& edges) {
    fp::FlightGraph g;
    for (int i = 0; i < n; ++i) {
        fp::Airport a;
        a.code = codeFor(i);
        a.city = "City" + std::to_string(i);
        a.country = "Testland";
        g.addAirport(a);
    }
    for (const auto& [from, to, meters] : edges) g.addRoute(from, to, meters);
    g.finalize();
    return g;
}

// Random directed graph with n airports, each possible edge present with
// probability `density`, lengths in [0, maxMeters]. Zero lengths are allowed
// on purpose: they are the classic place where tie handling goes wrong.
inline fp::FlightGraph randomGraph(std::mt19937& rng, int n, double density, int maxMeters) {
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    std::uniform_int_distribution<int> length(0, maxMeters);
    std::vector<std::tuple<int, int, fp::Meters>> edges;
    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            if (u != v && coin(rng) < density) edges.emplace_back(u, v, length(rng));
        }
    }
    return makeGraph(n, edges);
}

}  // namespace testutil
