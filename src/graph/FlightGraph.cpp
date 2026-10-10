// FlightGraph.cpp
//
// Implementation of the class declared in include/fp/graph/FlightGraph.hpp.
// Read the header first: it explains the "name layer" vs "number layer"
// split that everything here follows.

#include "fp/graph/FlightGraph.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

#include "fp/Errors.hpp"  // InvalidQuery, UnknownAirport

namespace fp {

std::string normalizeCode(const std::string& code) {
    std::string out = code;
    for (char& c : out) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return out;
}

int FlightGraph::addAirport(Airport airport) {
    airport.code = normalizeCode(airport.code);
    if (airport.code.empty()) {
        throw InvalidQuery("airport code must not be empty");
    }
    const int id = static_cast<int>(airports_.size());
    // AVLMap::insert returns false when the key exists, which is how we
    // reject a second airport claiming the same code.
    if (!idByCode_.insert(airport.code, id)) {
        throw InvalidQuery("duplicate airport code '" + airport.code + "'");
    }
    airports_.push_back(std::move(airport));
    adjacency_.emplace_back();
    return id;
}

void FlightGraph::addRoute(int from, int to, Meters meters) {
    requireValidId(from, "route origin");
    requireValidId(to, "route destination");
    if (meters < 0) {
        throw InvalidQuery("route length must be non-negative");
    }
    Edge edge;
    edge.to = to;
    edge.meters = meters;
    edge.airlines = 1;
    adjacency_[static_cast<std::size_t>(from)].push_back(edge);
    finalized_ = false;
}

void FlightGraph::finalize() {
    if (finalized_) return;
    for (std::vector<Edge>& edges : adjacency_) {
        // Sort by destination so duplicates sit next to each other, and so
        // edgeBetween() can binary search later.
        std::sort(edges.begin(), edges.end(),
                  [](const Edge& a, const Edge& b) { return a.to < b.to; });

        // Merge runs with the same destination (classic two-pointer dedupe).
        std::size_t write = 0;
        for (std::size_t read = 0; read < edges.size(); ++read) {
            if (write > 0 && edges[write - 1].to == edges[read].to) {
                Edge& kept = edges[write - 1];
                kept.meters = std::min(kept.meters, edges[read].meters);
                kept.airlines += edges[read].airlines;
            } else {
                edges[write++] = edges[read];
            }
        }
        edges.resize(write);
        edges.shrink_to_fit();
    }
    finalized_ = true;
}

int FlightGraph::idOf(const std::string& code) const {
    std::optional<int> id = findId(code);
    if (!id) {
        throw UnknownAirport(normalizeCode(code));
    }
    return *id;
}

std::optional<int> FlightGraph::findId(const std::string& code) const {
    const int* id = idByCode_.find(normalizeCode(code));
    if (!id) return std::nullopt;
    return *id;
}

std::vector<int> FlightGraph::idsWithPrefix(const std::string& prefix, std::size_t limit) const {
    const std::string wanted = normalizeCode(prefix);
    std::vector<int> result;
    // Start at the first code >= prefix. Codes are visited in sorted order,
    // so the first code that does not start with the prefix means we are done.
    idByCode_.forEachFrom(wanted, [&](const std::string& code, int id) {
        if (code.compare(0, wanted.size(), wanted) != 0) return false;
        if (result.size() >= limit) return false;
        result.push_back(id);
        return true;
    });
    return result;
}

const Airport& FlightGraph::airport(int id) const {
    requireValidId(id, "airport");
    return airports_[static_cast<std::size_t>(id)];
}

const std::vector<Edge>& FlightGraph::neighbors(int id) const {
    requireValidId(id, "airport");
    return adjacency_[static_cast<std::size_t>(id)];
}

const Edge* FlightGraph::edgeBetween(int from, int to) const {
    const std::vector<Edge>& edges = neighbors(from);
    if (finalized_) {
        // Sorted by destination after finalize(): binary search.
        auto it = std::lower_bound(edges.begin(), edges.end(), to,
                                   [](const Edge& e, int target) { return e.to < target; });
        if (it != edges.end() && it->to == to) return &*it;
        return nullptr;
    }
    // Not finalized yet: fall back to a linear scan for the shortest match.
    const Edge* best = nullptr;
    for (const Edge& e : edges) {
        if (e.to == to && (!best || e.meters < best->meters)) best = &e;
    }
    return best;
}

std::size_t FlightGraph::routeCount() const {
    std::size_t total = 0;
    for (const std::vector<Edge>& edges : adjacency_) total += edges.size();
    return total;
}

void FlightGraph::requireValidId(int id, const char* what) const {
    if (id < 0 || static_cast<std::size_t>(id) >= airports_.size()) {
        throw InvalidQuery(std::string(what) + " id " + std::to_string(id) + " is out of range");
    }
}

}  // namespace fp
