// main.cpp
//
// The command line tool. This file only does three things:
//   1. parse arguments,
//   2. load the dataset (src/io/DataLoader.cpp),
//   3. call the right algorithm and print the answer.
// All the real logic lives in the library (include/fp/...), so it can be
// tested without going through the command line.
//
// Run `flight-planner help` for usage.

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "fp/Errors.hpp"
#include "fp/algo/BFS.hpp"             // fewestFlights
#include "fp/algo/BellmanFord.hpp"     // bellmanFordMaxStops (for --verify)
#include "fp/algo/CostPolicies.hpp"    // DistancePolicy, FlightsPolicy, DistanceThenFlightsPolicy
#include "fp/algo/Dijkstra.hpp"        // dijkstra
#include "fp/algo/KStopSearch.hpp"     // cheapestWithMaxStops
#include "fp/analysis/HubAnalysis.hpp" // reachability, hubFailureImpact, rankHubs
#include "fp/io/DataLoader.hpp"        // loadOpenFlightsDir, LoadStats

#ifndef FP_DEFAULT_DATA_DIR
#define FP_DEFAULT_DATA_DIR "data"  // normally set by CMakeLists.txt
#endif

namespace {

using namespace fp;

const char* kUsage = R"(flight-planner: routes and connectivity on the OpenFlights network

USAGE
  flight-planner <command> [arguments] [options]

COMMANDS
  route <FROM> <TO>        Best route between two airports
      --by distance              shortest total distance (default, Dijkstra)
      --by flights               fewest flights (BFS)
      --by distance-then-flights shortest, ties broken by fewer flights
      --max-stops K              at most K stops (state-space Dijkstra)
      --verify                   cross-check the cost with Bellman-Ford

  reach <FROM>             What can be reached in 1, 2, 3... flights
      --max-hops H               ripple limit (default 3, -1 = no limit)
      --list                     print the airport codes in each ripple

  hub-impact <FROM> <HUB>  What FROM loses if HUB shuts down
      --max-hops H               (default 3)

  hubs <FROM>              Airports FROM depends on most
      --max-hops H               (default 3)
      --top N                    how many to show (default 5)

  find <PREFIX>            Airport codes starting with PREFIX
      --limit N                  (default 20)

  info <CODE>              Details and direct destinations of one airport
  stats                    What was loaded and what was skipped
  help                     This text

GLOBAL OPTIONS
  --data DIR               folder with airports.dat and routes.dat

EXAMPLES
  flight-planner route BHO JFK
  flight-planner route BHO JFK --max-stops 1 --verify
  flight-planner hub-impact BHO DEL
  flight-planner hubs BHO --top 5
)";

// Bad command line (as opposed to a bad query). Exit code 2.
struct UsageError {
    std::string message;
};

// ---------------------------------------------------------------- arguments

struct Args {
    std::string command;
    std::vector<std::string> positional;
    std::vector<std::pair<std::string, std::string>> options;  // --name value
    std::vector<std::string> switches;                         // --name

    std::optional<std::string> option(const std::string& name) const {
        for (const auto& kv : options) {
            if (kv.first == name) return kv.second;
        }
        return std::nullopt;
    }
    bool has(const std::string& name) const {
        for (const auto& s : switches) {
            if (s == name) return true;
        }
        return false;
    }
};

bool takesValue(const std::string& name) {
    return name == "--data" || name == "--by" || name == "--max-stops" || name == "--max-hops" ||
           name == "--top" || name == "--limit";
}

bool isSwitch(const std::string& name) {
    return name == "--verify" || name == "--list" || name == "--help" || name == "-h";
}

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("-", 0) == 0 && a.size() > 1) {
            if (takesValue(a)) {
                if (i + 1 >= argc) throw UsageError{a + " needs a value"};
                args.options.emplace_back(a, argv[++i]);
            } else if (isSwitch(a)) {
                args.switches.push_back(a);
            } else {
                throw UsageError{"unknown option " + a};
            }
        } else if (args.command.empty()) {
            args.command = a;
        } else {
            args.positional.push_back(a);
        }
    }
    return args;
}

int parseInt(const std::string& text, const std::string& what) {
    try {
        std::size_t used = 0;
        const int value = std::stoi(text, &used);
        if (used != text.size()) throw std::invalid_argument("trailing characters");
        return value;
    } catch (const std::exception&) {
        throw UsageError{what + " must be a whole number, got '" + text + "'"};
    }
}

int intOption(const Args& args, const std::string& name, int fallback) {
    std::optional<std::string> v = args.option(name);
    return v ? parseInt(*v, name) : fallback;
}

void requirePositional(const Args& args, std::size_t count, const std::string& shape) {
    if (args.positional.size() != count) {
        throw UsageError{"usage: flight-planner " + args.command + " " + shape};
    }
}

// --------------------------------------------------------------- formatting

// 11742000 m -> "11,742 km"
std::string formatKm(Meters meters) {
    std::string digits = std::to_string((meters + 500) / 1000);
    std::string out;
    int count = 0;
    for (auto it = digits.rbegin(); it != digits.rend(); ++it) {
        if (count > 0 && count % 3 == 0) out.insert(out.begin(), ',');
        out.insert(out.begin(), *it);
        ++count;
    }
    return out + " km";
}

std::string label(const FlightGraph& g, int id) {
    const Airport& a = g.airport(id);
    return a.code + " (" + a.city + ", " + a.country + ")";
}

std::string codes(const FlightGraph& g, const std::vector<int>& ids, std::size_t limit) {
    std::ostringstream os;
    for (std::size_t i = 0; i < ids.size() && i < limit; ++i) {
        if (i > 0) os << ", ";
        os << g.airport(ids[i]).code;
    }
    if (ids.size() > limit) os << ", ... (+" << (ids.size() - limit) << " more)";
    return os.str();
}

std::string plural(std::size_t n, const char* one, const char* many) {
    return std::to_string(n) + " " + (n == 1 ? one : many);
}

double millisSince(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void printItinerary(const FlightGraph& g, const Itinerary& it) {
    // Header line: BHO -> DEL -> JFK
    for (std::size_t i = 0; i < it.path.size(); ++i) {
        if (i > 0) std::cout << " -> ";
        std::cout << g.airport(it.path[i]).code;
    }
    std::cout << "\n\n";
    // One line per flight, using the real edge length (FlightGraph::edgeBetween).
    for (std::size_t i = 1; i < it.path.size(); ++i) {
        const Edge* e = g.edgeBetween(it.path[i - 1], it.path[i]);
        std::cout << "  " << std::setw(2) << i << ". " << std::left << std::setw(34)
                  << label(g, it.path[i - 1]) << " -> " << std::setw(34) << label(g, it.path[i])
                  << std::right << std::setw(10) << formatKm(e->meters) << "\n";
    }
    std::cout << "\n  Total: " << formatKm(it.totalMeters) << ", "
              << plural(static_cast<std::size_t>(it.flights()), "flight", "flights") << ", "
              << plural(static_cast<std::size_t>(it.stops()), "stop", "stops") << "\n";
}

// ------------------------------------------------------------------ commands

// Shared by every route variant. Policy decides what "best" means
// (include/fp/algo/CostPolicies.hpp); maxStops = -1 means "no limit".
template <typename Policy>
int runRoute(const FlightGraph& g, int from, int to, int maxStops, bool verify) {
    const auto start = std::chrono::steady_clock::now();
    std::optional<Itinerary> it;
    std::string algorithm;
    if (maxStops < 0) {
        it = dijkstra<Policy>(g, from, to);  // Dijkstra.hpp
        algorithm = std::string("Dijkstra (") + Policy::name + ")";
    } else {
        it = cheapestWithMaxStops<Policy>(g, from, to, maxStops);  // KStopSearch.hpp
        algorithm = std::string("state-space Dijkstra (") + Policy::name + ", at most " +
                    plural(static_cast<std::size_t>(maxStops), "stop", "stops") + ")";
    }
    const double ms = millisSince(start);

    if (!it) {
        std::cout << "No route from " << label(g, from) << " to " << label(g, to);
        if (maxStops >= 0) std::cout << " with at most " << plural(static_cast<std::size_t>(maxStops), "stop", "stops");
        std::cout << ".\n";
    } else {
        printItinerary(g, *it);
    }
    std::cout << "  Algorithm: " << algorithm << ", " << std::fixed << std::setprecision(2) << ms
              << " ms\n";

    if (verify) {
        // Independent answer from Bellman-Ford (BellmanFord.hpp). A stop
        // limit of n - 2 means "effectively unlimited".
        const int limit = maxStops >= 0 ? maxStops : static_cast<int>(g.airportCount()) - 2;
        const auto vStart = std::chrono::steady_clock::now();
        const auto oracle = bellmanFordMaxStops<Policy>(g, from, to, std::max(0, limit));
        const double vMs = millisSince(vStart);
        bool agree = false;
        if (!it && !oracle) {
            agree = true;
        } else if (it && oracle) {
            agree = pathCost<Policy>(g, it->path) == *oracle;  // Itinerary.hpp
        }
        std::cout << "  Verify:    Bellman-Ford " << (agree ? "agrees" : "DISAGREES") << " ("
                  << std::fixed << std::setprecision(2) << vMs << " ms)\n";
        if (!agree) return 1;
    }
    return 0;
}

int cmdRoute(const FlightGraph& g, const Args& args) {
    requirePositional(args, 2, "<FROM> <TO> [--by ...] [--max-stops K] [--verify]");
    const int from = g.idOf(args.positional[0]);  // throws UnknownAirport
    const int to = g.idOf(args.positional[1]);
    const std::string by = args.option("--by").value_or("distance");
    const int maxStops = intOption(args, "--max-stops", -1);
    if (args.option("--max-stops") && maxStops < 0) {
        throw UsageError{"--max-stops must be 0 or more"};
    }
    const bool verify = args.has("--verify");

    if (by == "distance") return runRoute<DistancePolicy>(g, from, to, maxStops, verify);
    if (by == "distance-then-flights") {
        return runRoute<DistanceThenFlightsPolicy>(g, from, to, maxStops, verify);
    }
    if (by == "flights") {
        // BFS is the right tool here (see BFS.hpp). A stop limit is just a
        // check on its answer, because BFS already found the fewest flights.
        const auto start = std::chrono::steady_clock::now();
        std::optional<Itinerary> it = fewestFlights(g, from, to);
        const double ms = millisSince(start);
        if (it && maxStops >= 0 && it->stops() > maxStops) it.reset();
        if (!it) {
            std::cout << "No route from " << label(g, from) << " to " << label(g, to) << ".\n";
        } else {
            printItinerary(g, *it);
        }
        std::cout << "  Algorithm: BFS (fewest flights), " << std::fixed << std::setprecision(2) << ms
                  << " ms\n";
        if (verify) {
            const int limit = maxStops >= 0 ? maxStops : static_cast<int>(g.airportCount()) - 2;
            const auto oracle = bellmanFordMaxStops<FlightsPolicy>(g, from, to, std::max(0, limit));
            const bool agree = (!it && !oracle) || (it && oracle && it->flights() == *oracle);
            std::cout << "  Verify:    Bellman-Ford " << (agree ? "agrees" : "DISAGREES") << "\n";
            if (!agree) return 1;
        }
        return 0;
    }
    throw UsageError{"--by must be distance, flights or distance-then-flights"};
}

int cmdReach(const FlightGraph& g, const Args& args) {
    requirePositional(args, 1, "<FROM> [--max-hops H] [--list]");
    const int from = g.idOf(args.positional[0]);
    const int maxHops = intOption(args, "--max-hops", 3);
    const ReachLayers r = reachability(g, from, maxHops);  // HubAnalysis.hpp

    std::cout << "From " << label(g, from) << ":\n\n";
    std::size_t cumulative = 0;
    for (std::size_t h = 1; h < r.layers.size(); ++h) {
        cumulative += r.layers[h].size();
        std::cout << "  " << plural(h, "flight ", "flights") << ": " << std::setw(5) << r.layers[h].size()
                  << " new airports  (" << cumulative << " total)\n";
        if (args.has("--list")) {
            std::cout << "             " << codes(g, r.layers[h], 40) << "\n";
        }
    }
    std::cout << "\n  " << r.destinations() << " of " << (g.airportCount() - 1)
              << " other airports reachable";
    if (maxHops != -1) std::cout << " within " << plural(static_cast<std::size_t>(maxHops), "flight", "flights");
    std::cout << ".\n";
    return 0;
}

int cmdHubImpact(const FlightGraph& g, const Args& args) {
    requirePositional(args, 2, "<FROM> <HUB> [--max-hops H]");
    const int from = g.idOf(args.positional[0]);
    const int hub = g.idOf(args.positional[1]);
    const int maxHops = intOption(args, "--max-hops", 3);
    const HubImpact impact = hubFailureImpact(g, from, hub, maxHops);  // HubAnalysis.hpp

    const std::string within =
        maxHops == -1 ? "" : " within " + plural(static_cast<std::size_t>(maxHops), "flight", "flights");
    std::cout << "If " << label(g, hub) << " shuts down, seen from " << label(g, from) << ":\n\n"
              << "  Reachable" << within << ": " << impact.reachableBefore << " -> "
              << impact.reachableAfter << "\n"
              << "  Lost:    " << impact.lost.size() << " airports\n"
              << "  Slower:  " << impact.slower.size() << " airports need more flights\n";
    if (!impact.lost.empty()) std::cout << "\n  Lost: " << codes(g, impact.lost, 30) << "\n";
    return 0;
}

int cmdHubs(const FlightGraph& g, const Args& args) {
    requirePositional(args, 1, "<FROM> [--max-hops H] [--top N]");
    const int from = g.idOf(args.positional[0]);
    const int maxHops = intOption(args, "--max-hops", 3);
    const int top = intOption(args, "--top", 5);
    if (top < 1) throw UsageError{"--top must be at least 1"};
    const auto start = std::chrono::steady_clock::now();
    const std::vector<HubReliance> hubs =
        rankHubs(g, from, maxHops, static_cast<std::size_t>(top));  // HubAnalysis.hpp
    const double ms = millisSince(start);

    std::cout << "Airports " << label(g, from) << " depends on ("
              << (maxHops == -1 ? std::string("no flight limit")
                                : "destinations within " +
                                      plural(static_cast<std::size_t>(maxHops), "flight", "flights"))
              << "):\n\n";
    std::cout << "  rank  hub                                 route share   lost if closed   slower\n";
    int rank = 1;
    for (const HubReliance& h : hubs) {
        std::cout << "  " << std::setw(4) << rank++ << "  " << std::left << std::setw(34)
                  << label(g, h.hub) << std::right << std::setw(10) << std::fixed
                  << std::setprecision(1) << (h.share * 100.0) << " %" << std::setw(17)
                  << h.lostIfClosed << std::setw(9) << h.slowerIfClosed << "\n";
    }
    std::cout << "\n  route share = fraction of destinations whose fewest-flight routes connect\n"
                 "  through that airport (Brandes dependency). lost/slower = BFS with it closed.\n"
              << "  " << std::fixed << std::setprecision(2) << ms << " ms\n";
    return 0;
}

int cmdFind(const FlightGraph& g, const Args& args) {
    requirePositional(args, 1, "<PREFIX> [--limit N]");
    const int limit = intOption(args, "--limit", 20);
    if (limit < 1) throw UsageError{"--limit must be at least 1"};
    // AVL prefix walk (FlightGraph::idsWithPrefix -> AVLMap::forEachFrom).
    const std::vector<int> ids = g.idsWithPrefix(args.positional[0], static_cast<std::size_t>(limit));
    if (ids.empty()) {
        std::cout << "No airport codes start with '" << normalizeCode(args.positional[0]) << "'.\n";
        return 0;
    }
    for (int id : ids) {
        std::cout << "  " << label(g, id) << "  " << g.airport(id).name << "\n";
    }
    return 0;
}

int cmdInfo(const FlightGraph& g, const Args& args) {
    requirePositional(args, 1, "<CODE>");
    const int id = g.idOf(args.positional[0]);
    const Airport& a = g.airport(id);
    std::vector<int> direct;
    for (const Edge& e : g.neighbors(id)) direct.push_back(e.to);
    std::cout << a.code << "  " << a.name << "\n"
              << "  " << a.city << ", " << a.country << "  (" << std::fixed << std::setprecision(4)
              << a.latitude << ", " << a.longitude << ")\n"
              << "  " << plural(direct.size(), "direct destination", "direct destinations")
              << (direct.empty() ? "" : ": " + codes(g, direct, 40)) << "\n";
    return 0;
}

int cmdStats(const FlightGraph& g, const LoadStats& s) {
    std::cout << "airports.dat: " << s.airportLines << " lines\n"
              << "  loaded:                 " << s.airportsLoaded << "\n"
              << "  skipped (no IATA code): " << s.airportsWithoutIata << "\n"
              << "routes.dat:   " << s.routeLines << " lines\n"
              << "  used:                   " << s.routesUsed << "\n"
              << "  skipped (unknown airport): " << s.routesUnknownAirport << "\n"
              << "  skipped (self loop):    " << s.routesSelfLoop << "\n"
              << "graph:        " << g.airportCount() << " airports, " << g.routeCount()
              << " distinct directed routes\n";
    return 0;
}

// Pick the data folder: --data, else ./data, else the source tree's data/.
std::string resolveDataDir(const Args& args) {
    if (std::optional<std::string> d = args.option("--data")) return *d;
    if (std::ifstream("data/airports.dat").good()) return "data";
    return FP_DEFAULT_DATA_DIR;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parseArgs(argc, argv);
        if (args.command.empty() || args.command == "help" || args.has("--help") || args.has("-h")) {
            std::cout << kUsage;
            return 0;
        }

        const auto loadStart = std::chrono::steady_clock::now();
        LoadStats stats;
        const FlightGraph graph = loadOpenFlightsDir(resolveDataDir(args), &stats);
        const double loadMs = millisSince(loadStart);

        const std::string& c = args.command;
        if (c == "route") return cmdRoute(graph, args);
        if (c == "reach") return cmdReach(graph, args);
        if (c == "hub-impact") return cmdHubImpact(graph, args);
        if (c == "hubs") return cmdHubs(graph, args);
        if (c == "find") return cmdFind(graph, args);
        if (c == "info") return cmdInfo(graph, args);
        if (c == "stats") {
            cmdStats(graph, stats);
            std::cout << "loaded in:    " << std::fixed << std::setprecision(1) << loadMs << " ms\n";
            return 0;
        }
        throw UsageError{"unknown command '" + c + "' (try: flight-planner help)"};
    } catch (const UsageError& e) {
        std::cerr << "usage error: " << e.message << "\n";
        return 2;
    } catch (const fp::PlannerError& e) {  // every project exception (Errors.hpp)
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
