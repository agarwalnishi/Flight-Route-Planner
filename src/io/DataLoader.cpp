// DataLoader.cpp
//
// Implementation of include/fp/io/DataLoader.hpp. The header explains the
// throw-vs-skip policy; this file just applies it line by line.
//
// OpenFlights column layout (https://openflights.org/data.html):
//
//   airports.dat: 0 id, 1 name, 2 city, 3 country, 4 IATA, 5 ICAO,
//                 6 latitude, 7 longitude, 8 altitude, ... (14 columns today)
//   routes.dat:   0 airline, 1 airline id, 2 source IATA, 3 source id,
//                 4 dest IATA, 5 dest id, 6 codeshare, 7 stops, 8 equipment
//
// Missing values are written as \N in both files.

#include "fp/io/DataLoader.hpp"

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <vector>

#include "fp/Errors.hpp"         // DataError
#include "fp/geo/Haversine.hpp"  // greatCircleMeters
#include "fp/io/Csv.hpp"         // parseCsvLine

namespace fp {
namespace {

constexpr const char* kMissing = "\\N";  // OpenFlights marker for "no value"

// Columns we read. Anything after these is ignored, so older or newer
// versions of the files with extra columns still load.
constexpr std::size_t kAirportMinColumns = 8;
constexpr std::size_t kRouteMinColumns = 5;

std::string where(const std::string& path, std::size_t lineNo) {
    return path + ":" + std::to_string(lineNo);
}

// Parse a decimal number, insisting the WHOLE field is a number.
// std::stod would happily read "12abc" as 12.
double parseDouble(const std::string& text, const std::string& context) {
    if (text.empty() || text == kMissing) {
        throw DataError(context + ": expected a number, got '" + text + "'");
    }
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (errno != 0 || end != text.c_str() + text.size()) {
        throw DataError(context + ": expected a number, got '" + text + "'");
    }
    return value;
}

// Read the next non-empty line, dropping a trailing '\r' so files saved on
// Windows (CRLF line endings) parse the same as Unix ones.
bool nextLine(std::ifstream& in, std::string& line, std::size_t& lineNo) {
    while (std::getline(in, line)) {
        ++lineNo;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) return true;
    }
    return false;
}

std::ifstream openOrThrow(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw DataError("cannot open '" + path + "'");
    }
    return in;
}

void loadAirports(const std::string& path, FlightGraph& graph, LoadStats& stats) {
    std::ifstream in = openOrThrow(path);
    std::string line;
    std::vector<std::string> f;  // reused for every line to avoid reallocating
    std::size_t lineNo = 0;

    while (nextLine(in, line, lineNo)) {
        ++stats.airportLines;
        const std::string ctx = where(path, lineNo);
        if (!parseCsvLine(line, f)) {
            throw DataError(ctx + ": malformed CSV (check the quotes)");
        }
        if (f.size() < kAirportMinColumns) {
            throw DataError(ctx + ": expected at least " + std::to_string(kAirportMinColumns) +
                            " columns, found " + std::to_string(f.size()));
        }

        const std::string& iata = f[4];
        if (iata.empty() || iata == kMissing) {
            ++stats.airportsWithoutIata;  // routes.dat refers to airports by IATA only
            continue;
        }

        Airport a;
        a.code = iata;
        a.name = f[1];
        a.city = f[2];
        a.country = f[3];
        a.latitude = parseDouble(f[6], ctx + " latitude");
        a.longitude = parseDouble(f[7], ctx + " longitude");
        if (a.latitude < -90.0 || a.latitude > 90.0 || a.longitude < -180.0 || a.longitude > 180.0) {
            throw DataError(ctx + ": coordinates out of range");
        }

        try {
            graph.addAirport(std::move(a));  // FlightGraph::addAirport
        } catch (const InvalidQuery& e) {
            // A duplicate code is a data problem, so report it as one.
            throw DataError(ctx + ": " + e.what());
        }
        ++stats.airportsLoaded;
    }
}

void loadRoutes(const std::string& path, FlightGraph& graph, LoadStats& stats) {
    std::ifstream in = openOrThrow(path);
    std::string line;
    std::vector<std::string> f;
    std::size_t lineNo = 0;

    while (nextLine(in, line, lineNo)) {
        ++stats.routeLines;
        const std::string ctx = where(path, lineNo);
        if (!parseCsvLine(line, f)) {
            throw DataError(ctx + ": malformed CSV (check the quotes)");
        }
        if (f.size() < kRouteMinColumns) {
            throw DataError(ctx + ": expected at least " + std::to_string(kRouteMinColumns) +
                            " columns, found " + std::to_string(f.size()));
        }

        // FlightGraph::findId never throws, which is what we want here:
        // an unknown airport in routes.dat is a known gap, not a crash.
        const std::optional<int> from = graph.findId(f[2]);
        const std::optional<int> to = graph.findId(f[4]);
        if (!from || !to) {
            ++stats.routesUnknownAirport;
            continue;
        }
        if (*from == *to) {
            ++stats.routesSelfLoop;
            continue;
        }

        const Airport& a = graph.airport(*from);
        const Airport& b = graph.airport(*to);
        graph.addRoute(*from, *to, greatCircleMeters(a.latitude, a.longitude, b.latitude, b.longitude));
        ++stats.routesUsed;
    }
}

}  // namespace

FlightGraph loadOpenFlights(const std::string& airportsPath, const std::string& routesPath,
                            LoadStats* statsOut) {
    FlightGraph graph;
    LoadStats stats;
    loadAirports(airportsPath, graph, stats);  // nodes first: routes refer to them
    loadRoutes(routesPath, graph, stats);
    graph.finalize();                          // merge e.g. 3 airlines flying BHO->DEL into 1 edge
    stats.distinctEdges = graph.routeCount();
    if (statsOut) *statsOut = stats;
    return graph;
}

FlightGraph loadOpenFlightsDir(const std::string& dataDir, LoadStats* stats) {
    std::string dir = dataDir;
    if (!dir.empty() && dir.back() != '/') dir += '/';
    return loadOpenFlights(dir + "airports.dat", dir + "routes.dat", stats);
}

}  // namespace fp
