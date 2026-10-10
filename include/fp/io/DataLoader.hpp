// DataLoader.hpp
//
// Reads the OpenFlights dataset (data/airports.dat and data/routes.dat) and
// builds a finalized FlightGraph (include/fp/graph/FlightGraph.hpp).
//
// Error policy, which is a deliberate design choice:
//
//   THROW DataError when a file is structurally broken. Missing file, wrong
//   number of columns, a latitude that is not a number, a broken quote.
//   That means the file is not what we think it is, and guessing would
//   produce silently wrong routes.
//
//   SKIP AND COUNT when a row is valid but unusable. These are known,
//   documented gaps in the dataset:
//     * airports with no IATA code ("\N"), which no route can refer to
//     * routes whose airport is not in airports.dat
//     * routes from an airport to itself
//   Every skip is counted in LoadStats, so nothing disappears silently and
//   `flight-planner stats` can print exactly what was dropped and why.
//
// Implementation: src/io/DataLoader.cpp

#pragma once

#include <cstddef>
#include <string>

#include "fp/graph/FlightGraph.hpp"

namespace fp {

struct LoadStats {
    std::size_t airportLines = 0;            // lines read from airports.dat
    std::size_t airportsLoaded = 0;          // became graph nodes
    std::size_t airportsWithoutIata = 0;     // skipped: IATA code is \N

    std::size_t routeLines = 0;              // lines read from routes.dat
    std::size_t routesUnknownAirport = 0;    // skipped: endpoint not loaded
    std::size_t routesSelfLoop = 0;          // skipped: from == to
    std::size_t routesUsed = 0;              // airline routes added to the graph
    std::size_t distinctEdges = 0;           // after merging duplicate pairs
};

// Load both files and return a finalized graph. `stats` may be nullptr.
// Throws DataError (see include/fp/Errors.hpp).
FlightGraph loadOpenFlights(const std::string& airportsPath, const std::string& routesPath,
                            LoadStats* stats = nullptr);

// Convenience overload: looks for airports.dat and routes.dat inside `dataDir`.
FlightGraph loadOpenFlightsDir(const std::string& dataDir, LoadStats* stats = nullptr);

}  // namespace fp
