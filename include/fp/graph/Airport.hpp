// Airport.hpp
//
// One node of the flight graph: what we know about a single airport.
// Created by the loader (src/io/DataLoader.cpp) from one line of
// data/airports.dat and stored inside FlightGraph
// (include/fp/graph/FlightGraph.hpp).

#pragma once

#include <string>

namespace fp {

struct Airport {
    std::string code;     // 3-letter IATA code, e.g. "BHO". Unique.
    std::string name;     // "Raja Bhoj International Airport"
    std::string city;     // "Bhopal"
    std::string country;  // "India"
    double latitude = 0.0;   // degrees, north positive
    double longitude = 0.0;  // degrees, east positive
};

}  // namespace fp
