// Haversine.hpp
//
// Great-circle distance between two points on Earth, i.e. the length of the
// shortest path along the surface of a sphere. This is the "cost" of a
// flight in this project: the dataset has no fares, but it does have
// accurate airport coordinates, so distance is a REAL cost, not an invented
// one.
//
// Used by the loader (src/io/DataLoader.cpp) to put a distance on every edge.
//
// Formula (haversine, from R. W. Sinnott, "Virtues of the Haversine", 1984):
//     a = sin^2(dlat/2) + cos(lat1) * cos(lat2) * sin^2(dlon/2)
//     d = 2 * R * asin(sqrt(a))
// It is numerically stable for short distances, unlike the plain
// spherical law of cosines.
//
// Earth is not a perfect sphere, so this can be off by up to about 0.5%
// compared to the exact ellipsoid distance. That is fine for comparing
// routes against each other.

#pragma once

#include <algorithm>
#include <cmath>

#include "fp/graph/Edge.hpp"  // Meters

namespace fp {

// IUGG mean Earth radius in metres.
constexpr double kEarthRadiusMeters = 6371008.8;

inline Meters greatCircleMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double lat1 = lat1Deg * kDegToRad;
    const double lat2 = lat2Deg * kDegToRad;
    const double dLat = (lat2Deg - lat1Deg) * kDegToRad;
    const double dLon = (lon2Deg - lon1Deg) * kDegToRad;

    const double sinHalfLat = std::sin(dLat / 2.0);
    const double sinHalfLon = std::sin(dLon / 2.0);
    double a = sinHalfLat * sinHalfLat + std::cos(lat1) * std::cos(lat2) * sinHalfLon * sinHalfLon;
    a = std::min(1.0, std::max(0.0, a));  // guard against rounding just outside [0, 1]

    const double d = 2.0 * kEarthRadiusMeters * std::asin(std::sqrt(a));
    return static_cast<Meters>(std::llround(d));
}

}  // namespace fp
