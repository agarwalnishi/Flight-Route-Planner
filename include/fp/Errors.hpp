// Errors.hpp
//
// Every exception this project throws lives here, so there is exactly one
// place to look when you want to know "what can go wrong?".
//
// The rule used throughout the code base:
//   * Throw when the CALLER did something wrong (asked for an airport that
//     does not exist, passed a negative stop limit, popped an empty heap) or
//     when the INPUT DATA is broken (malformed CSV line, file missing).
//   * Do NOT throw for normal outcomes. "There is no route from A to B" is a
//     perfectly valid answer, so the algorithms return std::optional instead.
//
// All exceptions derive from PlannerError, so main() can catch one type and
// print a clean message (see src/main.cpp).

#pragma once

#include <stdexcept>
#include <string>

namespace fp {

// Base class for everything this project throws.
class PlannerError : public std::runtime_error {
public:
    explicit PlannerError(const std::string& message) : std::runtime_error(message) {}
};

// A user typed an airport code we do not know about, e.g. "XYZ".
// Thrown by FlightGraph::idOf (src/graph/FlightGraph.cpp).
class UnknownAirport : public PlannerError {
public:
    explicit UnknownAirport(const std::string& code)
        : PlannerError("unknown airport code '" + code + "'"), code_(code) {}

    const std::string& code() const { return code_; }

private:
    std::string code_;
};

// The input files are missing or structurally broken (wrong number of
// columns, a latitude that is not a number, an unterminated quote).
// Thrown by the loader (src/io/DataLoader.cpp).
class DataError : public PlannerError {
public:
    explicit DataError(const std::string& message) : PlannerError("data error: " + message) {}
};

// The question itself does not make sense: negative stop limit, airport id
// out of range, asking which airports are lost when the hub IS the origin.
class InvalidQuery : public PlannerError {
public:
    explicit InvalidQuery(const std::string& message) : PlannerError("invalid query: " + message) {}
};

// Someone called pop()/front()/top() on an empty Queue or MinHeap.
// Thrown by include/fp/ds/Queue.hpp and include/fp/ds/MinHeap.hpp.
class EmptyContainer : public PlannerError {
public:
    explicit EmptyContainer(const std::string& what) : PlannerError(what + " on empty container") {}
};

// AVLMap::at() was asked for a key that is not stored.
// Thrown by include/fp/ds/AVLMap.hpp.
class KeyNotFound : public PlannerError {
public:
    explicit KeyNotFound(const std::string& message) : PlannerError(message) {}
};

}  // namespace fp
