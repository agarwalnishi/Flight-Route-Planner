// TestHarness.hpp
//
// A deliberately tiny unit test framework (about 100 lines) so the project
// has zero external dependencies. It does the three things we need:
//
//   TEST_CASE(name) { ... }      registers a test function automatically
//   CHECK(expr), CHECK_EQ(a, b)  record a failure and keep going
//   REQUIRE(expr)                record a failure and stop this test
//   CHECK_THROWS_AS(expr, Type)  expr must throw exactly that exception type
//
// The runner lives in tests/test_main.cpp.
//
// How registration works: TEST_CASE expands to a function declaration plus a
// static Registrar object. Static objects are constructed before main()
// runs, and each Registrar's constructor appends its test to the registry.
// So adding a test is just writing it; no list to keep in sync.

#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace th {

struct TestCase {
    const char* name;
    void (*fn)();
};

// Function-local static: initialised on first use, which avoids the
// "static initialisation order" problem across test files.
inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

// Counters for the summary line. Inline variables (C++17) so every test file
// shares one copy.
inline int checksRun = 0;
inline int checksFailed = 0;

// Thrown by REQUIRE to abort the current test only.
struct RequireFailed {};

inline void reportFailure(const char* file, int line, const std::string& message) {
    ++checksFailed;
    std::cerr << "  FAILED " << file << ":" << line << "  " << message << "\n";
}

}  // namespace th

#define TH_CONCAT_INNER(a, b) a##b
#define TH_CONCAT(a, b) TH_CONCAT_INNER(a, b)

#define TEST_CASE(name)                                                   \
    static void name();                                                   \
    static ::th::Registrar TH_CONCAT(name, _registrar)(#name, &name);     \
    static void name()

#define CHECK(expr)                                                       \
    do {                                                                  \
        ++::th::checksRun;                                                \
        if (!(expr)) ::th::reportFailure(__FILE__, __LINE__, #expr);      \
    } while (0)

#define REQUIRE(expr)                                                     \
    do {                                                                  \
        ++::th::checksRun;                                                \
        if (!(expr)) {                                                    \
            ::th::reportFailure(__FILE__, __LINE__, "REQUIRE " #expr);    \
            throw ::th::RequireFailed{};                                  \
        }                                                                 \
    } while (0)

// Prints both values on failure, so you see WHAT was wrong, not just where.
// The values are COPIED, not bound by reference: `CHECK_EQ(*maybe(), 5)`
// would otherwise keep a reference into a temporary std::optional that is
// already destroyed (a dangling reference this harness once had).
#define CHECK_EQ(a, b)                                                    \
    do {                                                                  \
        ++::th::checksRun;                                                \
        const auto th_a_ = (a);                                           \
        const auto th_b_ = (b);                                           \
        if (!(th_a_ == th_b_)) {                                          \
            std::ostringstream th_os_;                                    \
            th_os_ << #a " == " #b "  (" << th_a_ << " vs " << th_b_ << ")"; \
            ::th::reportFailure(__FILE__, __LINE__, th_os_.str());        \
        }                                                                 \
    } while (0)

#define CHECK_THROWS_AS(expr, Type)                                       \
    do {                                                                  \
        ++::th::checksRun;                                                \
        bool th_ok_ = false;                                              \
        try {                                                             \
            (void)(expr);                                                 \
        } catch (const Type&) {                                           \
            th_ok_ = true;                                                \
        } catch (...) {                                                   \
        }                                                                 \
        if (!th_ok_) ::th::reportFailure(__FILE__, __LINE__, #expr " did not throw " #Type); \
    } while (0)
