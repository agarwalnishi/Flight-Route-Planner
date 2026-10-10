// test_main.cpp
//
// Runs every TEST_CASE registered by the other test files (see
// tests/TestHarness.hpp for how registration works).
//
//   ./fp_tests            run everything
//   ./fp_tests avl        run only tests whose name contains "avl"
//
// Exit code is 0 only if every check passed, so `ctest` and CI can rely on it.

#include <exception>
#include <iostream>
#include <string>

#include "TestHarness.hpp"

int main(int argc, char** argv) {
    const std::string filter = argc > 1 ? argv[1] : "";
    int ran = 0;
    int failedTests = 0;

    for (const th::TestCase& t : th::registry()) {
        if (!filter.empty() && std::string(t.name).find(filter) == std::string::npos) continue;
        ++ran;
        const int failuresBefore = th::checksFailed;
        std::cout << "[ RUN  ] " << t.name << "\n";
        try {
            t.fn();
        } catch (const th::RequireFailed&) {
            // already reported, just stop this test
        } catch (const std::exception& e) {
            th::reportFailure(t.name, 0, std::string("unexpected exception: ") + e.what());
        } catch (...) {
            th::reportFailure(t.name, 0, "unexpected non-standard exception");
        }
        const bool passed = th::checksFailed == failuresBefore;
        if (!passed) ++failedTests;
        std::cout << (passed ? "[  OK  ] " : "[ FAIL ] ") << t.name << "\n";
    }

    std::cout << "\n" << ran << " tests, " << th::checksRun << " checks, " << failedTests
              << " failed tests, " << th::checksFailed << " failed checks\n";
    return (th::checksFailed == 0 && ran > 0) ? 0 : 1;
}
