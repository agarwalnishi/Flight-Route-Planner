# Build file for the flight planner.
#
#   cmake -S . -B build                       # Release by default
#   cmake --build build -j
#   ./build/flight-planner route BHO JFK
#   ctest --test-dir build --output-on-failure
#
# Debug build with AddressSanitizer + UndefinedBehaviorSanitizer, which
# catches out-of-bounds reads, use-after-free and integer overflow in the
# hand-written data structures:
#   cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DFP_SANITIZE=ON

cmake_minimum_required(VERSION 3.16)
project(flight_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(FP_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

# Shared warning and sanitizer settings for every target below.
add_library(fp_options INTERFACE)
if(MSVC)
    target_compile_options(fp_options INTERFACE /W4)
else()
    target_compile_options(fp_options INTERFACE -Wall -Wextra -Wpedantic -Wshadow)
    if(FP_SANITIZE)
        target_compile_options(fp_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(fp_options INTERFACE -fsanitize=address,undefined)
    endif()
endif()

# The core library: everything except main(). The CLI, the tests and the
# benchmark all link against it, so they test exactly the same code.
add_library(fp_core
    src/graph/FlightGraph.cpp
    src/io/DataLoader.cpp
    src/algo/BFS.cpp
    src/analysis/HubAnalysis.cpp
)
target_include_directories(fp_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(fp_core PUBLIC fp_options)

# Lets the programs find data/ even when run from inside build/.
set(FP_DEFAULT_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")

# Command line tool.
add_executable(flight-planner src/main.cpp)
target_link_libraries(flight-planner PRIVATE fp_core)
target_compile_definitions(flight-planner PRIVATE FP_DEFAULT_DATA_DIR="${FP_DEFAULT_DATA_DIR}")

# Tests (our own tiny harness in tests/TestHarness.hpp, no downloads needed).
add_executable(fp_tests
    tests/test_main.cpp
    tests/test_ds.cpp
    tests/test_loader.cpp
    tests/test_algo.cpp
    tests/test_analysis.cpp
    tests/test_random_oracle.cpp
)
target_link_libraries(fp_tests PRIVATE fp_core)
target_compile_definitions(fp_tests PRIVATE FP_DEFAULT_DATA_DIR="${FP_DEFAULT_DATA_DIR}")

enable_testing()
add_test(NAME fp_tests COMMAND fp_tests)

# Benchmark on the real dataset.
add_executable(fp_bench bench/bench_queries.cpp)
target_link_libraries(fp_bench PRIVATE fp_core)
target_compile_definitions(fp_bench PRIVATE FP_DEFAULT_DATA_DIR="${FP_DEFAULT_DATA_DIR}")
