# Flight Route Planner

A C++17 command line tool that treats the world's airline network as a graph and answers two
kinds of questions:

1. **Routing.** What is the shortest route from A to B? The fewest flights? The shortest with at
   most K stops?
2. **Connectivity.** How connected is a smaller city, and which big airports is it quietly
   depending on? What does it lose if one of them shuts down?

Every data structure the algorithms run on (queue, heap, balanced tree) is written from scratch,
and every algorithm is checked against a brute-force answer on thousands of random graphs.

```
$ flight-planner hubs BHO
Airports BHO (Bhopal, India) depends on (destinations within 3 flights):

  rank  hub                                 route share   lost if closed   slower
     1  DEL (Delhi, India)                      64.3 %              299       25
     2  BOM (Mumbai, India)                     35.6 %               53       11
     3  ORD (Chicago, United States)             8.0 %               81        0
     4  IST (Istanbul, Turkey)                   4.6 %               33        0
     5  EWR (Newark, United States)              4.2 %                6        0
```

---

## Contents

- [Why this project](#why-this-project)
- [Quick start](#quick-start)
- [Commands](#commands)
- [How it works](#how-it-works)
- [Algorithms, and why each one](#algorithms-and-why-each-one)
- [Data structures, and why each one](#data-structures-and-why-each-one)
- [Design decisions](#design-decisions)
- [Complexity](#complexity)
- [Testing](#testing)
- [Benchmarks](#benchmarks)
- [Data and its limits](#data-and-its-limits)
- [Project layout](#project-layout)
- [Deliberately left out](#deliberately-left-out)

---

## Why this project

"Flight planner with Dijkstra" is a common project. This one is built around a concrete question
instead: **how well connected is a tier-2 city, and how fragile is that connection?**

Bhopal (BHO) is the example, because it shows the pattern clearly. In the dataset it has exactly
two direct destinations, Delhi and Mumbai:

| Within | Airports reachable from BHO | from DEL, for comparison |
|---|---:|---:|
| 1 flight  |     2 |   |
| 2 flights |   114 |   |
| 3 flights | 1,277 | 2,717 |
| any number | 3,209 |  |

And the hub analysis shows the dependency is lopsided:

- About 64% of BHO's fewest-flight routes (within 3 flights) connect through Delhi.
- If Delhi closes, 299 of the 1,276 destinations become unreachable within 3 flights, and
  another 25 need an extra flight.
- If Mumbai closes instead, only 53 are lost.

The same commands work for any airport in the dataset. The routes are a 2014 snapshot, so read
these as a demonstration of the method rather than current facts (see
[Data and its limits](#data-and-its-limits)).

---

## Quick start

Needs a C++17 compiler and CMake 3.16+ (tested with GCC 13 and Clang 18, warning-free under
`-Wall -Wextra -Wpedantic -Wshadow`). There are no other dependencies; the dataset ships in `data/`.

```sh
cmake -S . -B build                 # Release build by default
cmake --build build -j

./build/flight-planner route BHO JFK
./build/fp_tests                    # or: ctest --test-dir build --output-on-failure
./build/fp_bench                    # timings on the real network
```

Debug build with AddressSanitizer and UndefinedBehaviorSanitizer (worth running after touching
any of the hand-written data structures):

```sh
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DFP_SANITIZE=ON
cmake --build build-asan -j && ./build-asan/fp_tests
```

---

## Commands

| Command | What it answers | Algorithm |
|---|---|---|
| `route A B` | Shortest total distance | Dijkstra |
| `route A B --by flights` | Fewest flights | BFS |
| `route A B --by distance-then-flights` | Shortest, ties broken by fewer flights | Dijkstra, lexicographic cost |
| `route A B --max-stops K` | Shortest with at most K stops | Dijkstra over (airport, flights) states |
| `route ... --verify` | Re-checks the cost independently | Bellman-Ford |
| `reach A [--max-hops H] [--list]` | What A reaches in 1, 2, 3... flights | BFS |
| `hub-impact A HUB` | What A loses if HUB shuts down | BFS twice, second time with HUB blocked |
| `hubs A [--top N]` | Which airports A depends on most | Brandes dependency + hub-impact |
| `find PREFIX` | Airport codes starting with PREFIX | AVL tree prefix walk |
| `info CODE` | Airport details and direct destinations | |
| `stats` | What was loaded and what was skipped | |

Codes are case-insensitive. `--data DIR` points at a different data folder.
Exit codes: `0` success (including "no route found"), `1` bad query or bad data, `2` bad command line.

Example: a stop limit changing the answer.

```
$ flight-planner route BHO LHR --by flights
BHO -> BOM -> LHR                       7,871 km, 2 flights
$ flight-planner route BHO LHR
BHO -> DEL -> LHR                       7,319 km, 2 flights
$ flight-planner route BHO JFK --max-stops 0
No route from BHO (Bhopal, India) to JFK (New York, United States) with at most 0 stops.
```

(Outputs shortened. The fewest-flights search has two 2-flight options and takes the first one
it discovers; the distance search picks the shorter of them.)

---

## How it works

Draw every airport as a dot and every route as an arrow labelled with its length. That drawing is
a **directed, weighted graph**, and every question above is a question about walking along the
arrows.

The graph (`include/fp/graph/FlightGraph.hpp`) has two layers:

```
  what you type          name layer                 number layer
  ─────────────          ──────────                 ────────────
    "BHO"     ──────▶   AVLMap<string,int>  ──▶ 1432 ──▶ adjacency[1432] = [ {to: 412,  ...},
                        (used once per query)            {to: 2017, ...} ]
                                                         (used in every inner loop)
```

- **Name layer:** a balanced tree from airport code to an integer id. Used only to translate
  what the user typed, and for prefix search.
- **Number layer:** airports are `0 .. n-1`, and edges live in `vector<vector<Edge>>`.
  "Who can I fly to from airport 1432" is a direct array index. The algorithms only ever see
  this layer.

Loading (`src/io/DataLoader.cpp`) reads the CSV files, computes each route's great-circle
distance from the airport coordinates, and merges duplicate routes (three airlines flying
BHO to DEL become one edge).

---

## Algorithms, and why each one

### Fewest flights: BFS (`include/fp/algo/BFS.hpp`)

BFS spreads out in ripples: first every airport one flight away, then every new airport two
flights away, and so on. The ripple an airport first appears in is its fewest-flight distance,
because every closer ripple was finished first. That ordering is exactly what a FIFO queue gives.

Why not Dijkstra here? Every flight costs 1, so the queue is already in cost order. A heap would
add a `log n` factor for the same answer: BFS is `O(n + m)`, Dijkstra `O(m log n)`.

### Shortest distance: Dijkstra (`include/fp/algo/Dijkstra.hpp`)

Always continue from the closest airport not settled yet. When an airport is popped as the
closest unsettled one, its distance is final: any other way to reach it would have to pass
through something at least as far away, and distances never go negative.

| Alternative | Why not |
|---|---|
| BFS | Ignores distances. Only correct when every flight costs the same. |
| Bellman-Ford | `O(n · m)`. Its strength is negative edges, which flights never have. Used here as a test oracle instead. |
| Floyd-Warshall | All pairs at once, `O(n³)`. With ~6,000 airports that is about 2 × 10¹¹ steps to answer one question. |
| A* | Needs a heuristic that never overestimates. Straight-line distance would work for the distance cost, but not for the other cost policies. A possible extension. |

The heap uses **lazy deletion**: when a shorter distance to an airport is found, a new entry is
pushed and the outdated one is skipped when it is popped later. No decrease-key needed, and the
bound is still `O(m log n)` because `log m < 2 log n`.

### Shortest with at most K stops (`include/fp/algo/KStopSearch.hpp`)

Plain Dijkstra is **wrong** for this. The smallest example (limit: 1 stop):

```
A ──100──▶ B ──100──▶ C ──100──▶ D
│                     ▲
└─────────500─────────┘
```

- Dijkstra settles C at 200 (via B) and discards the 500 direct way into C.
- But A → B → C → D has 2 stops, which breaks the limit.
- The only legal route, A → C → D (600), needed the "worse" way into C that was thrown away.
  Plain Dijkstra reports "no route". (This exact graph is a unit test.)

The fix: "at C after 1 flight" and "at C after 2 flights" are **different states**, because they
leave different amounts of stop budget. The same Dijkstra runs over `(airport, flights used)`
pairs.

A pruning rule keeps it fast: states come out of the heap in cost order, so if C was already
settled with *fewer or equal* flights, any later state for C is both more expensive and has less
budget. It can be skipped without losing any answer.

The second method, implemented for cross-checking, is **Bellman-Ford limited to K + 1 rounds**
(`include/fp/algo/BellmanFord.hpp`). Round r means "cheapest using at most r flights", and the
key detail is that each round relaxes from a *copy* of the previous round. Updating in place
would let one round chain several flights and silently break the limit. (Tests confirm this:
removing the copy makes 4 test cases fail with 2,995 failed checks.)

### Hub dependency: Brandes (`src/analysis/HubAnalysis.cpp`)

For each destination, consider *all* fewest-flight routes to it, and give each connecting
airport a share equal to the fraction of those routes that pass through it. Summed over all
destinations, that is the airport's "route share".

Listing all routes would be exponential. Brandes' algorithm (2001, the standard method for
betweenness centrality) gets the same numbers from one BFS plus one backwards pass, `O(n + m)`:

- **Forward pass.** `σ[v]` = number of fewest-flight routes reaching v. Each one-ripple-outward
  edge `u → w` adds `σ[u]` to `σ[w]`.
- **Backward pass.** In reverse BFS order:
  `δ[u] += (σ[u] / σ[w]) · (1 + δ[w])` for each such edge.
  `σ[u] / σ[w]` is the fraction of routes into w that came through u.

That is the *soft* measure: how much routing would naturally pass through an airport. The
*hard* measure, `hub-impact`, closes the airport and re-runs BFS. `hubs` uses the soft measure
to pick candidates, then reports the hard one for each.

---

## Data structures, and why each one

| Structure | File | Used by | Why this one |
|---|---|---|---|
| Circular-buffer queue | `include/fp/ds/Queue.hpp` | BFS, hub analysis | O(1) push/pop, one contiguous block, no allocation per element. |
| Binary min-heap with pluggable comparator | `include/fp/ds/MinHeap.hpp` | Dijkstra, K-stop search | "Give me the cheapest" in O(log n). The comparator is how one Dijkstra serves three cost definitions. |
| AVL tree map | `include/fp/ds/AVLMap.hpp` | Airport code → id | Sorted order (prefix search, sorted listing) and a guaranteed O(log n) worst case. |

**On the AVL tree versus a hash map:** it does *not* save memory; both are `O(n)` space. The
reasons are ordered iteration, which makes `find DE` a walk from a starting key, and worst-case
guarantees. It is also kept out of the hot path entirely: the algorithms run on integer-indexed
vectors. Deletion is deliberately not implemented. Airports are never removed, and "close this
hub" is done with a flag rather than by editing the graph.

---

## Design decisions

**Cost policies instead of one hard-coded cost** (`include/fp/algo/CostPolicies.hpp`).
Dijkstra, the K-stop search and Bellman-Ford are templates over a small `Policy` struct
(`zero`, `extend`, `less`, `infinity`). Three policies exist: distance, flights, and
distance-then-flights (lexicographic). The compiler generates a separate, fully inlined version
per policy, so the flexibility costs nothing at run time. A policy must never decrease along a
path and must preserve order, which is the "no negative edges" condition in general form.

**Distances are integer metres, not floating-point kilometres** (`include/fp/graph/Edge.hpp`).
With integers, `a + b + c` gives the same result in any order, so Dijkstra and Bellman-Ford agree
*exactly* and tests compare costs with `==`. With doubles, two equally short routes can differ in
the 12th decimal, and tie-breaking turns into a coin flip.

**Exceptions for mistakes, `std::optional` for answers** (`include/fp/Errors.hpp`).
Unknown airport, negative stop limit, malformed file: throw, from one hierarchy rooted at
`PlannerError`. "There is no route" is a normal answer, so it is returned as `std::nullopt`.

**Throw on broken files, count known gaps** (`src/io/DataLoader.cpp`).
A wrong column count or unparseable latitude means the file is not what we think it is, so
loading stops with the file and line number. Airports without an IATA code and routes to unknown
airports are documented gaps in the dataset, so they are skipped and counted (`stats` shows them).

**Closing an airport is a flag, not a deletion** (`SearchOptions::blocked` in
`include/fp/algo/Itinerary.hpp`). The graph stays immutable, so a "what if" query can never leave
it damaged for the next query, and no copy is needed.

**No dependencies.** Even the test framework is ~100 lines in `tests/TestHarness.hpp`. Clone,
build, run.

---

## Complexity

`n` = airports (6,072), `m` = distinct directed routes (37,041), `K` = stop limit.

| Operation | Time | Extra space |
|---|---|---|
| Load dataset | `O(L + m log m)` for L input lines (sorting each adjacency list) | `O(n + m)` |
| Code lookup / prefix search | `O(log n)` / `O(log n + matches)` | `O(1)` |
| BFS, `reach`, `hub-impact` | `O(n + m)` | `O(n)` |
| Dijkstra | `O(m log n)` | `O(n + m)` worst-case heap |
| Shortest with ≤ K stops | `O(K · m · log(K · m))` worst case, far less with pruning | `O(n · K)` |
| Bellman-Ford, ≤ K stops | `O(K · m)` | `O(n)` |
| Brandes dependency | `O(n + m)` | `O(n)` |
| `hubs --top N` | `O(N · (n + m))` | `O(n)` |

---

## Testing

`./build/fp_tests` runs 43 test cases and about 885,000 checks in roughly 0.1 seconds.

- **Data structures** (`tests/test_ds.cpp`): each one is compared against the standard library on
  thousands of random operations. The AVL test inserts 100,000 *sorted* keys (the worst case for
  a plain BST) and checks the height stays under `1.44 · log₂ n`.
- **Loader** (`tests/test_loader.cpp`): quoted commas, escaped quotes, missing IATA codes,
  unknown airports, self loops, duplicate airlines, Windows line endings, and every structural
  error path.
- **Algorithms** (`tests/test_algo.cpp`): hand-built graphs, one per way an implementation can be
  wrong, including the K-stop counterexample above, zero-length edges, ties and blocked airports.
- **Random oracle** (`tests/test_random_oracle.cpp`): on 4,300 random graphs, brute force
  (every simple path), Bellman-Ford and the fast algorithms must agree on the optimum for every
  destination and every stop limit. Each returned route is also validated: correct endpoints,
  real flights only, within the stop limit, and its recomputed cost equals the optimum.
- **Hub analysis** (`tests/test_analysis.cpp`): the Brandes scores are compared against a brute
  force that lists every fewest-flight route.

**Do the tests actually catch bugs?** Two classic mistakes were planted on purpose and then
reverted:

| Planted bug | Result |
|---|---|
| K-stop search reduced to plain Dijkstra (one state per airport) | 3 test cases fail, 245 checks |
| Bellman-Ford relaxing in place instead of from the previous round | 4 test cases fail, 2,995 checks |

The full suite and the CLI commands also run clean under AddressSanitizer and
UndefinedBehaviorSanitizer.

---

## Benchmarks

`./build/fp_bench`: 300 random origin/destination pairs, same pairs for every row, Release build,
Intel Xeon @ 2.1 GHz. "settled" and "relaxed" are average airports popped and edges examined per
query, which explain the timings independently of the machine.

| Algorithm | µs / query | settled | relaxed | found |
|---|---:|---:|---:|---:|
| BFS (fewest flights) | 52 | 606 | 14,330 | 286 |
| Dijkstra (distance) | 360 | 1,676 | 21,566 | 286 |
| Dijkstra (distance-then-flights) | 407 | 1,676 | 21,566 | 286 |
| State-space Dijkstra, ≤ 0 stops | 5 | 13 | 12 | 0 |
| State-space Dijkstra, ≤ 1 stop | 27 | 188 | 746 | 13 |
| State-space Dijkstra, ≤ 2 stops | 152 | 872 | 6,480 | 92 |
| State-space Dijkstra, ≤ 3 stops | 372 | 1,705 | 17,585 | 207 |
| Bellman-Ford, ≤ 1 stop | 16 | | 821 | 13 |
| Bellman-Ford, ≤ 2 stops | 52 | | 8,595 | 92 |
| Bellman-Ford, ≤ 3 stops | 137 | | 31,301 | 207 |
| Bellman-Ford, no limit | 1,167 | | 382,058 | 286 |

Loading the dataset takes about 60 ms. `hubs BHO` takes under 1 ms.

What the numbers say, including the inconvenient part:

- **BFS is ~7× faster than Dijkstra** for the same kind of query, as the theory predicts: no heap,
  and it stops the moment the target is discovered.
- **For small stop limits, Bellman-Ford is faster than the state-space Dijkstra on this network**,
  even though it examines about twice as many edges. Its inner loop is a straight scan over
  arrays; Dijkstra pays for heap operations and for allocating `n · (K + 2)` state arrays per query.
  With no limit, Dijkstra wins by about 3×.
- The CLI still uses the state-space Dijkstra for `--max-stops` because it returns the actual
  route. Bellman-Ford here returns only the cost; returning the route would need a parent array
  per round, `O(n · K)` memory, the same as the Dijkstra version. Choosing between them by K is a
  reasonable next step.

---

## Data and its limits

Source: [OpenFlights](https://openflights.org/data.html), Open Database License. Details and
cleaning counts in [`data/README.md`](data/README.md).

- **2014 snapshot.** OpenFlights stopped updating routes around then.
- **Distance, not price.** There are no fares in the data, and inventing them would make every
  "cheapest" answer fiction. Great-circle distance is real and computable from coordinates. It
  is a lower bound on the distance actually flown, and the haversine formula assumes a spherical
  Earth (about 0.5% error).
- **No schedules.** A route means "some airline flew this pair", not "you can make this
  connection on the same day". Time-dependent routing needs timetable data and a different
  algorithm.

---

## Project layout

```
flight-planner/
├── CMakeLists.txt              build: library + CLI + tests + benchmark
├── data/                       OpenFlights airports.dat, routes.dat (+ README on licence and cleaning)
├── scripts/fetch_data.sh       re-download the dataset
├── include/fp/
│   ├── Errors.hpp              every exception type, and the throw-vs-optional rule
│   ├── ds/
│   │   ├── Queue.hpp           circular-buffer FIFO
│   │   ├── MinHeap.hpp         binary heap with pluggable comparator
│   │   └── AVLMap.hpp          self-balancing ordered map with prefix walk
│   ├── geo/Haversine.hpp       great-circle distance in metres
│   ├── graph/
│   │   ├── Airport.hpp         node data
│   │   ├── Edge.hpp            edge data, why distances are integers
│   │   └── FlightGraph.hpp     name layer + number layer
│   ├── io/
│   │   ├── Csv.hpp             quote-aware CSV line parser
│   │   └── DataLoader.hpp      OpenFlights loader and LoadStats
│   ├── algo/
│   │   ├── Itinerary.hpp       result type, SearchOptions (blocked airports, stats)
│   │   ├── CostPolicies.hpp    distance / flights / distance-then-flights
│   │   ├── BFS.hpp             fewest flights, reachability tree
│   │   ├── Dijkstra.hpp        shortest path, templated on a policy
│   │   ├── KStopSearch.hpp     shortest with at most K stops
│   │   └── BellmanFord.hpp     K-round Bellman-Ford (oracle and --verify)
│   └── analysis/HubAnalysis.hpp  reach, hub-impact, Brandes dependency, rankHubs
├── src/
│   ├── main.cpp                CLI: parse arguments, load, call, print
│   ├── graph/FlightGraph.cpp
│   ├── io/DataLoader.cpp
│   ├── algo/BFS.cpp
│   └── analysis/HubAnalysis.cpp
├── tests/                      TestHarness.hpp, GraphBuilders.hpp and one file per area
└── bench/bench_queries.cpp     timings and work counters on the real network
```

Templates (Dijkstra, K-stop search, Bellman-Ford, the data structures) are header-only because the
compiler needs their full code to generate one version per type. Everything else is split into a
header and a `.cpp`.

---

## Deliberately left out

| Left out | Why |
|---|---|
| AVL deletion | Never needed, and it is the most bug-prone part of an AVL tree. |
| Synthetic ticket prices | Made-up costs make every "cheapest" answer meaningless. |
| Time-dependent routing (schedules, layovers) | No timetable data, and it is a different algorithm. |
| Floyd-Warshall | `O(n³)` for a question that needs one source. |
| GUI or web front end | Out of scope. The library is separate from the CLI, so one could be added. |
| Custom vector or string | Reimplementing these adds risk and teaches nothing about graphs. |

Possible extensions: A* for the distance policy, decrease-key heap (indexed heap), picking
Bellman-Ford or Dijkstra automatically by K, k-shortest alternative routes (Yen's algorithm).
