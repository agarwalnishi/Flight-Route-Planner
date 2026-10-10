// test_loader.cpp
//
// Tests for CSV parsing (include/fp/io/Csv.hpp), the haversine distance
// (include/fp/geo/Haversine.hpp), the graph container
// (src/graph/FlightGraph.cpp) and the dataset loader (src/io/DataLoader.cpp).
//
// The loader tests write small fake OpenFlights files into the system temp
// folder, so they cover every awkward case on purpose: quoted commas,
// missing IATA codes, unknown airports, self loops, duplicate airlines and
// Windows line endings.

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "TestHarness.hpp"
#include "fp/Errors.hpp"
#include "fp/geo/Haversine.hpp"
#include "fp/graph/FlightGraph.hpp"
#include "fp/io/Csv.hpp"
#include "fp/io/DataLoader.hpp"

using namespace fp;

namespace {

// Writes `content` to a fresh temp file and deletes it when done.
struct TempFile {
    explicit TempFile(const std::string& name, const std::string& content)
        : path((std::filesystem::temp_directory_path() / ("fp_test_" + name)).string()) {
        std::ofstream out(path, std::ios::binary);  // binary: keep \r\n as written
        out << content;
    }
    ~TempFile() { std::remove(path.c_str()); }
    std::string path;
};

Airport makeAirport(const std::string& code, double lat, double lon) {
    Airport a;
    a.code = code;
    a.latitude = lat;
    a.longitude = lon;
    return a;
}

}  // namespace

// --------------------------------------------------------------------- CSV

TEST_CASE(csv_plain_and_quoted_fields) {
    std::vector<std::string> f;
    REQUIRE(parseCsvLine(R"(1,"Goroka Airport",Goroka,"GKA",-6.08)", f));
    CHECK_EQ(f.size(), 5u);
    CHECK_EQ(f[1], std::string("Goroka Airport"));
    CHECK_EQ(f[3], std::string("GKA"));
}

TEST_CASE(csv_comma_inside_quotes_is_one_field) {
    std::vector<std::string> f;
    REQUIRE(parseCsvLine(R"(7,"Dulles, Terminal B","Washington",X)", f));
    CHECK_EQ(f.size(), 4u);
    CHECK_EQ(f[1], std::string("Dulles, Terminal B"));
}

TEST_CASE(csv_escaped_quote_and_empty_fields) {
    std::vector<std::string> f;
    REQUIRE(parseCsvLine(R"(a,"say ""hi""",,"")", f));
    CHECK_EQ(f.size(), 4u);
    CHECK_EQ(f[1], std::string("say \"hi\""));
    CHECK_EQ(f[2], std::string(""));
    CHECK_EQ(f[3], std::string(""));
}

TEST_CASE(csv_malformed_lines_are_rejected) {
    std::vector<std::string> f;
    CHECK(!parseCsvLine(R"(1,"never closed)", f));
    CHECK(!parseCsvLine(R"(1,"closed"junk,2)", f));
}

// --------------------------------------------------------------- Haversine

TEST_CASE(haversine_known_distances) {
    // Same point.
    CHECK_EQ(greatCircleMeters(23.2875, 77.3374, 23.2875, 77.3374), Meters{0});
    // Quarter of the equator: pi/2 * R.
    const Meters quarter = greatCircleMeters(0, 0, 0, 90);
    CHECK(std::llabs(quarter - 10007557) < 2);
    // Pole to pole along a meridian: pi * R.
    const Meters half = greatCircleMeters(90, 0, -90, 0);
    CHECK(std::llabs(half - 20015115) < 2);
    // Bhopal (BHO) to Delhi (DEL): about 590 km. Symmetric.
    const Meters bhoDel = greatCircleMeters(23.2875, 77.3374, 28.5665, 77.1031);
    CHECK(bhoDel > 580000 && bhoDel < 600000);
    CHECK_EQ(bhoDel, greatCircleMeters(28.5665, 77.1031, 23.2875, 77.3374));
}

// ------------------------------------------------------------- FlightGraph

TEST_CASE(graph_finalize_merges_duplicates_and_sorts) {
    FlightGraph g;
    const int a = g.addAirport(makeAirport("AAA", 0, 0));
    const int b = g.addAirport(makeAirport("BBB", 0, 1));
    const int c = g.addAirport(makeAirport("CCC", 0, 2));
    g.addRoute(a, c, 300);
    g.addRoute(a, b, 120);
    g.addRoute(a, b, 100);  // second airline, shorter recorded length
    g.addRoute(a, b, 110);
    g.finalize();

    const std::vector<Edge>& out = g.neighbors(a);
    REQUIRE(out.size() == 2u);
    CHECK_EQ(out[0].to, b);  // sorted by destination id
    CHECK_EQ(out[0].meters, Meters{100});  // shortest kept
    CHECK_EQ(out[0].airlines, 3);          // airline count summed
    CHECK_EQ(out[1].to, c);
    CHECK(g.edgeBetween(a, b) != nullptr);
    CHECK(g.edgeBetween(b, a) == nullptr);  // edges are directed
    CHECK_EQ(g.routeCount(), 2u);
}

TEST_CASE(graph_lookup_is_case_insensitive_and_validates) {
    FlightGraph g;
    const int id = g.addAirport(makeAirport("bho", 23, 77));
    CHECK_EQ(g.idOf("BHO"), id);
    CHECK_EQ(g.idOf("bho"), id);
    CHECK(!g.findId("XXX").has_value());
    CHECK_THROWS_AS(g.idOf("XXX"), UnknownAirport);
    CHECK_THROWS_AS(g.addAirport(makeAirport("BHO", 0, 0)), InvalidQuery);  // duplicate code
    CHECK_THROWS_AS(g.addAirport(makeAirport("", 0, 0)), InvalidQuery);
    CHECK_THROWS_AS(g.addRoute(id, 5, 10), InvalidQuery);   // unknown id
    CHECK_THROWS_AS(g.addRoute(id, id, -1), InvalidQuery);  // negative length
    CHECK_THROWS_AS(g.airport(-1), InvalidQuery);
}

TEST_CASE(graph_prefix_lookup) {
    FlightGraph g;
    for (const char* code : {"DEN", "DEL", "BHO", "DED", "BOM"}) g.addAirport(makeAirport(code, 0, 0));
    std::vector<std::string> found;
    for (int id : g.idsWithPrefix("de", 10)) found.push_back(g.airport(id).code);
    CHECK(found == (std::vector<std::string>{"DED", "DEL", "DEN"}));
    CHECK_EQ(g.idsWithPrefix("D", 2).size(), 2u);  // limit respected
    CHECK(g.idsWithPrefix("Q", 10).empty());
}

// ------------------------------------------------------------------ Loader

TEST_CASE(loader_handles_every_known_quirk) {
    // airports: one with a quoted comma, one without IATA (\N), CRLF endings.
    TempFile airports("airports.dat",
                      "1,\"Alpha, Main\",Alpha,Land,\"AAA\",\"XAAA\",10.0,20.0,0,0,\"U\",\"x\",\"airport\",\"t\"\r\n"
                      "2,\"Beta\",Beta,Land,\"BBB\",\"XBBB\",11.0,21.0,0,0,\"U\",\"x\",\"airport\",\"t\"\r\n"
                      "3,\"NoCode\",Gamma,Land,\\N,\"XCCC\",12.0,22.0,0,0,\"U\",\"x\",\"airport\",\"t\"\r\n"
                      "\r\n");
    // routes: two airlines on AAA->BBB, an unknown airport, a self loop.
    TempFile routes("routes.dat",
                    "AA,1,AAA,1,BBB,2,,0,320\n"
                    "BB,2,AAA,1,BBB,2,Y,0,320\n"
                    "CC,3,BBB,2,AAA,1,,0,320\n"
                    "DD,4,AAA,1,ZZZ,99,,0,320\n"
                    "EE,5,AAA,1,AAA,1,,0,320\n");

    LoadStats s;
    const FlightGraph g = loadOpenFlights(airports.path, routes.path, &s);
    CHECK_EQ(s.airportLines, 3u);  // blank line ignored
    CHECK_EQ(s.airportsLoaded, 2u);
    CHECK_EQ(s.airportsWithoutIata, 1u);
    CHECK_EQ(s.routeLines, 5u);
    CHECK_EQ(s.routesUsed, 3u);
    CHECK_EQ(s.routesUnknownAirport, 1u);
    CHECK_EQ(s.routesSelfLoop, 1u);
    CHECK_EQ(s.distinctEdges, 2u);  // AAA->BBB merged, plus BBB->AAA

    const int a = g.idOf("AAA");
    CHECK_EQ(g.airport(a).name, std::string("Alpha, Main"));
    const Edge* e = g.edgeBetween(a, g.idOf("BBB"));
    REQUIRE(e != nullptr);
    CHECK_EQ(e->airlines, 2);
    CHECK_EQ(e->meters, greatCircleMeters(10.0, 20.0, 11.0, 21.0));
}

TEST_CASE(loader_throws_on_structural_problems) {
    TempFile goodRoutes("routes_ok.dat", "AA,1,AAA,1,BBB,2,,0,320\n");

    TempFile tooFewColumns("few.dat", "1,\"Alpha\",Alpha,Land,\"AAA\"\n");
    CHECK_THROWS_AS(loadOpenFlights(tooFewColumns.path, goodRoutes.path), DataError);

    TempFile badLatitude("badlat.dat", "1,\"A\",A,L,\"AAA\",\"X\",north,20.0,0\n");
    CHECK_THROWS_AS(loadOpenFlights(badLatitude.path, goodRoutes.path), DataError);

    TempFile outOfRange("range.dat", "1,\"A\",A,L,\"AAA\",\"X\",95.0,20.0,0\n");
    CHECK_THROWS_AS(loadOpenFlights(outOfRange.path, goodRoutes.path), DataError);

    TempFile brokenQuote("quote.dat", "1,\"Alpha,Alpha,Land,AAA,X,1,2,0\n");
    CHECK_THROWS_AS(loadOpenFlights(brokenQuote.path, goodRoutes.path), DataError);

    TempFile duplicate("dup.dat", "1,\"A\",A,L,\"AAA\",\"X\",1,2,0\n2,\"B\",B,L,\"AAA\",\"Y\",3,4,0\n");
    CHECK_THROWS_AS(loadOpenFlights(duplicate.path, goodRoutes.path), DataError);

    CHECK_THROWS_AS(loadOpenFlights("/definitely/not/here.dat", goodRoutes.path), DataError);
}
