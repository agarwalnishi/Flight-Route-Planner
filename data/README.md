# Data

`airports.dat` and `routes.dat` come from the [OpenFlights](https://openflights.org/data.html)
project ([GitHub mirror](https://github.com/jpatokal/openflights)), made available under the
[Open Database License](https://opendatacommons.org/licenses/odbl/1-0/). They are included
unchanged. Run `sh scripts/fetch_data.sh` to download them again.

Things to know before trusting any number this project prints:

- **Routes are a 2014 snapshot.** OpenFlights stopped updating the route database around
  June 2014. Treat results as "how this method answers the question", not "how Bhopal is
  connected today".
- **No fares, no schedules.** A route means "some airline flew this pair". Cost in this
  project is great-circle distance, computed from the airport coordinates
  (`include/fp/geo/Haversine.hpp`).
- **Cleaning** (counts from `flight-planner stats`):
  - 1,626 airports have no IATA code and are skipped, because routes refer to airports by IATA code.
  - 729 routes point at an airport that is not in `airports.dat`, so they are skipped.
  - 1 route goes from an airport to itself, and it is skipped too.
  - The 66,933 remaining airline routes collapse to 37,041 distinct directed airport pairs.
