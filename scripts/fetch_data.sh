#!/usr/bin/env sh
# fetch_data.sh
#
# Downloads a fresh copy of the two OpenFlights files this project reads into
# data/. The repository already ships them, so you only need this if they
# are missing or you want the latest upstream version.
#
#   sh scripts/fetch_data.sh
#
# Source: https://github.com/jpatokal/openflights (Open Database License).

set -eu

DATA_DIR="$(cd "$(dirname "$0")/.." && pwd)/data"
BASE="https://raw.githubusercontent.com/jpatokal/openflights/master/data"

mkdir -p "$DATA_DIR"
for f in airports.dat routes.dat; do
    echo "downloading $f"
    curl -fsSL -o "$DATA_DIR/$f.tmp" "$BASE/$f"
    mv "$DATA_DIR/$f.tmp" "$DATA_DIR/$f"  # only replace once the download succeeded
done
echo "done: $(wc -l < "$DATA_DIR/airports.dat") airports lines, $(wc -l < "$DATA_DIR/routes.dat") routes lines"
