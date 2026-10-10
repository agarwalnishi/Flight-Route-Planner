// Csv.hpp
//
// Splits one line of OpenFlights CSV into fields.
//
// Why not just split on commas: airport names can contain commas inside
// quotes, for example
//     1234,"Washington Dulles International Airport, Terminal B",...
// A naive split would shift every later column by one and silently put a
// longitude where the latitude should be.
//
// Rules handled (RFC 4180 style):
//   * fields may be wrapped in double quotes
//   * inside quotes, a comma is just a character
//   * inside quotes, "" means one literal " character
//   * a quote that is opened but never closed makes the line malformed
//
// Used by src/io/DataLoader.cpp. Defined inline here because it is small and
// the tests (tests/test_loader.cpp) call it directly.

#pragma once

#include <string>
#include <vector>

namespace fp {

// Fills `fields` with the parsed values. Returns false if the line is
// malformed (unterminated quote, or junk right after a closing quote).
// The caller decides what to do with a bad line; the loader turns it into
// a DataError that names the file and line number.
inline bool parseCsvLine(const std::string& line, std::vector<std::string>& fields) {
    fields.clear();
    std::string current;
    bool inQuotes = false;
    bool justClosedQuote = false;  // true right after a closing quote

    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (inQuotes) {
            if (c == '"') {
                if (i + 1 < line.size() && line[i + 1] == '"') {
                    current += '"';  // escaped quote: "" -> "
                    ++i;
                } else {
                    inQuotes = false;
                    justClosedQuote = true;
                }
            } else {
                current += c;
            }
            continue;
        }

        if (c == ',') {
            fields.push_back(std::move(current));
            current.clear();
            justClosedQuote = false;
        } else if (c == '"' && current.empty() && !justClosedQuote) {
            inQuotes = true;  // opening quote at the start of a field
        } else if (justClosedQuote) {
            return false;     // e.g. "abc"x  -> text after a closing quote
        } else {
            current += c;
        }
    }

    if (inQuotes) {
        return false;  // the line ended while still inside quotes
    }
    fields.push_back(std::move(current));
    return true;
}

}  // namespace fp
