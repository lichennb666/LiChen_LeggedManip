#ifndef LIVOX_LASER_SIMULATION__CSV_READER_HPP_
#define LIVOX_LASER_SIMULATION__CSV_READER_HPP_

#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace livox_laser_simulation
{
class CsvReader
{
public:
  static bool ReadCsvFile(
    const std::string & file_name,
    std::vector<std::vector<double>> & rows)
  {
    std::ifstream input(file_name);
    if (!input.is_open()) {
      return false;
    }

    rows.clear();
    std::string line;
    bool first_line = true;
    while (std::getline(input, line)) {
      if (first_line) {
        first_line = false;
        continue;
      }
      if (line.empty()) {
        continue;
      }

      std::stringstream stream(line);
      std::string cell;
      std::vector<double> row;
      while (std::getline(stream, cell, ',')) {
        try {
          row.push_back(std::stod(cell));
        } catch (const std::exception &) {
          row.clear();
          break;
        }
      }
      if (!row.empty()) {
        rows.push_back(std::move(row));
      }
    }
    return !rows.empty();
  }
};
}  // namespace livox_laser_simulation

#endif  // LIVOX_LASER_SIMULATION__CSV_READER_HPP_
