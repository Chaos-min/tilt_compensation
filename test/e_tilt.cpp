#include "tilt_compensation.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: tilt_compensation_file_example <input.txt> <output.txt>\n";
        return 1;
    }

    try {
        std::ifstream input(argv[1]);
        if (!input) {
            throw std::runtime_error("cannot open input file");
        }

        std::vector<tilt_compensation::Point3f> points;
        std::vector<std::string> remaining_columns;
        std::string line;
        std::size_t line_number = 0;

        while (std::getline(input, line)) {
            ++line_number;
            std::istringstream row(line);
            tilt_compensation::Point3f point;
            if (!(row >> point.x >> point.y >> point.z)) {
                throw std::runtime_error(
                    "invalid XYZ at line " + std::to_string(line_number));
            }

            std::string remaining;
            std::getline(row, remaining);
            points.push_back(point);
            remaining_columns.push_back(remaining);
        }

        tilt_compensation::compensateInPlace(
            points,
            tilt_compensation::TiltAngles{1.2210, -0.6120},
            tilt_compensation::TiltAngles{0.0, 0.0});

        std::ofstream output(argv[2]);
        if (!output) {
            throw std::runtime_error("cannot open output file");
        }

        output << std::fixed << std::setprecision(4);
        for (std::size_t i = 0; i < points.size(); ++i) {
            output << points[i].x << ' ' << points[i].y << ' ' << points[i].z
                   << remaining_columns[i] << '\n';
        }
        if (!output) {
            throw std::runtime_error("failed to write output file");
        }
        std::cout << "Output file: " << argv[2] << " written successfully" << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
