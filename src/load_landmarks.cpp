#include "motion_update_pkg/load_landmarks.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<Eigen::Vector3d> loadLandmarksFromCSV(const std::string& csv_path) {
    std::vector<Eigen::Vector3d> landmarks;
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open landmark CSV file: " << csv_path << std::endl;
        return landmarks;
    }

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string tag, x_str, y_str, direction_str;
        std::getline(ss, tag, ',');
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, direction_str, ',');

        double x = std::stod(x_str);
        double y = std::stod(y_str);
        int color = std::stoi(direction_str);
        landmarks.emplace_back(Eigen::Vector3d(x, y, color));
    }

    return landmarks;
}
