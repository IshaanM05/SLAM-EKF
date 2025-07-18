#ifndef LOAD_LANDMARKS_HPP_
#define LOAD_LANDMARKS_HPP_

#include <vector>
#include <string>
#include <Eigen/Dense>

std::vector<Eigen::Vector3d> loadLandmarksFromCSV(const std::string& csv_path);

#endif  // LOAD_LANDMARKS_HPP_
