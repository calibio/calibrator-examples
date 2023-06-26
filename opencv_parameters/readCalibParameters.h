#pragma once

#include <opencv2/opencv.hpp>

bool readCalibParameters(const std::string &filePath,
                         std::vector<cv::Matx33d> &K,
                         std::vector<cv::Vec<double, 14>> &k,
                         std::vector<cv::Vec3d> &cam_rvecs,
                         std::vector<cv::Vec3d> &cam_tvecs);
