#include "readCalibParameters.h"
#include "fstream"
#include "json.hpp"

// Read Calib Camera Calibrator parameters into OpenCV datastructures, suitable
// for further use with OpenCV function. Uses nlohmann's single-header "JSON for
// Modern C++" for JSON import. See https://github.com/nlohmann/json
//
// (c) Calib.io ApS, Public domain

bool readCalibParameters(const std::string &filePath,
                         std::vector<cv::Matx33d> &K,
                         std::vector<cv::Vec<double, 14>> &k,
                         std::vector<cv::Vec3d> &cam_rvecs,
                         std::vector<cv::Vec3d> &cam_tvecs) {

  std::ifstream fileStream(filePath);
  nlohmann::json jsonStruct;
  try {
    fileStream >> jsonStruct;
  } catch (...) {
    return false;
  }

  assert(jsonStruct["Calibration"]["cameras"][0]["model"]["polymorphic_name"] ==
         "libCalib::CameraModelOpenCV");

  int nCameras = jsonStruct["Calibration"]["cameras"].size();

  if (nCameras < 1) {
    return false;
  }

  K.resize(nCameras);
  k.resize(nCameras);

  cam_rvecs.resize(nCameras);
  cam_tvecs.resize(nCameras);

  for (int i = 0; i < nCameras; ++i) {
    auto intrinsics = jsonStruct["Calibration"]["cameras"][i]["model"]
                                ["ptr_wrapper"]["data"]["parameters"];

    double f = intrinsics["f"]["val"];
    double ar = intrinsics["ar"]["val"];
    double cx = intrinsics["cx"]["val"];
    double cy = intrinsics["cy"]["val"];
    double k1 = intrinsics["k1"]["val"];
    double k2 = intrinsics["k2"]["val"];
    double k3 = intrinsics["k3"]["val"];
    double k4 = intrinsics["k4"]["val"];
    double k5 = intrinsics["k5"]["val"];
    double k6 = intrinsics["k6"]["val"];
    double p1 = intrinsics["p1"]["val"];
    double p2 = intrinsics["p2"]["val"];
    double s1 = intrinsics["s1"]["val"];
    double s2 = intrinsics["s2"]["val"];
    double s3 = intrinsics["s3"]["val"];
    double s4 = intrinsics["s4"]["val"];
    double tauX = intrinsics["tauX"]["val"];
    double tauY = intrinsics["tauY"]["val"];

    K[i] = cv::Matx33d(f, 0.0, cx, 0.0, f * ar, cy, 0.0, 0.0, 1.0);
    k[i] = cv::Vec<double, 14>(k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4,
                               tauX, tauY);

    auto transform = jsonStruct["Calibration"]["cameras"][i]["transform"];

    auto rot = transform["rotation"];
    cam_rvecs[i] = {rot["rx"], rot["ry"], rot["rz"]};

    auto t = transform["translation"];
    cam_tvecs[i] = cv::Vec3d(t["x"], t["y"], t["z"]);
  }

  return true;
}
