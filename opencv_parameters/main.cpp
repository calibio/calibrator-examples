#include <fstream>
#include <opencv2/opencv.hpp>

#include "json.hpp"

// OpenCV parameters. Demonstrates how libCalib results can be read and used
// with OpenCV.
//
// (c) Calib.io ApS

// Read Calib Camera Calibrator parameters into OpenCV datastructures, suitable
// for further use with OpenCV function. Uses nlohmann's single-header "JSON for
// Modern C++" for JSON import. See https://github.com/nlohmann/json
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

int main() {
  std::string jsonFilePath =
      std::string(SRCDIR) + "/../data/opencv_parameters.json";

  // read the most commonly used calibration results
  std::vector<cv::Matx33d> K;
  std::vector<cv::Vec<double, 14>> k;
  std::vector<cv::Vec3d> cam_rvecs;
  std::vector<cv::Vec3d> cam_tvecs;
  readCalibParameters(jsonFilePath, K, k, cam_rvecs, cam_tvecs);

  // read additional calibration optimization results
  std::ifstream fileStream(jsonFilePath);
  nlohmann::json jsonStruct;
  try {
    fileStream >> jsonStruct;
  } catch (...) {
    return -1;
  }

  // calibration rig poses (relative to camera 0)
  int nPoses = jsonStruct["Calibration"]["poses"].size();
  std::vector<cv::Vec3d> pose_rvecs(nPoses);
  std::vector<cv::Vec3d> pose_tvecs(nPoses);

  for (int i = 0; i < nPoses; ++i) {
    auto poseI = jsonStruct["Calibration"]["poses"][i]["transform"];

    auto rot = poseI["rotation"];
    pose_rvecs[i] = {rot["rx"], rot["ry"], rot["rz"]};

    auto t = poseI["translation"];
    pose_tvecs[i] = cv::Vec3d(t["x"], t["y"], t["z"]);
  }

  // target object points (may be nominal or optimized)
  int nTargets = jsonStruct["Calibration"]["targets"].size();
  std::vector<std::vector<cv::Point3d>> target_objPoints(nTargets);
  for (int i = 0; i < nTargets; ++i) {
    auto objPointsI = jsonStruct["Calibration"]["targets"][i]["objectPoints"];
    for (size_t j = 0; j < objPointsI.size(); ++j) {
      auto P = objPointsI[j];
      target_objPoints[i].emplace_back(P["x"], P["y"], P["z"]);
    }
  }

  // target poses (relative to target 0 -- only relevant in network calibration)
  std::vector<cv::Vec3d> target_rvecs(nTargets);
  std::vector<cv::Vec3d> target_tvecs(nTargets);

  for (int i = 0; i < nTargets; ++i) {
    auto poseI = jsonStruct["Calibration"]["targets"][i]["transform"];

    auto rot = poseI["rotation"];
    target_rvecs[i] = {rot["rx"], rot["ry"], rot["rz"]};

    auto t = poseI["translation"];
    target_tvecs[i] = cv::Vec3d(t["x"], t["y"], t["z"]);
  }

  // example: project object point with id 128 belonging to target 0 in pose 5
  // into camera 1
  const int pointId = 128;
  const int targetId = 0;
  const int poseId = 5;
  const int camId = 1;

  cv::Vec3d rvec, tvec;
  cv::composeRT(target_rvecs[targetId], target_tvecs[targetId],
                pose_rvecs[poseId], pose_tvecs[poseId], rvec, tvec);
  cv::composeRT(rvec, tvec, cam_rvecs[camId], cam_tvecs[camId], rvec, tvec);

  std::vector<cv::Point3d> P = {target_objPoints[targetId][pointId]};
  std::vector<cv::Point2d> p;
  cv::projectPoints(P, rvec, tvec, K[camId], k[camId], p);
  std::cout << "p = " << p << std::endl;
}
