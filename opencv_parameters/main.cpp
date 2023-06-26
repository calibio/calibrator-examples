#include <fstream>
#include <opencv2/opencv.hpp>

#include "json.hpp"
#include "readCalibParameters.h"

// OpenCV parameters. Demonstrates how Calibrator/libCalib results can be read
// and used with OpenCV.
// The function readCalibParameters() can be freely copied and used in
// customer projects.
//
// (c) Calib.io ApS, Public domain

int main() {

  // Example calibration exported from Calibrator for a system of two cameras
  // (stereo calibration)
  std::string jsonFilePath = std::string(SRCDIR) + "/stereo_calibration.json";

  // Read most common calibration parameters
  std::vector<cv::Matx33d> K;
  std::vector<cv::Vec<double, 14>> k;
  std::vector<cv::Vec3d> cam_rvecs;
  std::vector<cv::Vec3d> cam_tvecs;
  readCalibParameters(jsonFilePath, K, k, cam_rvecs, cam_tvecs);

  // Example: project a 3D point into both cameras
  std::vector<cv::Point3d> Q = {cv::Point3d(0.4, -0.02, 2.3)}; //[m]
  std::cout << "Q: " << Q << std::endl;
  std::vector<cv::Point2d> q0, q1;
  cv::projectPoints(Q, cam_rvecs[0], cam_tvecs[0], K[0], k[0], q0); // camera 0
  cv::projectPoints(Q, cam_rvecs[1], cam_tvecs[1], K[1], k[1], q1); // camera 1
  std::cout << "q0: " << q0[0] << std::endl;
  std::cout << "q1: " << q1[0] << std::endl;

  // undistort pixel coordinates
  std::vector<cv::Point2d> q0u, q1u;
  cv::undistortPoints(q0, q0u, K[0], k[0], cv::noArray(), K[0]);
  cv::undistortPoints(q1, q1u, K[1], k[1], cv::noArray(), K[1]);
  // undistorted points now obey the pinhole models K0 and K1

  // Example: compute the fundamental and essential matrices
  cv::Matx33d R0, R1;
  cv::Rodrigues(cam_rvecs[0], R0);
  cv::Rodrigues(cam_rvecs[1], R1);
  // cross product operator for the translation
  cv::Matx33d tx(0.0, -cam_tvecs[1][2], cam_tvecs[1][1], //
                 cam_tvecs[1][2], 0.0, -cam_tvecs[1][0], //
                 -cam_tvecs[1][1], cam_tvecs[1][0], 0.0);
  // essential matrix
  cv::Matx33d E = tx * R1;
  // fundamental matrix
  cv::Matx33d F = K[1].t().inv() * E * K[0].inv();
  // check - should be close to zero:
  std::cout << "q1u^T * F * q0u: "
            << cv::Vec3d(q1u[0].x, q1u[0].y, 1.0).t() * F *
                   cv::Vec3d(q0u[0].x, q0u[0].y, 1.0)
            << std::endl;

  // Example: triangulate back to 3D
  // construct linear projection matrices (P = K[R|t])
  // for camera 0 this will be K[I|0] as camera 0 is located at the origin of
  // the global/world coordinate system
  cv::Mat Rt0, Rt1;
  cv::hconcat(R0, cam_tvecs[0], Rt0);
  cv::hconcat(R1, cam_tvecs[1], Rt1);
  cv::Matx34d P0 = cv::Mat(K[0] * Rt0);
  cv::Matx34d P1 = cv::Mat(K[1] * Rt1);
  // correct matches - assuming gaussian noise this gives the max likelihood
  // 'optimal triangulation' result. without this step, triangulation will be
  // faster but non statistically optimal.
  std::vector<cv::Point2d> q0c, q1c;
  cv::correctMatches(F, q0u, q1u, q0c, q1c);
  // triangulate (DLT algorithm)
  cv::Vec4d Qhom;
  cv::triangulatePoints(P0, P1, q0c, q1c, Qhom);
  // convert to non-homogenous
  cv::Point3d Qtriang{Qhom(0) / Qhom(3), Qhom(1) / Qhom(3), Qhom(2) / Qhom(3)};
  std::cout << "Qtriang: " << Qtriang << std::endl;

  // Example: compute rectifying homographies which can warp images to match
  // line by line
  cv::Matx33d H0, H1;
  cv::Matx34d P0rect, P1rect;
  cv::Matx44d Qdtd;
  cv::Size frameSize(5120, 3840);
  cv::stereoRectify(K[0], k[0], K[1], k[1], frameSize, R1, cam_tvecs[1], H0, H1,
                    P0rect, P1rect, Qdtd);
  cv::Mat map0X, map0Y, map1X, map1Y;
  // precompute the remapping maps
  cv::initUndistortRectifyMap(K[0], k[0], H0, P0rect, frameSize, CV_32F, map0X,
                              map0Y);
  cv::initUndistortRectifyMap(K[1], k[1], H1, P1rect, frameSize, CV_32F, map1X,
                              map1Y);
  //  // do remap on actual images
  //  cv::remap(frame0, frame0Rect, map0X, map0Y, cv::INTER_LINEAR,
  //            cv::BORDER_CONSTANT, 0);
  //  cv::remap(frame1, frame1Rect, map1X, map1Y, cv::INTER_LINEAR,
  //            cv::BORDER_CONSTANT, 0);
  // triangulate a point by means of disparity projection
  cv::Matx33f H0invx = cv::Matx33f(cv::Mat(H0.t()));
  const float disp = 5.0;  // disparity value[px]
  const double row = 2000; // row coordinate in both cameras
  const double col = 500;  // col coordinate in camera 0
  cv::Vec4f Qih = Qdtd * cv::Vec4f(col, row, disp, 1.0);
  cv::Point3f Qdisp =
      H0invx * cv::Point3f(Qih[0] / Qih[3], Qih[1] / Qih[3], Qih[2] / Qih[3]);

  ////////////////////////////////////////////////////////////////////////////
  /// Read additional results (poses of the calibration rig, optimized target
  /// points and target poses (if more than one, network calibration))
  ////////////////////////////////////////////////////////////////////////////
  std::ifstream fileStream(jsonFilePath);
  nlohmann::json jsonStruct;
  try {
    fileStream >> jsonStruct;
  } catch (...) {
    return -1;
  }

  // Calibration rig poses (relative to camera 0)
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

  // Target object points (may be nominal or optimized)
  int nTargets = jsonStruct["Calibration"]["targets"].size();
  std::vector<std::vector<cv::Point3d>> target_objPoints(nTargets);
  for (int i = 0; i < nTargets; ++i) {
    auto objPointsI = jsonStruct["Calibration"]["targets"][i]["objectPoints"];
    for (size_t j = 0; j < objPointsI.size(); ++j) {
      auto P = objPointsI[j];
      target_objPoints[i].emplace_back(P["x"], P["y"], P["z"]);
    }
  }

  // Target poses (relative to target 0 -- only relevant in network
  // calibration)
  std::vector<cv::Vec3d> target_rvecs(nTargets);
  std::vector<cv::Vec3d> target_tvecs(nTargets);

  for (int i = 0; i < nTargets; ++i) {
    auto poseI = jsonStruct["Calibration"]["targets"][i]["transform"];

    auto rot = poseI["rotation"];
    target_rvecs[i] = {rot["rx"], rot["ry"], rot["rz"]};

    auto t = poseI["translation"];
    target_tvecs[i] = cv::Vec3d(t["x"], t["y"], t["z"]);
  }

  // Example: project the object point with id 128 belonging to target 0 in
  // pose 5 into camera 1
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
