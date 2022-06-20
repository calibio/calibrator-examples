// Read lens distortion maps and use them to undistort a new camera image.
//
// (c) Calib.io ApS

#include <opencv2/opencv.hpp>

bool readCalibDistortionMap(const std::string &filePathBase, cv::Matx33d &K,
                            cv::Mat_<float> &mapU, cv::Mat_<float> &mapV) {

  cv::FileStorage fs(filePathBase + "_K.json", cv::FileStorage::READ);

  try {
    fs["K"] >> K;
  } catch (...) {
    return false;
  }
  mapU = cv::imread(filePathBase + "_mapU.exr", cv::IMREAD_UNCHANGED);
  mapV = cv::imread(filePathBase + "_mapV.exr", cv::IMREAD_UNCHANGED);
  
  return true;
}

int main() {

  std::string filePathBase =
      std::string(SRCDIR) +
      "/../../data/pensylvania_c1/camera_0";

  cv::Matx33d K;
  cv::Mat_<float> mapU, mapV;
  readCalibDistortionMap(filePathBase, K, mapU, mapV);

  cv::Mat im = cv::imread(std::string(SRCDIR) +
                          "/../../data/pensylvania_c1/img_0020.png");
  cv::Mat imUndistorted;
  cv::remap(im, imUndistorted, mapU, mapV, cv::INTER_CUBIC);

  cv::imshow("im", im);
  cv::imshow("imUndistorted", imUndistorted);
  cv::waitKey(0);

  return 0;
}
