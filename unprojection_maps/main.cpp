// Read unprojection maps and use them to lift a point in image pixel
// coordinates to its corresponding ray on the Z==1 plane.
//
//
// (c) Calib.io ApS

#include <opencv2/opencv.hpp>

bool readCalibUnprojectionMap(const std::string &filePathBase,
                              cv::Mat_<float> &mapX, cv::Mat_<float> &mapY) {

  mapX = cv::imread(filePathBase + "_mapX.exr", cv::IMREAD_UNCHANGED);
  mapY = cv::imread(filePathBase + "_mapY.exr", cv::IMREAD_UNCHANGED);

  return true;
}

float interp(const cv::Mat &img, cv::Point2f pt) {
  cv::Mat patch;
  cv::getRectSubPix(img, cv::Size(1, 1), pt, patch);
  return patch.at<float>(0, 0);
}

int main() {

  std::string filePathBase =
      std::string(SRCDIR) + "/../../data/pensylvania_c1/camera_0";

  cv::Mat_<float> mapX, mapY;
  readCalibUnprojectionMap(filePathBase, mapX, mapY);

  // we want to get the ray corresponding to pixel position [200.23, 103.356]
  cv::Point2f p(200.23, 103.356);
  float X = interp(mapX, p);
  float Y = interp(mapY, p);
  float Z = 1.0;

  return 0;
}
