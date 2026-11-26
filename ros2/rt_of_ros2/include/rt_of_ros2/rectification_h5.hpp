#pragma once

#include <H5Cpp.h>
#include <opencv2/core.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace rt_of_ros2
{

inline cv::Mat load_rectification_h5(
  const std::string& filename,
  int width,
  int height)
{
  if (width <= 0 || height <= 0)
    throw std::runtime_error("Invalid rectification dimensions");

  H5::H5File file(filename, H5F_ACC_RDONLY);
  auto dataset = file.openDataSet("/rectify_map");
  auto space = dataset.getSpace();

  if (space.getSimpleExtentNdims() != 3)
    throw std::runtime_error("Expected a 3D rectify_map");

  hsize_t dims[3];
  space.getSimpleExtentDims(dims);

  if (dims[0] != static_cast<hsize_t>(height) ||
      dims[1] != static_cast<hsize_t>(width) ||
      dims[2] != 2)
    throw std::runtime_error("Invalid rectify_map dimensions");

  std::vector<float> data(
    static_cast<size_t>(height) * width * 2);

  dataset.read(data.data(), H5::PredType::NATIVE_FLOAT);

  cv::Mat map(height, width, CV_32SC2);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {

      const size_t i =
        (static_cast<size_t>(y) * width + x) * 2;

      const double rx = data[i];
      const double ry = data[i + 1];

      if (!std::isfinite(rx) || !std::isfinite(ry) ||
          rx < std::numeric_limits<int>::min() ||
          rx > std::numeric_limits<int>::max() ||
          ry < std::numeric_limits<int>::min() ||
          ry > std::numeric_limits<int>::max()) {

        map.at<cv::Point2i>(y, x) = cv::Point2i(-1, -1);

      } else {

        map.at<cv::Point2i>(y, x) = cv::Point2i(
          static_cast<int>(rx),
          static_cast<int>(ry));
      }
    }
  }

  return map;
}

}
