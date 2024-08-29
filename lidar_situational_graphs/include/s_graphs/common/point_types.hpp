#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#include <pcl/point_types.h>

inline bool use_rgb = false;

if (!use_rgb)
  using PointT = pcl::PointXYZI;
else
  using PointT = pcl::PointXYZRGB;
using PointNormal = pcl::PointXYZRGBNormal;

#endif  // POINT_TYPES_H