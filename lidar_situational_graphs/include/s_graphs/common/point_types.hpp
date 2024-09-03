#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#include <pcl/point_types.h>

#ifdef USE_RGB_CLOUD
using PointT = pcl::PointXYZRGB;
#else
using PointT = pcl::PointXYZI;
#endif

using PointNormal = pcl::PointXYZRGBNormal;

#endif  // POINT_TYPES_H