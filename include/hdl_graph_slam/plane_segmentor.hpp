#ifndef PLANE_SEGMENTOR_HPP
#define PLANE_SEGMENTOR_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


namespace hdl_graph_slam {

class plane_segmentor {
public:
    plane_segmentor(){}
    ~plane_segmentor() {}

private:
    /* private variables of the class */

};
}


#endif  // PLANE_SEGMENTOR_HPP