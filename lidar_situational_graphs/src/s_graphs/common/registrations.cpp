#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <iostream>
#include <s_graphs/common/registrations.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

namespace s_graphs {

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr select_registration_method(
    registration_params params) {
  using PointT = pcl::PointXYZI;

  // select a registration method (ICP, GICP, NDT)
  std::string registration_method = params.registration_method;
  if (registration_method == "FAST_GICP") {
    std::cout << "registration: FAST_GICP" << std::endl;
    fast_gicp::FastGICP<PointT, PointT>::Ptr gicp(
        new fast_gicp::FastGICP<PointT, PointT>());
    gicp->setNumThreads(params.reg_num_threads);
    gicp->setTransformationEpsilon(params.reg_transformation_epsilon);
    gicp->setMaximumIterations(params.reg_maximum_iterations);
    gicp->setMaxCorrespondenceDistance(params.reg_max_correspondence_distance);
    gicp->setCorrespondenceRandomness(params.reg_correspondence_randomness);
    return gicp;
  }
#ifdef USE_VGICP_CUDA
  else if (registration_method == "FAST_VGICP_CUDA") {
    std::cout << "registration: FAST_VGICP_CUDA" << std::endl;
    fast_gicp::FastVGICPCuda<PointT, PointT>::Ptr vgicp(
        new fast_gicp::FastVGICPCuda<PointT, PointT>());
    vgicp->setResolution(pnh.param<double>("reg_resolution", 1.0));
    vgicp->setTransformationEpsilon(
        pnh.param<double>("reg_transformation_epsilon", 0.01));
    vgicp->setMaximumIterations(pnh.param<int>("reg_maximum_iterations", 64));
    vgicp->setCorrespondenceRandomness(
        pnh.param<int>("reg_correspondence_randomness", 20));
    return vgicp;
  }
#endif
  else if (registration_method == "FAST_VGICP") {
    std::cout << "registration: FAST_VGICP" << std::endl;
    fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp(
        new fast_gicp::FastVGICP<PointT, PointT>());
    vgicp->setNumThreads(params.reg_num_threads);
    vgicp->setResolution(params.reg_resolution);
    vgicp->setTransformationEpsilon(params.reg_transformation_epsilon);
    vgicp->setMaximumIterations(params.reg_maximum_iterations);
    vgicp->setCorrespondenceRandomness(params.reg_correspondence_randomness);
    return vgicp;
  } else if (registration_method == "ICP") {
    std::cout << "registration: ICP" << std::endl;
    pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp(
        new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setTransformationEpsilon(params.reg_transformation_epsilon);
    icp->setMaximumIterations(params.reg_maximum_iterations);
    icp->setMaxCorrespondenceDistance(params.reg_max_correspondence_distance);
    icp->setUseReciprocalCorrespondences(params.reg_use_reciprocal_correspondences);
    return icp;
  } else if (registration_method.find("GICP") != std::string::npos) {
    if (registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: GICP" << std::endl;
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(
          new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(params.reg_transformation_epsilon);
      gicp->setMaximumIterations(params.reg_maximum_iterations);
      gicp->setUseReciprocalCorrespondences(params.reg_use_reciprocal_correspondences);
      gicp->setMaxCorrespondenceDistance(params.reg_max_correspondence_distance);
      gicp->setCorrespondenceRandomness(params.reg_correspondence_randomness);
      gicp->setMaximumOptimizerIterations(params.reg_max_optimizer_iterations);
      return gicp;
    } else {
      std::cout << "registration: GICP_OMP" << std::endl;
      pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(
          new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(params.reg_transformation_epsilon);
      gicp->setMaximumIterations(params.reg_maximum_iterations);
      gicp->setUseReciprocalCorrespondences(params.reg_use_reciprocal_correspondences);
      gicp->setMaxCorrespondenceDistance(params.reg_max_correspondence_distance);
      gicp->setCorrespondenceRandomness(params.reg_correspondence_randomness);
      gicp->setMaximumOptimizerIterations(params.reg_max_optimizer_iterations);
      return gicp;
    }
  } else {
    if (registration_method.find("NDT") == std::string::npos) {
      std::cerr << "warning: unknown registration type(" << registration_method << ")"
                << std::endl;
      std::cerr << "       : use NDT" << std::endl;
    }

    double ndt_resolution = params.reg_resolution;
    if (registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: NDT " << ndt_resolution << std::endl;
      pcl::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(
          new pcl::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(params.reg_transformation_epsilon);
      ndt->setMaximumIterations(params.reg_maximum_iterations);
      ndt->setResolution(ndt_resolution);
      return ndt;
    } else {
      int num_threads = params.reg_num_threads;
      std::string nn_search_method = params.reg_nn_search_method;
      std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution
                << " (" << num_threads << " threads)" << std::endl;
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(
          new pclomp::NormalDistributionsTransform<PointT, PointT>());
      if (num_threads > 0) {
        ndt->setNumThreads(num_threads);
      }
      ndt->setTransformationEpsilon(params.reg_transformation_epsilon);
      ndt->setMaximumIterations(params.reg_maximum_iterations);
      ndt->setResolution(ndt_resolution);
      if (nn_search_method == "KDTREE") {
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      } else if (nn_search_method == "DIRECT1") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      }
      return ndt;
    }
  }

  return nullptr;
}

}  // namespace s_graphs
