/**
 * @Date: 2023-01-06 15:29:34
 * @LastEditTime: 2023-01-06 15:29:34
 * @LastEditors: Shane
 * @Description: 料知此生无大事，关心雪后有梅花.
 */
#pragma once
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "include/yaml-cpp/yaml.h"
namespace myslam {
using namespace Eigen;
class Mapping_front {
 public:
  Mapping_front();
  ~Mapping_front();

  // initial calss.
  bool Initial(const std::string yaml_path);

  // cloud matching.
  bool CloudMatchingFlow(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& source);

  // save the transformed cloud.
  bool SavetheTransformedcloud();

  // filter the pointclous.
  bool CloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& output);

  // get result.
  Matrix4f RoughReslut() { return rough_trans_; };
  Matrix4f PreciseReslut() { return precise_trans_; };
  double GetMatchingscore() { return score_; };
  double GetMatchinglfinalscore() { return last_score_; };

 private:
  // get the paramenters from yaml.
  bool GetParamentersFromYaml(const std::string yaml_path);

  // initial filter.
  bool InitialFilter();

  // initial CloudRegistration.
  bool InitialCloudRegistration();

  // matching funcation.
  template <typename Registration>
  bool CloudMatching(Registration& reg,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& source);
  // get the precise result.
  bool GetPreciseResult(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source);
  // yaml paramenters.
  float voxel_size_;
  float map_size_;
  int num_mean_;
  float mul_thro_;
  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;
  float min_z_;
  float max_z_;
  int skip_frame_;
  int frames_num_;
  int matching_way_;
  int filter_way_;
  float resolution_;
  int numthread_;
  int numiteration_;
  // filter.
  pcl::CropBox<pcl::PointXYZ> box_filter;
  pcl::CropBox<pcl::PointXYZ> submap_filter;
  pcl::VoxelGrid<pcl::PointXYZ> downSizeFiltermap;
  pcl::VoxelGrid<pcl::PointXYZ> summapFiltermap;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // result.
  Matrix4f rough_trans_;
  Matrix4f precise_trans_;
  double score_;
  double last_score_;
  // ICP and orther matching way.
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
  fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_cuda;
  fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ> ndt_cuda;
  fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp_mt;
  // matching cloud result.
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> trans_cloud_;
};

}  // namespace myslam
