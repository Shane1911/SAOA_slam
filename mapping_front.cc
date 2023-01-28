/*
 * @Date: 2023-01-06 15:29:41
 * @LastEditTime: 2023-01-12 15:55:50
 * @LastEditors: Shane
 * @Description: 料知此生无大事，关心雪后有梅花.
 */
#include "mapping_front.h"

namespace myslam {

Mapping_front::Mapping_front() { LOG(INFO) << "mapping front begaining!!!"; }

Mapping_front::~Mapping_front() {}

bool Mapping_front::Initial(const std::string yaml_path) {
  if (!GetParamentersFromYaml(yaml_path))
    LOG(INFO) << "Getting paramenters from yaml failed!!!";
  if (!InitialFilter()) LOG(INFO) << "Initial filter failed!!!";
  if (!InitialCloudRegistration())
    LOG(INFO) << "Initial CloudRegistration failed!!!";

  trans_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  return true;
}

bool Mapping_front::GetParamentersFromYaml(const std::string yaml_path) {
  YAML::Node config_node = YAML::LoadFile(yaml_path);
  voxel_size_ = config_node["leaf_size"].as<float>();
  map_size_ = config_node["map_size"].as<float>();
  num_mean_ = config_node["meank"].as<int>();
  mul_thro_ = config_node["StddevMulThresh"].as<float>();
  min_x_ = config_node["min_x"].as<float>();
  max_x_ = config_node["max_x"].as<float>();
  min_y_ = config_node["min_y"].as<float>();
  max_y_ = config_node["max_y"].as<float>();
  min_z_ = config_node["min_z"].as<float>();
  max_z_ = config_node["max_z"].as<float>();
  filter_way_ = config_node["filter_way"].as<int>();
  matching_way_ = config_node["matching_way"].as<int>();
  resolution_ = config_node["resolution"].as<float>();
  numthread_ = config_node["numthread"].as<int>();
  numiteration_ = config_node["numiteration"].as<int>();
  LOG(INFO)
      << "\n ===================The front_end paramenters================= \n"
      << "lidar frame filter size: " << voxel_size_ << "\n"
      << "full map filter size: " << map_size_ << "\n"
      << "StatisticalOutlierRemoval paramenters: " << num_mean_ << ", "
      << mul_thro_ << "\n"
      << "cloud matching resolution: " << resolution_ << "\n"
      << "cloud matching numthread: " << numthread_ << "\n"
      << "cloud matching numiteration: " << numiteration_ << "\n";
  return true;
}

bool Mapping_front::InitialFilter() {
  box_filter.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1e-6));
  box_filter.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1e6));
  downSizeFiltermap.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  summapFiltermap.setLeafSize(map_size_, map_size_, map_size_);
  sor.setMeanK(num_mean_);
  sor.setStddevMulThresh(mul_thro_);
  return true;
}

bool Mapping_front::InitialCloudRegistration() {
  switch (matching_way_) {
    case 1:
      // ==========================VGICP-mt===================
      // static fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp_mt;
      vgicp_mt.setResolution(resolution_);
      vgicp_mt.setNumThreads(numthread_);
      vgicp_mt.setMaximumIterations(numiteration_);
      LOG(INFO) << "Cloud matching way is: VGICP-mt";
      break;
    case 2:
      // ====================VGICP-cuda=======================
      // static fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ>
      // vgicp_cuda;
      vgicp_cuda.setResolution(resolution_);
      vgicp_cuda.setNearestNeighborSearchMethod(
          fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
      vgicp_cuda.setKernelWidth(0.5);
      LOG(INFO) << "Cloud matching way is: VGICP-cuda";
      break;
    default:
      //=====================GPU-NDT==========================
      // static fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ> ndt_cuda;
      ndt_cuda.setResolution(resolution_);
      ndt_cuda.setMaximumIterations(numiteration_);
      ndt_cuda.setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
      LOG(INFO) << "Cloud matching way is: GPU-NDT";
      break;
  }
  icp_.setMaxCorrespondenceDistance(0.05);
  // Set the maximum number of iterations (criterion 1)
  icp_.setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  icp_.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp_.setEuclideanFitnessEpsilon(1);
  return true;
}

template <typename Registration>
bool Mapping_front::CloudMatching(
    Registration& reg, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(
      new pcl::PointCloud<pcl::PointXYZ>);
  reg.clearTarget();
  reg.clearSource();
  reg.setInputTarget(target);
  reg.setInputSource(source);
  reg.align(*aligned);
  // get score and result.
  rough_trans_ = reg.getFinalTransformation();
  score_ = reg.getFitnessScore();
  return true;
}

bool Mapping_front::CloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
  switch (filter_way_) {
    case 1:
      // option 1: use voxel filter.
      downSizeFiltermap.setInputCloud(input);
      downSizeFiltermap.filter(*input);
      // option 2: use StatisticalOutlierRemoval.
      sor.setInputCloud(input);
      sor.filter(*output);
      break;
    case 2:
      // option 1: boxfilter.
      box_filter.setInputCloud(input);
      box_filter.filter(*input);
      // option 2: use voxid filter.
      downSizeFiltermap.setInputCloud(input);
      downSizeFiltermap.filter(*output);
    case 3:
      // option 1: boxfilter.
      box_filter.setInputCloud(input);
      box_filter.filter(*input);
      // option 2: use voxid filter.
      downSizeFiltermap.setInputCloud(input);
      downSizeFiltermap.filter(*input);
      // option 3: use StatisticalOutlierRemoval.
      sor.setInputCloud(input);
      sor.filter(*output);
      break;
    default:
      *output = *input;
      break;
  }
  return true;
}

bool Mapping_front::GetPreciseResult(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(
      new pcl::PointCloud<pcl::PointXYZ>);
  icp_.setInputSource(source);
  icp_.setInputTarget(target);
  icp_.align(*aligned, rough_trans_);
  precise_trans_ = icp_.getFinalTransformation();
  last_score_ = icp_.getFitnessScore();
  // trans_cloud;
  trans_cloud_->clear();
  trans_cloud_->swap(*aligned);
  return true;
}

bool Mapping_front::CloudMatchingFlow(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source) {
  switch (matching_way_) {
    case 1:
      // ======================VGICP-mt=======================
      CloudMatching(vgicp_mt, target, source);
      GetPreciseResult(target, source);
      break;
    case 2:
      // ====================VGICP-cuda=======================
      CloudMatching(vgicp_cuda, target, source);
      GetPreciseResult(target, source);
      break;
    default:
      // =====================GPU-NDT=========================
      CloudMatching(ndt_cuda, target, source);
      GetPreciseResult(target, source);
      break;
  }
  return true;
}

bool Mapping_front::SavetheTransformedcloud() {
  if (trans_cloud_->empty()) {
    LOG(INFO) << "trans_cloud_ is empty!";
    return false;
  }
  std::string out_path = "../test_result/result1.pcd";
  pcl::io::savePCDFileBinary(out_path, *trans_cloud_);
  return true;
}

}  // namespace myslam
