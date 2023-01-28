/**
 * @Date: 2023-01-16 20:37:11
 * @LastEditTime: 2023-01-16 20:37:11
 * @LastEditors: Shane
 * @Description: 料知此生无大事，关心雪后有梅花.
 */
#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fstream>
#include <iostream>
#include <vector>

#include "include/yaml-cpp/yaml.h"
namespace myslam {
using namespace std;
using namespace Eigen;

struct Loop_frames {
  int id_1 = 0;
  int id_2 = 0;
  Matrix4f relative_pose = Matrix4f::Identity();
};

class Loop_closing {
 public:
  // Initial the loopclose finder.
  Loop_closing(const string yaml_path);
  ~Loop_closing();
  bool GetLoopclosing();

 private:
  bool ReadGpsPose(std::string& pose_path,
                   std::vector<Eigen::Matrix4d>& pose_buff);
  bool FindNearstPose(Eigen::Vector2d& current_pose, int& id);
  bool SaveloopPose(const std::string& filename,
                    vector<Loop_frames>& loop_pose);
  bool GetLoopclosingResults(const int& history_id, const int& cur_id,
                             double* score, Eigen::Matrix4f* reltive_pose);
  vector<Eigen::Vector2d> history_pose_;
  string gps_path_ = "";
  string pcd_path_ = "";
  string loopclosing_path_ = "";
  std::vector<Eigen::Matrix4d> gps_pose_;
  fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_ptr_;
};

}  // namespace myslam
