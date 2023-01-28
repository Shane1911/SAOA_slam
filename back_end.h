/**
 * @Date: 2023-01-20 23:10:50
 * @LastEditTime: 2023-01-20 23:14:25
 * @LastEditors: Shane
 * @Description: 料知此生无大事，关心雪后有梅花.
 */
#pragma once
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "include/yaml-cpp/yaml.h"
namespace myslam {
using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

struct Pose_frames {
  Eigen::Quaterniond q;
  Vector3 t;
};

struct Loop_frame {
  int id_1;
  int id_2;
  Eigen::Quaterniond q;
  Vector3 t;
};

class Back_end {
 public:
  Back_end(const string yaml_path);
  ~Back_end();
  bool Optimizition();

 private:
  bool ReadPose(std::string& pose_path, std::vector<Pose_frames>& pose_buff,
                int& trans_flag);
  std::vector<Pose_frames> Gnsspose2lidarframe(
      std::vector<Pose_frames>& pose_buff);
  bool SaveResults(std::string& pose_path, Values& results);

  bool ReadLoopPose(std::string& pose_path,
                    std::vector<Loop_frame>& loop_pose_buff);

  bool SaveCov(std::string& pose_path, std::vector<Eigen::MatrixX2d>& cov_res);
  bool GetGpspose();
  bool GetLoopid();
  Eigen::Matrix4d ToMatrix4d(const double& x, const double& y, const double& z,
                             const double& roll, const double& pitch,
                             const double& yaw);
  // Data.
  std::vector<Pose_frames> lidar_odom_data_buff_;
  std::vector<Pose_frames> gps_data_buff_;
  std::vector<Loop_frame> loop_pose_buff_;
  std::vector<Eigen::MatrixX2d> cov_xy_;
  std::vector<Pose3> gpsPose_;
  std::vector<int> loop_id_;
  int node_nums_;
  noiseModel::Diagonal::shared_ptr lidar_odom_noise_model_;
  noiseModel::Diagonal::shared_ptr priorNoise_;
  noiseModel::Diagonal::shared_ptr loop_odom_noise_model_;
  // input data.
  string lidar_pose_path_ = "";
  // load gps odometry
  string gps_pose_path_ = "";
  // load loop odometry
  string loop_pose_path_ = "";
  // results path
  string res_pose_path_ = "";
  string res_cov_path_ = "";
  // gnss2lidar matrix
  Eigen::Matrix4d gnss2lidar_;
};

}  // namespace myslam
