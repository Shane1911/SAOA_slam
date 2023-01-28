/*
 * @Date: 2023-01-17 18:01:48
 * @LastEditTime: 2023-01-28 16:03:00
 * @LastEditors: Shane
 * @Description: 料知此生无大事，关心雪后有梅花.
 */
#include "back_end.h"
namespace myslam {

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

Back_end::Back_end(const string yaml_path) {
  YAML::Node config_node = YAML::LoadFile(yaml_path);
  lidar_pose_path_ = config_node["lidar_pose_path"].as<string>();
  // load gps odometry.
  gps_pose_path_ = config_node["gps_pose_path"].as<string>();
  // load loop odometry.
  loop_pose_path_ = config_node["loop_pose_path"].as<string>();
  res_pose_path_ = config_node["res_pose_path"].as<string>();
  res_cov_path_ = config_node["res_cov_path"].as<string>();
  // noise.
  double lidar_t_noise = config_node["lidar_pose_noise"].as<double>();
  double lidar_r_noise = config_node["lidar_rotate_noise"].as<double>();
  double gnss_t_noise = config_node["gnss_pose_noise"].as<double>();
  double gnss_r_noise = config_node["gnss_rotate_noise"].as<double>();
  double loop_t_noise = config_node["loop_pose_noise"].as<double>();
  double loop_r_noise = config_node["loop_rotate_noise"].as<double>();
  int nums_node = config_node["num_frames"].as<int>();
  // gnss2lidar matrix.
  double fix_x = config_node["correctfix"]["fix_x"].as<double>();
  double fix_y = config_node["correctfix"]["fix_y"].as<double>();
  double fix_z = config_node["correctfix"]["fix_z"].as<double>();
  double fix_roll = config_node["correctfix"]["fix_roll"].as<double>();
  double fix_pitch = config_node["correctfix"]["fix_pitch"].as<double>();
  double fix_yaw = config_node["correctfix"]["fix_yaw"].as<double>();
  gnss2lidar_ = ToMatrix4d(fix_x, fix_y, fix_x, fix_roll, fix_pitch, fix_yaw);
  // set the gtsam noise.
  lidar_odom_noise_model_ = noiseModel::Diagonal::Variances(
      (Vector(6) << lidar_t_noise, lidar_t_noise, lidar_t_noise, lidar_r_noise,
       lidar_r_noise, lidar_r_noise)
          .finished());
  priorNoise_ = noiseModel::Diagonal::Sigmas(
      (Vector(6) << gnss_t_noise, gnss_t_noise, gnss_t_noise, gnss_r_noise,
       gnss_r_noise, gnss_r_noise)
          .finished());
  loop_odom_noise_model_ = noiseModel::Diagonal::Variances(
      (Vector(6) << loop_t_noise, loop_t_noise, loop_t_noise, loop_r_noise,
       loop_r_noise, loop_r_noise)
          .finished());
  if (!ReadLoopPose(loop_pose_path_, loop_pose_buff_))
    cout << "Cann't get the loop_pose from:" << loop_pose_path_ << endl;
  int lidar_flag = 0;
  if (!ReadPose(lidar_pose_path_, lidar_odom_data_buff_, lidar_flag))
    cout << "Cann't get the lidar_pose from:" << lidar_pose_path_ << endl;
  int gnss_flag = 1;
  if (!ReadPose(gps_pose_path_, gps_data_buff_, gnss_flag))
    cout << "Cann't get the gps_pose from:" << gps_pose_path_ << endl;
  if (nums_node != 0) {
    node_nums_ = nums_node;
  } else {
    node_nums_ = gps_data_buff_.size();
  }
  if (!GetGpspose()) LOG(INFO) << "Can't get the gpspose!!!";
  if (!GetLoopid()) LOG(INFO) << "Can't get the looppose!!!";
  cout << "slam back_end initial success!!!" << endl;
  LOG(INFO) << "slam back_end initial paramenters: \n"
            << "lidar_odom_noise: " << lidar_t_noise << ", " << lidar_r_noise
            << "\n"
            << "gnss_odom_noise: " << gnss_t_noise << ", " << gnss_r_noise
            << "\n"
            << "loop_odom_noise: " << loop_t_noise << ", " << loop_r_noise;
}
Back_end::~Back_end() {}

// TODO:
bool Back_end::ReadPose(std::string& pose_path,
                        std::vector<Pose_frames>& pose_buff, int& trans_flag) {
  std::ifstream infile;
  infile.open(pose_path.data());
  std::string s;
  while (getline(infile, s)) {
    vector<string> res;
    stringstream input(s);
    string result;
    while (input >> result) res.push_back(result);
    Eigen::Quaterniond q(stod(res[3]), stod(res[4]), stod(res[5]),
                         stod(res[6]));
    Eigen::Vector3d t1;
    t1 << stod(res[0]), stod(res[1]), stod(res[2]);
    Pose_frames current_pose;
    current_pose.q = q;
    current_pose.t = t1;
    pose_buff.push_back(current_pose);
  }
  infile.close();
  // transform the gnss_pose to lidarframe.
  if (trans_flag != 0) pose_buff = Gnsspose2lidarframe(pose_buff);
  return !(pose_buff.empty());
}

Eigen::Matrix4d Back_end::ToMatrix4d(const double& x, const double& y,
                                     const double& z, const double& roll,
                                     const double& pitch, const double& yaw) {
  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
  tf(0, 3) = x;
  tf(1, 3) = y;
  tf(2, 3) = z;
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  tf.block(0, 0, 3, 3) = rotation_matrix;
  return tf;
}
// TODO(xiec): using isometry3d to describle the odometry.
std::vector<Pose_frames> Back_end::Gnsspose2lidarframe(
    std::vector<Pose_frames>& pose_buff) {
  std::vector<Pose_frames> res_pose;
  if (pose_buff.empty()) return res_pose;
  Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity();
  T0.block(0, 0, 3, 3) = pose_buff[0].q.toRotationMatrix();
  T0.block(0, 3, 3, 1) = pose_buff[0].t;
  for (int k = 0; k < pose_buff.size(); k++) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0, 0, 3, 3) = pose_buff[k].q.toRotationMatrix();
    T.block(0, 3, 3, 1) = pose_buff[k].t;
    auto delta_T = gnss2lidar_.inverse() * (T0.inverse() * T) * gnss2lidar_;
    Pose_frames pose_trans;
    Eigen::Matrix3d R = delta_T.block(0, 0, 3, 3);
    pose_trans.q = Eigen::Quaterniond(R);
    pose_trans.t = delta_T.block(0, 3, 3, 1);
    res_pose.push_back(pose_trans);
  }
  return res_pose;
}

bool Back_end::ReadLoopPose(std::string& pose_path,
                            std::vector<Loop_frame>& loop_pose_buff) {
  std::ifstream infile;
  infile.open(pose_path.data());
  std::string s;
  while (getline(infile, s)) {
    vector<string> res;
    stringstream input(s);
    string result;
    while (input >> result) res.push_back(result);
    Eigen::Quaterniond q(stod(res[5]), stod(res[6]), stod(res[7]),
                         stod(res[8]));
    Eigen::Vector3d t1;
    t1 << stod(res[2]), stod(res[3]), stod(res[4]);
    Loop_frame current_loop_pose;
    current_loop_pose.id_1 = stoi(res[0]);
    current_loop_pose.id_2 = stoi(res[1]);
    current_loop_pose.q = q;
    current_loop_pose.t = t1;
    loop_pose_buff.push_back(current_loop_pose);
  }
  infile.close();
  return !(loop_pose_buff.empty());
}

bool Back_end::SaveResults(std::string& pose_path, Values& results) {
  std::ofstream ofs;
  ofs.open(pose_path.c_str(), std::ios::out);
  for (int i = 0; i < results.size(); i++) {
    auto pose_key = X(i + 1);
    Pose3 pose = results.at<Pose3>(pose_key);
    auto rot_q = pose.rotation().toQuaternion();
    ofs << rot_q.x() << " " << rot_q.y() << " " << rot_q.z() << " " << rot_q.w()
        << " " << pose.x() << " " << pose.y() << " " << pose.z() << endl;
  }
  ofs.close();
  return true;
}

bool Back_end::SaveCov(std::string& pose_path,
                       std::vector<Eigen::MatrixX2d>& cov_res) {
  std::ofstream ofs;
  ofs.open(pose_path.c_str(), std::ios::out);
  for (int i = 0; i < cov_res.size(); i++) {
    auto pcov_xy = cov_res[i];
    ofs << pcov_xy(0, 0) << " " << pcov_xy(0, 1) << " " << pcov_xy(1, 0) << " "
        << pcov_xy(1, 1) << endl;
  }
  ofs.close();
  return true;
}

bool Back_end::GetGpspose() {
  for (auto gps_frame_ : gps_data_buff_) {
    gpsPose_.push_back(Pose3(Rot3(gps_frame_.q), gps_frame_.t));
  }
  return !(gpsPose_.empty());
}

bool Back_end::GetLoopid() {
  for (int i = 0; i < loop_pose_buff_.size(); i++) {
    auto current_loop_frame_ = loop_pose_buff_[i];
    loop_id_.push_back(current_loop_frame_.id_2);
  }
  return !(loop_id_.empty());
}

bool Back_end::Optimizition() {
  // initial gtsam.
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);
  // Create a Factor Graph and Values to hold the new data.
  NonlinearFactorGraph graph_;
  Values initialEstimate;
  Values currentEstimate;
  Pose_frames current_frame_ = lidar_odom_data_buff_[0];
  auto current_pose_global = Pose3(Rot3(current_frame_.q), current_frame_.t);
  int s = 0;
  bool opimize = false;
  Pose3 pre_pose3_;
  for (int j = 0; j < node_nums_; j++) {
    auto current_pose_key = X(j + 1);
    auto current_frame_ = lidar_odom_data_buff_[j];
    auto current_pose = Pose3(Rot3(current_frame_.q), current_frame_.t);
    if (j == 0) {
      initialEstimate.insert(current_pose_key, current_pose_global);
      graph_.push_back(
          PriorFactor<Pose3>(current_pose_key, gpsPose_[j], priorNoise_));
    } else {
      if (j % 10 == 0) {
        graph_.push_back(
            PriorFactor<Pose3>(current_pose_key, gpsPose_[j], priorNoise_));
        initialEstimate.insert(current_pose_key, gpsPose_[j]);
        opimize = true;
      } else {
        initialEstimate.insert(current_pose_key, current_pose_global);
      }
      // add edge
      auto previous_pose_key = X(j);
      graph_.push_back(BetweenFactor<Pose3>(previous_pose_key, current_pose_key,
                                            pre_pose3_.between(current_pose),
                                            lidar_odom_noise_model_));
      if (!loop_id_.empty() && j == loop_id_[s] + 2) {
        auto current_loop_frame_ = loop_pose_buff_[s];
        auto current_loop_pose =
            Pose3(Rot3(current_loop_frame_.q), current_loop_frame_.t);
        auto loop_prepose_key = X(current_loop_frame_.id_1);
        auto loop_pose_key = X(current_loop_frame_.id_2);
        graph_.push_back(BetweenFactor<Pose3>(loop_prepose_key, loop_pose_key,
                                              current_loop_pose,
                                              loop_odom_noise_model_));
        s++;
        opimize = true;
      }
      if (j > 0 && opimize) {
        isam.update(graph_, initialEstimate);
        isam.update();
        currentEstimate = isam.calculateEstimate();
        graph_.resize(0);
        initialEstimate.clear();
        current_pose_global = currentEstimate.at<Pose3>(current_pose_key);
        opimize = false;
      }
    }
    pre_pose3_ = current_pose;
  }
  isam.update(graph_, initialEstimate);
  isam.update();
  currentEstimate = isam.calculateEstimate();
  graph_.resize(0);
  initialEstimate.clear();
  for (int k = 0; k < node_nums_; k++) {
    auto pose_key = X(k + 1);
    Eigen::MatrixX2d cov = isam.marginalCovariance(pose_key).block(3, 3, 2, 2);
    cov_xy_.push_back(cov);
  }
  // save results.
  SaveResults(res_pose_path_, currentEstimate);
  SaveCov(res_cov_path_, cov_xy_);
  std::cout << "The res_pose save to: " << res_pose_path_
            << "\n The res_cov save to: " << res_cov_path_ << std::endl;
  LOG(INFO) << "The res_pose save to: " << res_pose_path_
            << "\n The res_cov save to: " << res_cov_path_;
  return true;
}

}  // namespace myslam
