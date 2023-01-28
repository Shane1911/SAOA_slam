/*
 * @Date: 2023-01-16 20:38:14
 * @LastEditTime: 2023-01-18 13:58:43
 * @LastEditors: Shane
 * @Description: 料知此生无大事，关心雪后有梅花.
 */
#include "loop_closing.h"
namespace myslam {

Loop_closing::Loop_closing(const string yaml_path) {
  YAML::Node config_node = YAML::LoadFile(yaml_path);
  gps_path_ = config_node["gps_pose_path"].as<string>();
  pcd_path_ = config_node["pcd_pose_path"].as<string>();
  loopclosing_path_ = config_node["loopclosing_path"].as<string>();
  ReadGpsPose(gps_path_, gps_pose_);
  // initial vgicp
  //   vgicp_ptr_ =
  //       std::make_shared<fast_gicp::NDTCuda<pcl::PointXYZ, pcl::PointXYZ>>();
  vgicp_ptr_.setResolution(1.0);
  //   vgicp_ptr_->setNumThreads(12);
  vgicp_ptr_.setMaximumIterations(50);
}
Loop_closing::~Loop_closing() {}

bool Loop_closing::ReadGpsPose(std::string& pose_path,
                               std::vector<Eigen::Matrix4d>& pose_buff) {
  std::ifstream infile;
  infile.open(pose_path.data());
  std::string s;
  while (getline(infile, s)) {
    vector<string> res;
    stringstream input(s);
    string result;
    while (input >> result) res.push_back(result);
    // cout << result << endl;
    Eigen::Quaterniond q(stod(res[3]), stod(res[4]), stod(res[5]),
                         stod(res[6]));
    Eigen::Matrix4d current_pose;
    current_pose.block(0, 0, 3, 3) = q.toRotationMatrix();
    current_pose(0, 3) = stod(res[0]);
    current_pose(1, 3) = stod(res[1]);
    current_pose(2, 3) = stod(res[2]);
    pose_buff.push_back(current_pose);
  }
  infile.close();
  return true;
}

bool Loop_closing::FindNearstPose(Eigen::Vector2d& current_pose, int& id) {
  double mindistance = 1e8;
  auto nearest_itr = history_pose_.end();
  int count = 0;
  if (history_pose_.size() < 200) return false;
  for (auto itr = history_pose_.begin(); itr != history_pose_.end(); ++itr) {
    auto cur_pose = *itr;
    double dx = current_pose(0, 0) - cur_pose(0, 0);
    double dy = current_pose(1, 0) - cur_pose(1, 0);
    double diff = sqrt(pow(dx, 2) + pow(dy, 2));
    if (diff < mindistance) {
      mindistance = diff;
      nearest_itr = itr;
      id = count + 1;
    }
    count++;
    if (count >= history_pose_.size() - 100) break;
  }
  if (mindistance > 2.0 || mindistance < 1e-6) return false;
  return true;
}

bool Loop_closing::SaveloopPose(const std::string& filename,
                                vector<Loop_frames>& loop_pose) {
  std::ofstream ofs(filename.c_str());
  if (ofs.fail()) {
    std::cout << "Fail to open file " << filename;
    return false;
  }
  ofs.precision(6);
  // skip the first point and the last point
  for (size_t i = 0; i < loop_pose.size(); i++) {
    Eigen::Quaternionf q(loop_pose[i].relative_pose.block<3, 3>(0, 0));
    Eigen::Vector3f t = loop_pose[i].relative_pose.block<3, 1>(0, 3);
    Eigen::Quaternionf Quater = q.normalized();
    Eigen::Vector3f Vector = t;
    ofs << std::fixed << loop_pose[i].id_1 << " " << loop_pose[i].id_2 << " "
        << Vector.x() << " " << Vector.y() << " " << Vector.z() << " "
        << Quater.w() << " " << Quater.x() << " " << Quater.y() << " "
        << Quater.z() << "\n";
  }
  ofs.close();
  std::cout << "key_gnss_pose result saved to: " << filename << std::endl;
  return true;
}

bool Loop_closing::GetLoopclosing() {
  int pre_loop_id = 0;
  bool pre_find_loop = false;
  int cur_id = 0;
  vector<Loop_frames> loop_pose;
  for (auto& cur_pose : gps_pose_) {
    Eigen::Vector2d points;
    points << cur_pose(0, 3), cur_pose(1, 3);
    history_pose_.push_back(points);
    cur_id++;
    int history_id = 0;
    if (pre_find_loop) {
      if (cur_id < pre_loop_id + 20) {
        continue;
      } else {
        pre_find_loop = false;
        continue;
      }
    }
    if (FindNearstPose(points, history_id)) {
      pre_loop_id = cur_id;
      pre_find_loop = true;
      // history_id,cur_id,relative_pose
      double score = 1e3;
      Eigen::Matrix4f reltive_pose;
      GetLoopclosingResults(history_id, cur_id, &score, &reltive_pose);
      if (score > 0.5) {
        continue;
      }
      cout << " score: " << score << endl;
      Loop_frames curre_loop;
      curre_loop.id_1 = history_id;
      curre_loop.id_2 = cur_id;
      curre_loop.relative_pose = reltive_pose;
      cout << "loop pose: " << history_id << ",  " << cur_id << endl;
      loop_pose.push_back(curre_loop);
    }
  }
  // save loop_pose to file.
  SaveloopPose(loopclosing_path_, loop_pose);
  return true;
}

bool Loop_closing::GetLoopclosingResults(const int& history_id,
                                         const int& cur_id, double* score,
                                         Eigen::Matrix4f* reltive_pose) {
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  string filename1 = pcd_path_ + to_string(history_id) + ".pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_current(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(filename1, *target_current);
  string filename2 = pcd_path_ + to_string(cur_id) + ".pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_current(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(filename2, *source_current);
  vgicp_ptr_.clearTarget();
  vgicp_ptr_.clearSource();
  vgicp_ptr_.setInputSource(source_current);
  vgicp_ptr_.setInputTarget(target_current);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  vgicp_ptr_.align(*output_cloud, init_guess);
  *reltive_pose = vgicp_ptr_.getFinalTransformation();
  *score = vgicp_ptr_.getFitnessScore();
  return true;
}

}  // namespace myslam
