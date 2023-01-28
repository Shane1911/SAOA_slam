/*
 * @Date: 2023-01-06 15:37:22
 * @LastEditTime: 2023-01-28 16:01:22
 * @LastEditors: Shane
 * @Description: 料知此生无大事，关心雪后有梅花.
 */
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include "back_end.h"
#include "loop_closing.h"
#include "mapping_front.h"
using namespace std;
using namespace Eigen;
DEFINE_string(yaml_path, "../config/front_end.yaml",
              "The paraments form yaml.");
DEFINE_string(loop_yaml_path, "../config/loop_find.yaml",
              "The loop paraments form yaml.");
DEFINE_string(back_yaml_path, "../config/back_end.yaml",
              "The back_end paraments form yaml.");
DEFINE_string(front_result_path, "../result/front_result/front_result.txt",
              "The results of front end");
DEFINE_string(front_map_path, "../result/front_result/front_full_map.pcd",
              "The map of front end");
DEFINE_string(optimize_result_path,
              "../result/back_result/opimization_pose.txt",
              "The results of pose");
DEFINE_string(back_map_path, "../result/back_result/final_full_map.pcd",
              "The map of back end");

static pcl::CropBox<pcl::PointXYZ> box_filter;
static pcl::VoxelGrid<pcl::PointXYZ> downSizeFiltermap;
static pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
static string str;

void Export(std::string& filename, std::vector<Eigen::Vector3f>& position,
            std::vector<Eigen::Vector4f>& quat) {
  std::ofstream ofs(filename.c_str());
  if (ofs.fail()) {
    std::cout << "Fail to open file " << filename;
    return;
  }
  int id = 0;
  for (auto Vector : position) {
    auto quat_q = quat[id];
    ofs.precision(12);
    // qw,qx,qy,qz,x,y,z
    ofs << quat_q[0] << " " << quat_q[1] << " " << quat_q[2] << " " << quat_q[3]
        << " " << Vector[0] << " " << Vector[1] << " " << Vector[2] << "\n";
    id++;
  }
  ofs.close();
  std::cout << "Front-end results saved to: " << filename << std::endl;
}
bool ReadPose(std::string& pose_path, std::vector<Eigen::Matrix4f>& pose_buff) {
  std::ifstream infile;
  infile.open(pose_path.data());
  std::string s;
  while (getline(infile, s)) {
    vector<string> res;
    stringstream input(s);
    string result;
    while (input >> result) res.push_back(result);
    Eigen::Quaternionf q(stof(res[3]), stof(res[0]), stof(res[1]),
                         stof(res[2]));
    Eigen::Matrix4f current_pose;
    current_pose.block(0, 0, 3, 3) = q.toRotationMatrix();
    current_pose(0, 3) = stof(res[4]);
    current_pose(1, 3) = stof(res[5]);
    current_pose(2, 3) = stof(res[6]);
    pose_buff.push_back(current_pose);
  }
  infile.close();
  return true;
}

bool CloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
  box_filter.setInputCloud(input);
  box_filter.filter(*input);
  //  option 2: use voxid
  downSizeFiltermap.setInputCloud(input);
  downSizeFiltermap.filter(*input);
  //  option 4: use StatisticalOutlierRemoval
  sor.setInputCloud(input);
  sor.filter(*output);
  return true;
}

bool Mapping(std::string& pose_path, std::vector<Eigen::Matrix4f>& results) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_sum(
      new pcl::PointCloud<pcl::PointXYZ>);
  cout << results.size() << endl;
  // Set the filter.
  box_filter.setMin(Eigen::Vector4f(5, -70, -4, 1e-6));
  box_filter.setMax(Eigen::Vector4f(70, 70, 20, 1e6));
  downSizeFiltermap.setLeafSize(0.1, 0.1, 0.1);
  sor.setMeanK(30);
  sor.setStddevMulThresh(2.0);
  for (int i = 0; i < results.size() - 1; i = i + 2) {
    string filename1 = str + to_string(i + 1) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(filename1, *cloud_current);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    CloudProcess(cloud_current, transformed_cloud);
    pcl::transformPointCloud(*cloud_current, *transformed_cloud, results[i]);
    *output_cloud_sum += *transformed_cloud;
  }
  pcl::VoxelGrid<pcl::PointXYZ> summapFiltermap;
  float mapRes = 0.6;
  summapFiltermap.setLeafSize(mapRes, mapRes, mapRes);
  summapFiltermap.setInputCloud(output_cloud_sum);
  summapFiltermap.filter(*output_cloud_sum);
  pcl::io::savePCDFileBinary(pose_path, *output_cloud_sum);
  return true;
}

int main(int argc, char** argv) {
  FLAGS_log_dir = "../result/log";
  google::InitGoogleLogging(argv[0]);
  LOG(INFO) << "test begaining!!!";
  // Get yaml paramenters.
  string paramenters_path = FLAGS_yaml_path;
  YAML::Node config_node = YAML::LoadFile(paramenters_path);
  str = config_node["pcd_path"].as<string>();
  int skip_frame = config_node["skip_frame"].as<int>();
  int frames_num = config_node["frames_num"].as<int>();
  int rate_frame = config_node["rate_frames"].as<int>();
  float map_filter = config_node["map_filter"].as<float>();
  int start_fram = config_node["start_fram"].as<int>();
  bool viewer_mapping_flag = config_node["viewer_mapping"].as<bool>();
  int sleep_count = config_node["sleep_time"].as<int>();
  double sleep_time = 1e5 * (static_cast<double>(sleep_count));
  // ======================test-front=============================
  myslam::Mapping_front slam_front_;
  slam_front_.Initial(paramenters_path);
  //========================data-flow======================
  std::string filename0 = str + to_string(start_fram) + ".pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pre(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(filename0, *cloud_pre);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pre_filter(
      new pcl::PointCloud<pcl::PointXYZ>);
  slam_front_.CloudProcess(cloud_pre, cloud_pre_filter);
  //================initial pose.(wait to modify)
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  std::vector<Eigen::Vector3f> out_pose_result;
  std::vector<Eigen::Vector4f> out_q_result;
  Eigen::Matrix4f pre_pose = init_guess;
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_sum(
      new pcl::PointCloud<pcl::PointXYZ>);
  *output_cloud_sum += *cloud_pre_filter;
  // get initial value.
  Eigen::Matrix3f R_q = pre_pose.block(0, 0, 3, 3);
  Eigen::Quaternionf tmp_q(R_q);
  out_pose_result.push_back({pre_pose(0, 3), pre_pose(1, 3), pre_pose(2, 3)});
  out_q_result.push_back({tmp_q.w(), tmp_q.x(), tmp_q.y(), tmp_q.z()});
  int count = 0;
  // viewer the mapping.
  // pcl::visualization::PCLVisualizer viewer("The map of front end");
  for (int k = skip_frame + start_fram; k < frames_num; k += skip_frame) {
    // get current lidar frame.
    std::string filenamek = str + to_string(k) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(filenamek, *cloud_cur);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cur_filter(
        new pcl::PointCloud<pcl::PointXYZ>);
    slam_front_.CloudProcess(cloud_cur, cloud_cur_filter);
    // matching.......
    slam_front_.CloudMatchingFlow(cloud_pre_filter, cloud_cur_filter);
    Matrix4f current_trans = slam_front_.PreciseReslut();
    double score = slam_front_.GetMatchinglfinalscore();
    cloud_pre_filter.swap(cloud_cur_filter);
    if (score > 2.0) std::cout << " score: " << score << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    // result process.
    Matrix3f delta_R = current_trans.block(0, 0, 3, 3);
    Vector3f euler_angles = delta_R.eulerAngles(2, 1, 0);
    if (abs(euler_angles[1]) < 3.7e-5 && abs(euler_angles[2]) < 3.7e-5 &&
        abs(current_trans(2, 3)) < 8e-4) {
      current_trans(2, 3) = 0.0;
      AngleAxisf rotation_vector(euler_angles[0], Vector3f(0, 0, 1));
      current_trans.block(0, 0, 3, 3) = rotation_vector.toRotationMatrix();
    }
    if (abs(current_trans(0, 3)) < 0.03 && abs(current_trans(1, 3)) < 0.03) {
      current_trans = Matrix4f::Identity();
    }
    pre_pose = pre_pose * current_trans;
    // save path to *.txt
    Matrix3f R_q = pre_pose.block(0, 0, 3, 3);
    Quaternionf tmp_q(R_q);
    out_pose_result.push_back({pre_pose(0, 3), pre_pose(1, 3), pre_pose(2, 3)});
    out_q_result.push_back({tmp_q.w(), tmp_q.x(), tmp_q.y(), tmp_q.z()});
    // transform the submap.
    pcl::transformPointCloud(*cloud_cur, *transformed_cloud, pre_pose);
    *output_cloud_sum += *transformed_cloud;
    if ((++count) % (skip_frame * rate_frame) == 0)
      cout << "Front complish===================================>["
           << 100 * k / frames_num << "]" << endl;
    // Viewer the front mapping.
    // if (viewer_mapping_flag) {
    //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    //       target_handler(output_cloud_sum, 0.0, 255.0, 0.0);
    //   viewer.addPointCloud(output_cloud_sum, target_handler,
    //   std::to_string(k)); viewer.spin(); usleep(sleep_time);
    //   viewer.removePointCloud(std::to_string(k) + "car");
    // }
  }
  // write to txt.
  std::string path_file = FLAGS_front_result_path;
  Export(path_file, out_pose_result, out_q_result);
  // save map.
  pcl::VoxelGrid<pcl::PointXYZ> summapFiltermap;
  summapFiltermap.setLeafSize(map_filter, map_filter, map_filter);
  string out_path = FLAGS_front_map_path;
  summapFiltermap.setInputCloud(output_cloud_sum);
  summapFiltermap.filter(*output_cloud_sum);
  pcl::io::savePCDFileBinary(out_path, *output_cloud_sum);
  //=======================test-loopclosing finding=================
  std::string loop_pose_path = FLAGS_loop_yaml_path;
  myslam::Loop_closing slam_loop_(loop_pose_path);
  slam_loop_.GetLoopclosing();
  //=======================test-gtsam=========================
  std::string back_pose_path = FLAGS_back_yaml_path;
  myslam::Back_end slam_backend_(back_pose_path);
  slam_backend_.Optimizition();
  // ======================test-mapping=======================
  vector<Eigen::Matrix4f> result_pose;
  std::string optimize_pose_path = FLAGS_optimize_result_path;
  if (!ReadPose(optimize_pose_path, result_pose)) {
    LOG(INFO) << "Can't get pose from this path: " << optimize_pose_path;
  }
  cout << result_pose.size() << endl;
  std::string mapping_path = FLAGS_back_map_path;
  if (Mapping(mapping_path, result_pose)) {
    LOG(INFO) << "Building the cloud map to the path: " << mapping_path;
  }
  google::ShutdownGoogleLogging();
  return 0;
}