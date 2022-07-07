/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef REACH_CORE_REACH_STUDY_H
#define REACH_CORE_REACH_STUDY_H

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <rclcpp/rclcpp.hpp>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_loader.hpp>
#include <reach_core/ik_helper.h>
#include <reach_core/plugins/ik_solver_base.h>
#include <reach_core/plugins/path_base.h>
#include <reach_core/reach_visualizer.h>
#include <reach_core/study_parameters.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>

bool transcribeInputMap(const std::map<std::string, double>& input,
                        const std::vector<std::string>& joint_names,
                        std::vector<double>& input_subset) {
  //    for(const auto & i : input){
  //        printf("%s...%f\n", i.first.c_str(), i.second);
  //    }
  //    for(const auto & i : joint_names){
  //        printf("%s\n", i.c_str());
  //    }
  if (joint_names.size() > input.size()) {
    return false;
  }

  // Pull the joints of the planning group out of the input map
  std::vector<double> tmp;
  tmp.reserve(joint_names.size());
  for (const std::string& name : joint_names) {
    const auto it = input.find(name);
    if (it == input.end()) {
      return false;
    } else {
      tmp.push_back(it->second);
    }
  }

  input_subset = std::move(tmp);

  return true;
}

namespace reach {
namespace core {

typedef std::map<std::string, std::map<std::string, double>>
    RobotConfigurations;
/**
 * @brief The ReachStudy class
 */
class ReachStudy {
 public:
  /**
   * @brief ReachStudy
   * @param nh
   */
  ReachStudy(const rclcpp::Node::SharedPtr node);

  ~ReachStudy();

  /**
   * @brief run
   * @param sp
   * @return
   */
  bool run(const StudyParameters& sp);

  std::shared_ptr<rclcpp::Node> get_node() {
    if (!node_.get()) {
      throw std::runtime_error("Node hasn't been initialized yet!");
    }
    return node_;
  }

 private:
  bool initializeStudy(const StudyParameters& sp);

  bool initializePathGeneration(const StudyParameters& sp);

  void generatePaths();

  bool getReachObjectPointCloud();

  void runInitialReachStudy();

  void optimizeReachStudyResults();

  void getAverageNeighborsCount();

  bool compareDatabases();

  bool visualizeDatabases();

  StudyParameters sp_;

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_;

  ReachDatabasePtr db_;

  // Plugins
  pluginlib::ClassLoader<reach::plugins::IKSolverBase> solver_loader_;
  pluginlib::ClassLoader<reach::plugins::DisplayBase> display_loader_;
  pluginlib::ClassLoader<reach::plugins::PathBase> path_generator_loader_;
  reach::plugins::IKSolverBasePtr ik_solver_;
  reach::plugins::DisplayBasePtr display_;
  std::vector<reach::plugins::PathBasePtr> path_generators_;
  ReachDatabasePtr paths_db_;
  RobotConfigurations robot_configurations_;
  std::string paths_path_;

  ReachVisualizerPtr visualizer_;

  SearchTreePtr search_tree_;

  std::string dir_;

  std::string results_dir_;

  sensor_msgs::msg::PointCloud2 cloud_msg_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_pub_;

  // robot model
  moveit::core::RobotModelConstPtr model_;
};

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_REACH_STUDY_H
