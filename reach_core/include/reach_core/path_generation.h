/*
 * Copyright 2022 PickNik, Inc.
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
/* Authors: Lovro Ivanov, @livanov93
   Desc:
*/
#ifndef REACH_CORE_PATH_GENERATION_H
#define REACH_CORE_PATH_GENERATION_H

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "reach_core/plugins/path_base.h"

#include <rclcpp/rclcpp.hpp>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_loader.hpp>
#include <reach_core/ik_helper.h>
#include <reach_core/plugins/ik_solver_base.h>
#include <reach_core/reach_visualizer.h>
#include <reach_core/study_parameters.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>

constexpr char PACKAGE[] = "reach_core";
constexpr char PATH_BASE_CLASS[] = "reach::plugins::PathBase";
constexpr char IK_BASE_CLASS[] = "reach::plugins::IKSolverBase";
constexpr char DISPLAY_BASE_CLASS[] = "reach::plugins::DisplayBase";

namespace reach {
namespace core {

/// \brief Holds parameters for the \p PathGeneration
class Configuration {
 public:
  Configuration()
      : path_loader(PACKAGE, PATH_BASE_CLASS),
        solver_loader(PACKAGE, IK_BASE_CLASS),
        display_loader(PACKAGE, DISPLAY_BASE_CLASS) {}

  std::string results_package;
  std::string results_directory;
  std::string results_db_name;

  // external interfaces
  ReachDatabasePtr db;
  ReachVisualizerPtr visualizer;

  // plugin loaders
  pluginlib::ClassLoader<reach::plugins::PathBase> path_loader;
  pluginlib::ClassLoader<reach::plugins::IKSolverBase> solver_loader;
  pluginlib::ClassLoader<reach::plugins::DisplayBase> display_loader;

  // plugins
  std::vector<reach::plugins::PathBasePtr> path_generators;
  reach::plugins::IKSolverBasePtr ik_solver;
  reach::plugins::DisplayBasePtr display;
  // robot model
  std::shared_ptr<moveit::core::RobotModel const> robot_model;
  /// \brief Utility for making a common \p PathGeneration configuration
  /// \param node Used to initialize solvers
  /// \param prefix
  /// \returns Typical configuration
  Configuration Make(std::shared_ptr<rclcpp::Node> node,
                     const std::string& prefix);
};

/**
 * @brief The ReachStudy class
 */
class PathGeneration : public rclcpp::Node {
 public:
  /**
   * @brief PathGeneration
   * @param name
   */
  PathGeneration(const std::string& name);

  ~PathGeneration();

 private:
  bool initialize();

  bool visualizeDatabases();

  bool getObjectPointCloud();

 protected:
  std::string getFullPathToDb(const std::string& db_pkg,
                              const std::string& db_dir,
                              const std::string& db_config,
                              const std::string& db_name);

  Configuration configuration_;
};

}  // namespace core
}  // namespace reach

#endif  // REACH_CORE_REACH_STUDY_H
