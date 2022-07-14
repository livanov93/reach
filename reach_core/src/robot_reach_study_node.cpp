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

#include "reach_core/reach_study.h"
#include "reach_core/study_parameters.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

// yaml stuff
#include <yaml-cpp/yaml.h>

class RobotReachStudyNode : public rclcpp::Node {
 public:
  explicit RobotReachStudyNode(const std::string& node_name)
      : Node(node_name,
             rclcpp::NodeOptions()
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)) {}

 public:
  bool getStudyParameters(reach::core::StudyParameters& sp) {
    // fetch parameteres
    if (!this->get_parameter("tool_frame", sp_.tool_frame) ||
        !this->get_parameter("config_name", sp_.config_name) ||
        !this->get_parameter("fixed_frame", sp_.fixed_frame) ||
        !this->get_parameter("results_package", sp_.results_package) ||
        !this->get_parameter("results_directory", sp_.results_directory) ||
        !this->get_parameter("object_frame", sp_.object_frame) ||
        !this->get_parameter("pcd_package", sp_.pcd_package) ||
        !this->get_parameter("pcd_filename_path", sp_.pcd_filename_path) ||
        !this->get_parameter("planning_group", sp_.planning_group) ||
        !this->get_parameter("run_initial_study_only",
                             sp_.run_initial_study_only) ||
        !this->get_parameter("optimization.radius", sp_.optimization.radius) ||
        !this->get_parameter("optimization.max_steps",
                             sp_.optimization.max_steps) ||
        !this->get_parameter("optimization.step_improvement_threshold",
                             sp_.optimization.step_improvement_threshold) ||
        !this->get_parameter("get_avg_neighbor_count", sp_.get_neighbors) ||
        !this->get_parameter("compare_dbs", sp_.compare_dbs) ||
        !this->get_parameter("visualize_results", sp_.visualize_results) ||
        !this->get_parameter("ik_solver_config.name",
                             sp_.ik_solver_config_name) ||
        !this->get_parameter("display_config.name", sp_.display_config_name)) {
      RCLCPP_ERROR(rclcpp::get_logger("robot_reach_study_node"),
                   "One of the main parameters do not exist...");
      return false;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "config_name: '%s'", sp_.config_name.c_str());
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "fixed_frame: '%s'", sp_.fixed_frame.c_str());
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "results_package: '%s'", sp_.results_package.c_str());
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "results_directory: '%s'", sp_.results_directory.c_str());
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "object_frame: '%s'", sp_.object_frame.c_str());
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "pcd_package: '%s'", sp_.pcd_package.c_str());
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "pcd_filename: '%s'", sp_.pcd_filename_path.c_str());
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "optimization.radius: '%f'", sp_.optimization.radius);
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "optimization.max_steps: '%d'", sp_.optimization.max_steps);
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "optimization.step_improvement_threshold: '%f'",
                  sp_.optimization.step_improvement_threshold);
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "get_avg_neighbor_count: '%d'", sp_.get_neighbors);
      for (auto const& compare_db : sp_.compare_dbs) {
        RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                    "compare_dbs: '%s'", compare_db.c_str());
      }
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "visualize_results: '%c'", sp_.visualize_results);
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "ik_solver_config.name: '%s'",
                  sp_.ik_solver_config_name.c_str());
      RCLCPP_INFO(rclcpp::get_logger("robot_reach_study_node"),
                  "display_config.name: '%s'", sp_.display_config_name.c_str());

      if (std::find(sp_.compare_dbs.begin(), sp_.compare_dbs.end(), "") !=
          sp_.compare_dbs.end()) {
        sp_.compare_dbs.clear();
      }

      // additional parameters that don't have to be set
      this->get_parameter_or("initial_seed_state", sp_.initial_seed_state, {});
      this->get_parameter_or("keep_running", sp_.keep_running, false);
      this->get_parameter_or("visualize_dbs", sp_.visualize_dbs, {});
      this->get_parameter_or("invert_z_tool_rotation",
                             sp_.invert_z_tool_rotation, true);

      // to do path generation or not
      this->get_parameter_or("generate_paths", sp_.generate_paths, false);
      this->get_parameter_or("generate_paths_only", sp_.generate_paths_only,
                             false);
      this->get_parameter_or("path_generation_config.path_plugins", sp_.paths,
                             {});
      for (const auto& p : sp_.paths) {
        std::string tmp_plugin_name;
        std::string tmp_param_name =
            "path_generation_config." + p + ".plugin_name";
        this->get_parameter_or(tmp_param_name, tmp_plugin_name,
                               std::string(""));
        sp_.path_based_plugins.push_back(tmp_plugin_name);
      }

      this->get_parameter_or("path_generation_config.initial_source_db",
                             sp_.initial_source_db, false);
      this->get_parameter_or(
          "path_generation_config.initial_source_robot_configurations",
          sp_.initial_source_robot_configurations, false);

      if (sp_.generate_paths && !sp_.initial_source_db &&
          !sp_.initial_source_robot_configurations) {
        RCLCPP_ERROR(rclcpp::get_logger("robot_reach_study_node"),
                     "No source defined for path generation!");
        return false;
      }

      if (!this->get_parameter("path_generation_config.db.config_name",
                               sp_.db_config_name) ||
          !this->get_parameter("path_generation_config.db.package",
                               sp_.db_package) ||
          !this->get_parameter("path_generation_config.db.directory",
                               sp_.db_dir) ||
          !this->get_parameter("path_generation_config.db.name", sp_.db_name)) {
        return false;
      }

      this->get_parameter_or("path_generation_config.db_goals_are_start",
                             sp_.db_goals_are_start, true);

      if (!sp_.db_goals_are_start) {
        if (!this->get_parameter("path_generation_config.start_state.name",
                                 sp_.start_state_name) ||
            !this->get_parameter("path_generation_config.start_state.package",
                                 sp_.start_state_pkg) ||
            !this->get_parameter("path_generation_config.start_state.directory",
                                 sp_.start_state_dir)) {
          return false;
        }
        std::string yaml_path =
            ament_index_cpp::get_package_share_directory(sp_.start_state_pkg) +
            "/" + sp_.start_state_dir + "/" + sp_.start_state_name;

        // load yaml and catch exceptions
        try {
          YAML::Node config = YAML::LoadFile(yaml_path);

          // get start state map
          sp_.start_state =
              config["start_state"].as<std::map<std::string, double>>();

        } catch (const YAML::ParserException& pe) {
          return false;
        } catch (const YAML::BadFile& be) {
          return false;
        }
      }

      if (!this->get_parameter(
              "path_generation_config.robot_configurations.name",
              sp_.robot_configurations_name) ||
          !this->get_parameter(
              "path_generation_config.robot_configurations.package",
              sp_.robot_configurations_package) ||
          !this->get_parameter(
              "path_generation_config.robot_configurations.directory",
              sp_.robot_configurations_dir)) {
        return false;
      }

      if (std::find(sp_.visualize_dbs.begin(), sp_.visualize_dbs.end(), "") !=
          sp_.visualize_dbs.end()) {
        sp_.visualize_dbs.clear();
      }

      // set params
      sp = sp_;

      return true;
    }
  }

 private:
  reach::core::StudyParameters sp_;
};

int main(int argc, char** argv) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  // create node
  auto node = std::make_shared<RobotReachStudyNode>("robot_reach_study_node");

  executor.add_node(node);

  // get the study parameters
  reach::core::StudyParameters sp;
  if (!node->getStudyParameters(sp)) {
    return -1;
  }

  std::thread t1([&executor] {
    // spin
    executor.spin();
  });

  // Initialize the reach study
  reach::core::ReachStudy rs(node);

  // Run the reach study
  if (!rs.run(sp) || !rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_reach_study_node"),
                 "Unable to perform the reach study");
    rclcpp::shutdown();
    t1.join();
    return 0;
  }

  rclcpp::shutdown();
  t1.join();

  return 0;
}
