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

#include "reach_core/path_generation.h"

#include <filesystem>

#include <moveit/robot_model_loader/robot_model_loader.h>  // for RobotModelLoader

namespace reach {
namespace core {

Configuration Configuration::Make(std::shared_ptr<rclcpp::Node> node,
                                  const std::string& prefix) {
  try {
    auto const loader =
        robot_model_loader::RobotModelLoader(node, "robot_description");
    auto const model = loader.getModel();
    if (model == nullptr) {
      throw std::runtime_error(
          "'robot_description' parameter parsing resulted in nullptr");
    }

    // fetch parameters
    node->get_parameter("ik_solver_config");

    Configuration c;
    c.results_package = "";
    c.results_directory = "";
    c.results_db_name = "";
    c.db = std::make_shared<ReachDatabase>();
    c.robot_model = model;
    c.ik_solver = c.solver_loader.createSharedInstance("");
    c.display = c.display_loader.createSharedInstance("");
    c.path_generators.push_back(c.path_loader.createSharedInstance(""));

    return c;

  } catch (std::exception const& e) {
    throw std::runtime_error(
        "Could not generate configuration for PathGeneration");
  }
}

PathGeneration::PathGeneration(const std::string& name) : rclcpp::Node(name) {}

PathGeneration::~PathGeneration() {}

bool PathGeneration::initialize() { return false; }

bool PathGeneration::visualizeDatabases() { return false; }

bool PathGeneration::getObjectPointCloud() { return false; }

std::string PathGeneration::getFullPathToDb(const std::string& db_pkg,
                                            const std::string& db_dir,
                                            const std::string& db_config,
                                            const std::string& db_name) {
  // Create a directory to store results of study

  std::string full_path_to_db;
  std::string path_tmp =
      ament_index_cpp::get_package_share_directory(db_pkg) + "/" + db_dir;
  if (!path_tmp.empty() && std::filesystem::exists(path_tmp.c_str())) {
    full_path_to_db = path_tmp + "/";
  } else {
    throw std::runtime_error("Could not obtain full path to database");
  }

  full_path_to_db = full_path_to_db + db_config + "/" + db_name;

  if (!std::filesystem::exists(full_path_to_db.c_str())) {
    throw std::runtime_error("Database does not exist");
  }

  return full_path_to_db;
}

}  // namespace core
}  // namespace reach