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
#include "tf2_eigen/tf2_eigen.hpp"

#include <exception>
#include <filesystem>
#include <numeric>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_loader.hpp>
#include <reach_core/reach_study.h>
#include <reach_core/utils/general_utils.h>
#include <reach_core/utils/serialization_utils.h>
#include <reach_msgs/msg/reach_record.hpp>
#include <reach_msgs/srv/load_point_cloud.hpp>

// yaml stuff
#include <yaml-cpp/yaml.h>

constexpr char SAMPLE_MESH_SRV_TOPIC[] = "sample_mesh";
const static double SRV_TIMEOUT = 5.0;
constexpr char INPUT_CLOUD_TOPIC[] = "input_cloud";
constexpr char SAVED_DB_NAME[] = "reach.db";
constexpr char OPT_SAVED_DB_NAME[] = "optimized_reach.db";

namespace reach {
namespace core {

constexpr char PACKAGE[] = "reach_core";
constexpr char IK_BASE_CLASS[] = "reach::plugins::IKSolverBase";
constexpr char DISPLAY_BASE_CLASS[] = "reach::plugins::DisplayBase";
constexpr char PATH_BASE_CLASS[] = "reach::plugins::PathBase";

ReachStudy::ReachStudy(const rclcpp::Node::SharedPtr node)
    : node_(node),
      cloud_(new pcl::PointCloud<pcl::PointNormal>()),
      db_(new ReachDatabase()),
      solver_loader_(PACKAGE, IK_BASE_CLASS),
      display_loader_(PACKAGE, DISPLAY_BASE_CLASS),
      path_generator_loader_(PACKAGE, PATH_BASE_CLASS) {}

ReachStudy::~ReachStudy() {
  ik_solver_.reset();
  display_.reset();
  visualizer_.reset();
  db_.reset();
  cloud_.reset();
  node_.reset();
  model_.reset();
  paths_db_.reset();
  for (size_t i = 0; i < path_generators_.size(); ++i) {
    path_generators_[i].reset();
  }
}

bool ReachStudy::initializeStudy(const StudyParameters& sp) {
  ik_solver_.reset();
  display_.reset();
  // create robot model shared ptr
  model_ = moveit::planning_interface::getSharedRobotModel(node_,
                                                           "robot_description");

  // expose information if analysis has finished
  done_pub_ = node_->create_publisher<std_msgs::msg::Empty>("analysis_done", 1);

  try {
    ik_solver_ = solver_loader_.createSharedInstance(sp_.ik_solver_config_name);
    display_ = display_loader_.createSharedInstance(sp_.display_config_name);
  } catch (const pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Pluginlib exception thrown while creating shared instances "
                 "of ik solver and/or display: '%s'",
                 ex.what());
    ik_solver_.reset();
    display_.reset();
    return false;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Error while creating shared instances of ik solver and/or "
                 "display: '%s'",
                 ex.what());
    ik_solver_.reset();
    display_.reset();
    return false;
  }

  // Initialize the IK solver plugin and display plugin
  if (!ik_solver_->initialize(sp_.ik_solver_config_name, node_, model_) ||
      !display_->initialize(sp_.display_config_name, node_, model_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Could not initialized both display and ik solver plugins!");
    ik_solver_.reset();
    display_.reset();
    return false;
  }

  // Create a directory to store results of study
  std::string tmp_dir =
      ament_index_cpp::get_package_share_directory(sp_.results_package) + "/" +
      sp_.results_directory;
  if (!tmp_dir.empty() && std::filesystem::exists(tmp_dir.c_str())) {
    dir_ = tmp_dir + "/";
  } else {
    dir_ = ament_index_cpp::get_package_share_directory("reach_core") +
           "/results/";
    RCLCPP_WARN(node_->get_logger(),
                "Using default results file directory: '%s'", dir_.c_str());
  }
  results_dir_ = dir_ + sp_.config_name + "/";
  const char* char_dir = results_dir_.c_str();

  if (!std::filesystem::exists(char_dir)) {
    std::filesystem::path path(char_dir);
    std::filesystem::create_directory(path);
  }
  if (sp_.keep_running) {
    // sleep to visualize collision object
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  display_->showEnvironment();

  return true;
}

bool ReachStudy::run(const StudyParameters& sp) {
  RCLCPP_INFO(node_->get_logger(),
              "RUN RUN RUN RUN RUN RUN RUN RUN RUN RUN RUN ");

  // Overwrite the old study parameters
  sp_ = sp;

  // Initialize the study
  if (!initializeStudy(sp)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize the reach study");
    return false;
  }

  // Get the reach object point cloud
  if (!getReachObjectPointCloud()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Unable to obtain reach object point cloud");
    ik_solver_.reset();
    display_.reset();
    return false;
  }

  // Show the reach object collision object and reach object point cloud
  if (sp_.visualize_results) {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            INPUT_CLOUD_TOPIC, 1);
    pub->publish(cloud_msg_);
  }

  // Create markers
  visualizer_.reset(new ReachVisualizer(db_, ik_solver_, display_,
                                        sp_.optimization.radius, nullptr,
                                        &path_generators_));

  if (!sp_.generate_paths_only) {
    // Attempt to load previously saved optimized reach_study database
    if (!db_->load(results_dir_ + OPT_SAVED_DB_NAME)) {
      RCLCPP_INFO(node_->get_logger(),
                  "Unable to load optimized database at '%s'!",
                  (results_dir_ + OPT_SAVED_DB_NAME).c_str());
      // Attempt to load previously saved initial reach study database
      if (!db_->load(results_dir_ + SAVED_DB_NAME)) {
        RCLCPP_INFO(node_->get_logger(), "------------------------------");
        RCLCPP_INFO(node_->get_logger(), "No reach study database loaded");
        RCLCPP_INFO(node_->get_logger(), "------------------------------");

        // Run the first pass of the reach study
        runInitialReachStudy();
        db_->printResults();
        visualizer_->update();
        // check if we don't have to optimize
        if (sp.run_initial_study_only) {
          done_pub_->publish(std_msgs::msg::Empty());
          while (rclcpp::ok() && sp_.keep_running) {
            // keep it running
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          db_->calculateResults();
          db_->save(results_dir_ + SAVED_DB_NAME);
          return true;
        }
      } else {
        RCLCPP_INFO(node_->get_logger(),
                    "----------------------------------------------------");
        RCLCPP_INFO(node_->get_logger(),
                    "Unoptimized reach study database successfully loaded");
        RCLCPP_INFO(node_->get_logger(),
                    "----------------------------------------------------");

        db_->printResults();
        visualizer_->update();

        // check if we don't have to optimize
        if (sp.run_initial_study_only) {
          done_pub_->publish(std_msgs::msg::Empty());

          while (rclcpp::ok() && sp_.keep_running) {
            // keep it running
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }

          db_->calculateResults();
          db_->save(results_dir_ + SAVED_DB_NAME);
          return true;
        }
      }

      // Create an efficient search tree for doing nearest neighbors search
      search_tree_.reset(
          new SearchTree(flann::KDTreeSingleIndexParams(1, true)));

      flann::Matrix<double> dataset(new double[db_->size() * 3], db_->size(),
                                    3);
      for (std::size_t i = 0; i < db_->size(); ++i) {
        auto it = db_->begin();
        std::advance(it, i);

        dataset[i][0] = static_cast<double>(it->second.goal.position.x);
        dataset[i][1] = static_cast<double>(it->second.goal.position.y);
        dataset[i][2] = static_cast<double>(it->second.goal.position.z);
      }
      search_tree_->buildIndex(dataset);

      // Run the optimization
      optimizeReachStudyResults();
      db_->printResults();
      visualizer_->update();
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "--------------------------------------------------");
      RCLCPP_INFO(node_->get_logger(),
                  "Optimized reach study database successfully loaded");
      RCLCPP_INFO(node_->get_logger(),
                  "--------------------------------------------------");

      db_->printResults();
      visualizer_->update();
    }
  }

  // check if paths should be generated
  if (sp_.generate_paths || sp_.generate_paths_only) {
    if (!initializePathGeneration(sp_)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Path Generation could not be initialized");
      return false;
    }
    // path generation
    generatePaths();
    db_->save(paths_path_);
    db_->printResults();
    visualizer_->update();
  }

  // Find the average number of neighboring points can be reached by the robot
  // from any given point
  if (sp_.get_neighbors) {
    // Perform the calculation if it hasn't already been done
    if (db_->getStudyResults().avg_num_neighbors == 0.0f) {
      getAverageNeighborsCount();
    }
  }

  // Visualize the results of the reach study
  if (sp_.visualize_results) {
    // Compare database results
    if (!sp_.compare_dbs.empty()) {
      if (!compareDatabases()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Unable to compare the current reach study database with "
                     "the other specified databases");
      }
    }
    if (!sp_.visualize_dbs.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Visualizing databases...");
      if (!visualizeDatabases()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Unable to compare the current reach study database with "
                     "the other specified databases");
      }
    }
  }

  // analysis done information sharing
  done_pub_->publish(std_msgs::msg::Empty());

  while (rclcpp::ok() && sp_.keep_running) {
    // keep it running
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ik_solver_.reset();
  display_.reset();
  visualizer_.reset();
  for (size_t i = 0; i < path_generators_.size(); ++i) {
    path_generators_[i].reset();
  }

  return true;
}

bool ReachStudy::getReachObjectPointCloud() {
  // Call the sample mesh service to create a point cloud of the reach object
  // mesh
  auto callback_group_input_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  auto client = node_->create_client<reach_msgs::srv::LoadPointCloud>(
      SAMPLE_MESH_SRV_TOPIC, rmw_qos_profile_services_default,
      callback_group_input_);

  auto req = std::make_shared<reach_msgs::srv::LoadPointCloud::Request>();
  req->cloud_filename =
      ament_index_cpp::get_package_share_directory(sp_.pcd_package) + "/" +
      sp_.pcd_filename_path;
  req->fixed_frame = sp_.fixed_frame;
  req->object_frame = sp_.object_frame;

  RCLCPP_INFO(node_->get_logger(), "Waiting for service '%s'.",
              SAMPLE_MESH_SRV_TOPIC);
  client->wait_for_service();
  bool success_tmp = false;
  bool inner_callback_finished = false;

  auto inner_client_callback =
      [&, this](rclcpp::Client<reach_msgs::srv::LoadPointCloud>::SharedFuture
                    inner_future) {
        success_tmp = inner_future.get()->success;
        cloud_msg_ = inner_future.get()->cloud;
        RCLCPP_DEBUG(node_->get_logger(),
                     "Inner service callback message: '%s'",
                     inner_future.get()->message.c_str());
        inner_callback_finished = true;
      };
  auto inner_future_result =
      client->async_send_request(req, inner_client_callback);

  // quick fix to wait for inner callback to finish
  // TODO(livanov93) Quick fix - find other solution
  while (!inner_callback_finished) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (success_tmp) {
    pcl::fromROSMsg(cloud_msg_, *cloud_);

    cloud_msg_.header.frame_id = sp_.fixed_frame;
    cloud_msg_.header.stamp = node_->now();

    return true;

  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Failed to call point cloud loading service '"
                            << client->get_service_name() << "'");
    return false;
  }
}

void ReachStudy::runInitialReachStudy() {
  // Rotation to flip the Z axis of the surface normal point
  const Eigen::AngleAxisd tool_z_rot(M_PI, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd tool_z_rot_align(M_PI, Eigen::Vector3d::UnitZ());

  // Loop through all points in point cloud and get IK solution
  std::atomic<int> current_counter, previous_pct;
  current_counter = previous_pct = 0;
  const int cloud_size = static_cast<int>(cloud_->points.size());

#pragma omp parallel for
  for (int i = 0; i < cloud_size; ++i) {
    // Get pose from point cloud array
    const pcl::PointNormal& pt = cloud_->points[i];
    Eigen::Isometry3d tgt_frame;
    tgt_frame =
        utils::createFrame(pt.getArray3fMap(), pt.getNormalVector3fMap());
    if (sp_.invert_z_tool_rotation) {
      tgt_frame = tgt_frame * tool_z_rot * tool_z_rot_align;
    } else {
      tgt_frame = tgt_frame * tool_z_rot;
    }

    // Get the seed position
    sensor_msgs::msg::JointState seed_state;
    seed_state.name = ik_solver_->getJointNames();
    if (seed_state.name.size() != sp_.initial_seed_state.size()) {
      seed_state.position = std::vector<double>(seed_state.name.size(), 0.0);
    } else {
      seed_state.position = sp_.initial_seed_state;
    }

    // Solve IK
    std::vector<double> solution;
    std::optional<double> score = ik_solver_->solveIKFromSeed(
        tgt_frame, jointStateMsgToMap(seed_state), solution);

    // Create objects to save in the reach record
    geometry_msgs::msg::Pose tgt_pose;
    tgt_pose = tf2::toMsg(tgt_frame);

    sensor_msgs::msg::JointState goal_state(seed_state);

    if (score) {
      geometry_msgs::msg::PoseStamped tgt_pose_stamped;
      tgt_pose_stamped.pose = tgt_pose;
      tgt_pose_stamped.header.frame_id = cloud_msg_.header.frame_id;

      // fill the goal state
      goal_state.position = solution;

      auto msg = makeRecord(std::to_string(i), true, tgt_pose, seed_state,
                            goal_state, *score);
      db_->put(msg);
    } else {
      auto msg = makeRecord(std::to_string(i), false, tgt_pose, seed_state,
                            goal_state, 0.0);
      db_->put(msg);
    }

    // Print function progress
    current_counter++;
    utils::integerProgressPrinter(current_counter, previous_pct, cloud_size);
  }

  // Save the results of the reach study to a database that we can query later
  db_->calculateResults();
  db_->save(results_dir_ + SAVED_DB_NAME);
}

void ReachStudy::optimizeReachStudyResults() {
  RCLCPP_INFO(node_->get_logger(), "----------------------");
  RCLCPP_INFO(node_->get_logger(), "Beginning optimization");

  // Create sequential vector to be randomized
  std::vector<std::size_t> rand_vec(db_->size());
  std::iota(rand_vec.begin(), rand_vec.end(), 0);

  // Iterate
  std::atomic<int> current_counter, previous_pct;
  const int cloud_size = static_cast<int>(cloud_->size());
  int n_opt = 0;
  float previous_score = 0.0;
  float pct_improve = 1.0;

  while (pct_improve > sp_.optimization.step_improvement_threshold &&
         n_opt < sp_.optimization.max_steps) {
    RCLCPP_INFO(node_->get_logger(), "Entering optimization loop %d", n_opt);
    previous_score = db_->getStudyResults().norm_total_pose_score;
    current_counter = 0;
    previous_pct = 0;

    // Randomize
    std::random_shuffle(rand_vec.begin(), rand_vec.end());

#pragma parallel for
    for (std::size_t i = 0; i < rand_vec.size(); ++i) {
      auto it = db_->begin();
      std::advance(it, rand_vec[i]);
      reach_msgs::msg::ReachRecord msg = it->second;
      if (msg.reached) {
        NeighborReachResult result = reachNeighborsDirect(
            db_, msg, ik_solver_, sp_.optimization.radius);  // search_tree_);
      }
      // Print function progress
      current_counter++;
      utils::integerProgressPrinter(current_counter, previous_pct, cloud_size);
    }

    // Recalculate optimized reach study results
    db_->calculateResults();
    db_->printResults();
    pct_improve = std::abs(
        (db_->getStudyResults().norm_total_pose_score - previous_score) /
        previous_score);
    ++n_opt;
  }

  // Save the optimized reach database
  db_->calculateResults();
  db_->save(results_dir_ + OPT_SAVED_DB_NAME);

  RCLCPP_INFO(node_->get_logger(), "----------------------");
  RCLCPP_INFO(node_->get_logger(), "Optimization concluded");
}

void ReachStudy::getAverageNeighborsCount() {
  RCLCPP_INFO(node_->get_logger(),
              "--------------------------------------------");
  RCLCPP_INFO(node_->get_logger(),
              "Beginning average neighbor count calculation");

  std::atomic<int> current_counter, previous_pct, neighbor_count;
  current_counter = previous_pct = neighbor_count = 0;
  std::atomic<double> total_joint_distance = 0.0;
  const int total = db_->size();
  // Iterate
  for (auto it = db_->begin(); it != db_->end(); ++it) {
    reach_msgs::msg::ReachRecord msg = it->second;
    if (msg.reached) {
      //      NeighborReachResult result;
      //      // TODO(livanov93): find out why using search_tree_ throttles down
      //      the loop reachNeighborsRecursive(db_, msg, ik_solver_,
      //      sp_.optimization.radius,
      //                              result);//, search_tree_);
      NeighborReachResult result =
          reachNeighborsDirect(db_, msg, ik_solver_, sp_.optimization.radius);
      neighbor_count += static_cast<int>(result.reached_pts.size() - 1);
      total_joint_distance = total_joint_distance + result.joint_distance;
    }

    // Print function progress
    ++current_counter;
    utils::integerProgressPrinter(current_counter, previous_pct, total);
  }

  float avg_neighbor_count = static_cast<float>(neighbor_count.load()) /
                             static_cast<float>(db_->size());
  float avg_joint_distance = static_cast<float>(total_joint_distance.load()) /
                             static_cast<float>(neighbor_count.load());

  RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "Average number of neighbors reached: " << avg_neighbor_count);
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Average joint distance: " << avg_joint_distance);
  RCLCPP_INFO(node_->get_logger(),
              "------------------------------------------------");

  db_->setAverageNeighborsCount(avg_neighbor_count);
  db_->setAverageJointDistance(avg_joint_distance);
  db_->save(results_dir_ + OPT_SAVED_DB_NAME);
}

bool ReachStudy::compareDatabases() {
  // Add the newly created database to the list if it isn't already there
  if (std::find(sp_.compare_dbs.begin(), sp_.compare_dbs.end(),
                sp_.config_name) == sp_.compare_dbs.end()) {
    sp_.compare_dbs.push_back(sp_.config_name);
  }

  // Create list of optimized database file names from the results folder
  std::vector<std::string> db_filenames;
  for (auto it = sp_.compare_dbs.begin(); it != sp_.compare_dbs.end(); ++it) {
    db_filenames.push_back(dir_ + *it + "/" + OPT_SAVED_DB_NAME);
  }

  // Load databases to be compared
  std::map<std::string, reach_msgs::msg::ReachDatabase> data;
  for (size_t i = 0; i < db_filenames.size(); ++i) {
    ReachDatabase db;
    if (!db.load(db_filenames[i])) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot load database at:\n %s",
                   db_filenames[i].c_str());
      continue;
    }
    data.emplace(sp_.compare_dbs[i], db.toReachDatabaseMsg());
  }

  if (data.size() < 2) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Only %lu database(s) loaded; cannot compare fewer than 2 databases",
        data.size());
    return false;
  }

  display_->compareDatabases(data);

  return true;
}

bool ReachStudy::visualizeDatabases() {
  // Add the newly created database to the list if it isn't already there
  if (std::find(sp_.visualize_dbs.begin(), sp_.visualize_dbs.end(),
                sp_.config_name) == sp_.visualize_dbs.end()) {
    sp_.visualize_dbs.push_back(sp_.config_name);
  }
  // Create list of optimized database file names from the results folder
  std::vector<std::string> db_filenames;
  for (auto it = sp_.visualize_dbs.begin(); it != sp_.visualize_dbs.end();
       ++it) {
    db_filenames.push_back(dir_ + *it + "/" + OPT_SAVED_DB_NAME);
  }

  // Load databases to be compared
  std::map<std::string, reach_msgs::msg::ReachDatabase> data;
  for (size_t i = 0; i < db_filenames.size(); ++i) {
    ReachDatabase db;
    if (!db.load(db_filenames[i])) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot load database at:\n %s",
                   db_filenames[i].c_str());
      continue;
    }
    data.emplace(sp_.visualize_dbs[i], db.toReachDatabaseMsg());
  }

  if (data.size() < 2) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Only %lu database(s) loaded; cannot compare fewer than 2 databases",
        data.size());
    return false;
  }

  display_->visualizeDatabases(data);

  return true;
}

bool ReachStudy::initializePathGeneration(const StudyParameters& sp) {
  // create instances
  path_generators_.resize(sp.paths.size());
  for (size_t i = 0; i < sp.paths.size(); ++i) {
    path_generators_[i] =
        path_generator_loader_.createSharedInstance(sp.path_based_plugins[i]);
    if (!path_generators_[i]->initialize(sp.paths[i], node_, model_)) {
      RCLCPP_ERROR(node_->get_logger(), "Could not initialize '%s' '%s'",
                   sp.paths[i].c_str(), sp.path_based_plugins[i].c_str());
      return false;
    }
  }

  if (sp.initial_source_db) {
    //    paths_path_ =
    //    ament_index_cpp::get_package_share_directory(sp.db_package) +
    //                  "/" + sp.db_dir + "/" + sp.db_config_name + "/" +
    //                  sp.db_name;
    //    if (!std::filesystem::exists(paths_path_)) {
    //      return false;
    //    } else {
    //      bool loaded = paths_db_->load(paths_path_);
    //      if (!loaded) return false;
    //    }
  } else if (sp.initial_source_robot_configurations) {
    std::string yaml_path = ament_index_cpp::get_package_share_directory(
                                sp.robot_configurations_package) +
                            "/" + sp.robot_configurations_dir + "/" +
                            sp.robot_configurations_name;
    // check if yaml exists
    if (!std::filesystem::exists(yaml_path)) {
      RCLCPP_ERROR(node_->get_logger(), "No file found under path '%s'",
                   yaml_path.c_str());
      return false;
    }

    // load yaml
    YAML::Node config = YAML::LoadFile(yaml_path);
    for (const auto& robot_configuration :
         config["robot_configurations"].as<std::vector<std::string>>()) {
      robot_configurations_[robot_configuration] =
          config[robot_configuration].as<std::map<std::string, double>>();

      // initialize target, seed state and goal state
      reach_msgs::msg::ReachRecord r;
      {
        moveit::core::RobotState rs(model_);
        auto jmg = model_->getJointModelGroup(sp_.planning_group);
        if (!jmg) {
          RCLCPP_ERROR_STREAM(node_->get_logger(),
                              "Failed to get joint model group for '"
                                  << sp_.planning_group << "'");
          return false;
        }
        std::vector<double> start_state_subset;
        if (!transcribeInputMap(robot_configurations_[robot_configuration],
                                jmg->getActiveJointModelNames(),
                                start_state_subset)) {
          RCLCPP_ERROR(node_->get_logger(), "Could not transcribe input map");
          return false;
        }
        rs.setJointGroupPositions(jmg, start_state_subset);
        // count that it is reached state
        r.reached = true;
        r.id = robot_configuration;
        r.goal =
            tf2::toMsg(rs.getGlobalLinkTransform(sp_.fixed_frame).inverse() *
                       rs.getGlobalLinkTransform(sp_.tool_frame));
        r.goal_state =
            mapToJointStateMsg(robot_configurations_[robot_configuration]);
        r.seed_state =
            mapToJointStateMsg(robot_configurations_[robot_configuration]);
      }
      db_->put(r);
    }

    // create database
    //          std::string tmp_db_path =
    //                  ament_index_cpp::get_package_share_directory(sp.db_package)
    //                  + "/" + sp.db_dir + "/" + sp.db_config_name + "/";
    //          if (!std::filesystem::exists(tmp_db_path)) {
    //              std::filesystem::path path(tmp_db_path.c_str());
    //              std::filesystem::create_directories(path);
    //          }
    //          paths_path_ = tmp_db_path + sp.db_name;
    //          // save database
    //          paths_db_->save(paths_path_);

  } else {
    paths_db_ = nullptr;
    return false;
  }

  {
    // create database
    std::string tmp_db_path =
        ament_index_cpp::get_package_share_directory(sp.db_package) + "/" +
        sp.db_dir + "/" + sp.db_config_name + "/";
    if (!std::filesystem::exists(tmp_db_path)) {
      std::filesystem::path path(tmp_db_path.c_str());
      std::filesystem::create_directories(path);
    }
    paths_path_ = tmp_db_path + sp.db_name;

    db_->save(paths_path_);
  }

  RCLCPP_INFO(node_->get_logger(), "Path Generation Initialized Successfully");
  return true;
}

void ReachStudy::generatePaths() {
  RCLCPP_INFO(node_->get_logger(), "Generating paths...");
  const int total_size = static_cast<int>(db_->size());
  // Loop through all points in point cloud and get IK solution
  std::atomic<int> current_counter, previous_pct;
  current_counter = previous_pct = 0;

  // for each record
  for (auto it = db_->begin(); it != db_->end(); ++it) {
    // get the whole record to update it with paths
    reach_msgs::msg::ReachRecord r = it->second;
    if (r.reached) {
      // if it is reached clear paths
      r.paths.clear();

      std::map<std::string, double> start_state;
      std::map<std::string, double> end_state;

      // get target from db
      // decide if first point is goto or from
      if (sp_.db_goals_are_start) {
        start_state = jointStateMsgToMap(r.goal_state);
      } else {
        start_state = sp_.start_state;
        end_state = jointStateMsgToMap(r.goal_state);
      }

      double fraction = 0.0;
      moveit_msgs::msg::RobotTrajectory trajectory;

      bool total_score_ok = false;

      // for each path generator
      for (size_t i = 0; i < path_generators_.size(); ++i) {
        std::optional<double> score = path_generators_[i]->solvePath(
            start_state, end_state, fraction, trajectory);

        // check if there is score
        if (score) {
          total_score_ok = true;
          reach_msgs::msg::ReachPath path;
          path.fraction = fraction;
          path.moveit_trajectory = trajectory;
          r.paths.push_back(path);

          // prepare for next path generator
          start_state = end_state;
          end_state.clear();

        } else {
          reach_msgs::msg::ReachPath path;
          path.fraction = fraction;
          //          path.moveit_trajectory = trajectory;
          r.paths.push_back(path);

          total_score_ok = false;
          break;
        }
      }
      db_->put(r);
    }

    // Print function progress
    current_counter++;
    utils::integerProgressPrinter(current_counter, previous_pct, total_size);
  }
  RCLCPP_INFO(node_->get_logger(), "Paths generated!");
}

}  // namespace core
}  // namespace reach
