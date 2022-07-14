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

#include "moveit_reach_plugins/path/free_space_path_generation.h"

#include "moveit_reach_plugins/utils.h"

#include <algorithm>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <reach_core/utils/general_utils.h>

namespace {

template <typename T>
T clamp(const T& val, const T& low, const T& high) {
  return std::max(low, std::min(val, high));
}

}  // namespace

namespace moveit_reach_plugins {
namespace path {

FreeSpacePathGeneration::FreeSpacePathGeneration() : PathBase() {}

bool FreeSpacePathGeneration::initialize(
    const std::string& name, rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const moveit::core::RobotModel> model) {
  // interface to outer ros params and descriptions
  node_ = node;
  model_ = model;
  name_ = name;

  auto LOGGER = rclcpp::get_logger(
      name_ + ".moveit_reach_plugins.FreeSpacePathGeneration");

  std::string param_prefix = "path_generation_config." + name_ + ".";
  // get parameters for FreeSpace path computation
  try {
    double global_tx = 0.0, global_ty = 0.0, global_tz = 0.0;
    double global_rx = 0.0, global_ry = 0.0, global_rz = 0.0;
    if (!node->get_parameter(param_prefix + "global_transformation.tx",
                             global_tx)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.tx").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.ty",
                             global_ty)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.ty").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.tz",
                             global_tz)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.tz").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.rx",
                             global_rx)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.rx").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.ry",
                             global_ry)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.ry").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.rz",
                             global_rz)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.rz").c_str());
      return false;
    }
    // init global transformation
    global_transformation_ =
        Eigen::Translation3d(global_tx, global_ty, global_tz);
    global_transformation_ =
        global_transformation_ *
        Eigen::AngleAxisd(global_rx * M_PI / 180.0, Eigen::Vector3d::UnitX());
    global_transformation_ =
        global_transformation_ *
        Eigen::AngleAxisd(global_ry * M_PI / 180.0, Eigen::Vector3d::UnitY());
    global_transformation_ =
        global_transformation_ *
        Eigen::AngleAxisd(global_rz * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    double tool_tx = 0.0, tool_ty = 0.0, tool_tz = 0.0;
    double tool_rx = 0.0, tool_ry = 0.0, tool_rz = 0.0;
    if (!node->get_parameter(param_prefix + "tool_transformation.tx",
                             tool_tx)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.tx").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.ty",
                             tool_ty)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.ty").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.tz",
                             tool_tz)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.tz").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.rx",
                             tool_rx)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.rx").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.ry",
                             tool_ry)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.ry").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.rz",
                             tool_rz)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.rz").c_str());
      return false;
    }

    // init tool transformation
    tool_transformation_ = Eigen::Translation3d(tool_tx, tool_ty, tool_tz);
    tool_transformation_ =
        tool_transformation_ *
        Eigen::AngleAxisd(tool_rx * M_PI / 180.0, Eigen::Vector3d::UnitX());
    tool_transformation_ =
        tool_transformation_ *
        Eigen::AngleAxisd(tool_ry * M_PI / 180.0, Eigen::Vector3d::UnitY());
    tool_transformation_ =
        tool_transformation_ *
        Eigen::AngleAxisd(tool_rz * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    if (!node->get_parameter(param_prefix + "distance_threshold",
                             distance_threshold_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "distance_threshold").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "tool_frame", tool_frame_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_frame").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "max_velocity_scaling_factor",
                             max_velocity_scaling_factor_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "max_velocity_scaling_factor").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "max_acceleration_scaling_factor",
                             max_acceleration_scaling_factor_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "max_acceleration_scaling_factor").c_str());
      return false;
    }

    if (!node->get_parameter("path_generation_config.collision_mesh_package",
                             collision_mesh_package_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   "path_generation_config.collision_mesh_package");
      return false;
    }

    if (!node->get_parameter("path_generation_config.collision_mesh_frame",
                             collision_mesh_frame_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   "path_generation_config.collision_mesh_frame");
      return false;
    }

    if (!node->get_parameter(param_prefix + "touch_links", touch_links_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "touch_links").c_str());
      return false;
    }

    if (std::find(touch_links_.begin(), touch_links_.end(), "") !=
        touch_links_.end()) {
      touch_links_.clear();
    }

    if (!node->get_parameter(param_prefix + "planner_id", planner_id_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'%s' ",
                   (param_prefix + "planner_id").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "allowed_planning_time",
                             allowed_planning_time_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'%s' ",
                   (param_prefix + "allowed_planning_time").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "poses_frame_id",
                             poses_frame_id_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'%s' ",
                   (param_prefix + "poses_frame_id").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "display_motion_plans",
                             display_motion_plans_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'%s' ",
                   (param_prefix + "display_motion_plans").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "publish_planning_requests",
                             publish_planning_requests_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'%s' ",
                   (param_prefix + "publish_planning_requests").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "num_planning_attempts",
                             num_planning_attempts_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'%s' ",
                   (param_prefix + "num_planning_attempts").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "pipeline_name", pipeline_name_)) {
      RCLCPP_ERROR(LOGGER,
                   "No parameter defined by the name "
                   "'%s' ",
                   (param_prefix + "pipeline_name").c_str());
      return false;
    }

    node->get_parameter_or(param_prefix + "goal_joint_tolerance",
                           goal_joint_tolerance_, 1e-4);
    node->get_parameter_or(param_prefix + "goal_position_tolerance",
                           goal_position_tolerance_, 1e-4);
    node->get_parameter_or(param_prefix + "goal_orientation_tolerance",
                           goal_orientation_tolerance_, 1e-4);

  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(LOGGER, ex.what());
    return false;
  }

  workspace_parameter_ = moveit_msgs::msg::WorkspaceParameters();

  // planner creation
  if (!planner_) {
    PlanningSpecs spec;
    spec._robot_model = model;
    spec._planning_pipeline = pipeline_name_;
    planner_ = create(node_, spec);
    planner_->displayComputedMotionPlans(display_motion_plans_);
    planner_->publishReceivedRequests(publish_planning_requests_);

  } else if (model != planner_->getRobotModel()) {
    throw std::runtime_error(
        "Difference in robot models between planner and ik solver");
  }
  robot_start_state_pub_ =
      node->create_publisher<moveit_msgs::msg::DisplayRobotState>(
          "planner_based_ik_solver_start_state", 1);

  robot_end_state_pub_ =
      node->create_publisher<moveit_msgs::msg::DisplayRobotState>(
          "planner_based_ik_solver_end_state", 1);

  // init planning scene
  scene_ = std::make_shared<planning_scene::PlanningScene>(model_);

  // Check that the input collision mesh frame exists
  if (!scene_->knowsFrameTransform(collision_mesh_frame_)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Specified collision mesh frame '"
                                    << collision_mesh_frame_
                                    << "' does not exist");
    return false;
  }

  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::msg::CollisionObject obj = utils::createCollisionObject(
      collision_mesh_package_, collision_mesh_frame_, object_name);
  if (!scene_->processCollisionObjectMsg(obj)) {
    RCLCPP_ERROR(LOGGER, "Failed to add collision mesh to planning scene");
    return false;
  } else {
    scene_->getAllowedCollisionMatrixNonConst().setEntry(object_name,
                                                         touch_links_, true);
  }

  std::string planning_group;
  if (!node->get_parameter("planning_group", planning_group)) {
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if (!jmg_) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to get joint model group for '"
                                    << planning_group << "'");
    return false;
  }

  for (const auto& j : jmg_->getActiveJointModelNames()) {
    double value = 0.0;
    if (node->get_parameter(param_prefix + "end_state." + j, value)) {
      RCLCPP_INFO(LOGGER, "Found value...");

      // store value
      end_state_[j] = value;
    } else {
      RCLCPP_INFO(LOGGER, "Cleared end_state_");
      end_state_.clear();
      break;
    }
  }

  // output message about successful initialization
  RCLCPP_INFO(LOGGER, "Successfully initialized '%s' plugin",
              (name_ + ".FreeSpacePathGeneration").c_str());
  return true;
}

// https://github.com/ros-planning/moveit_task_constructor/blob/60229db010ea305296bc1c90d04faa3e4dacd976/core/src/solvers/pipeline_planner.cpp#L51-L75
struct PlannerCache {
  using PlannerID = std::tuple<std::string, std::string>;
  using PlannerMap =
      std::map<PlannerID, std::weak_ptr<planning_pipeline::PlanningPipeline>>;
  using ModelList = std::list<
      std::pair<std::weak_ptr<const moveit::core::RobotModel>, PlannerMap>>;
  ModelList cache_;

  PlannerMap::mapped_type& retrieve(
      const moveit::core::RobotModelConstPtr& model, const PlannerID& id) {
    // find model in cache_ and remove expired entries while doing so
    ModelList::iterator model_it = cache_.begin();
    while (model_it != cache_.end()) {
      if (model_it->first.expired()) {
        model_it = cache_.erase(model_it);
        continue;
      }
      if (model_it->first.lock() == model) break;
      ++model_it;
    }
    if (model_it ==
        cache_.end())  // if not found, create a new PlannerMap for this model
      model_it =
          cache_.insert(cache_.begin(), std::make_pair(model, PlannerMap()));

    return model_it->second
        .insert(std::make_pair(id, PlannerMap::mapped_type()))
        .first->second;
  }
};

// https://github.com/ros-planning/moveit_task_constructor/blob/60229db010ea305296bc1c90d04faa3e4dacd976/core/src/solvers/pipeline_planner.cpp#L77-L102
planning_pipeline::PlanningPipelinePtr FreeSpacePathGeneration::create(
    const rclcpp::Node::SharedPtr& node,
    const FreeSpacePathGeneration::PlanningSpecs& spec) {
  static PlannerCache cache;

  static constexpr char const* PLUGIN_PARAMETER_NAME = "planning_plugin";

  std::string pipeline_ns = spec._namespace;
  const std::string parameter_name = pipeline_ns + "." + PLUGIN_PARAMETER_NAME;
  // fallback to old structure for pipeline parameters in MoveIt
  if (!node->has_parameter(parameter_name)) {
    node->declare_parameter(parameter_name,
                            rclcpp::ParameterType::PARAMETER_STRING);
  }
  if (std::string parameter; !node->get_parameter(parameter_name, parameter)) {
    RCLCPP_WARN(node->get_logger(), "Failed to find '%s.%s'. %s",
                pipeline_ns.c_str(), PLUGIN_PARAMETER_NAME,
                "Attempting to load pipeline from old parameter structure. "
                "Please update your MoveIt config.");
    pipeline_ns = "move_group";
  }

  PlannerCache::PlannerID id(pipeline_ns, spec._planning_adapter_param);

  std::weak_ptr<planning_pipeline::PlanningPipeline>& entry =
      cache.retrieve(spec._robot_model, id);
  planning_pipeline::PlanningPipelinePtr planner = entry.lock();
  if (!planner) {
    // create new entry
    planner = std::make_shared<planning_pipeline::PlanningPipeline>(
        spec._robot_model, node, pipeline_ns, PLUGIN_PARAMETER_NAME,
        spec._planning_adapter_param);
    // store in cache
    entry = planner;
  }
  return planner;
}

std::optional<double> FreeSpacePathGeneration::solvePath(
    const std::map<std::string, double>& start_state,
    std::map<std::string, double>& end_state, double& fraction,
    moveit_msgs::msg::RobotTrajectory& moveit_trajectory) {
  moveit::core::RobotState state(model_);

  // if end state is not empty use it directly
  // if end state is empty use internally retrieved end_state_
  // if end_state_ is not defined use start state moved for tool and global
  // transform

  const bool end_state_empty = end_state.empty();

  const std::vector<std::string>& joint_names =
      jmg_->getActiveJointModelNames();

  std::vector<double> start_state_subset;
  if (!utils::transcribeInputMap(start_state, joint_names,
                                 start_state_subset)) {
    auto LOGGER = rclcpp::get_logger(
        name_ + ".moveit_reach_plugins.FreeSpacePathGeneration");
    RCLCPP_ERROR_STREAM(
        LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
    return {};
  }

  state.setJointGroupPositions(jmg_, start_state_subset);
  state.update();

  // free space planning part
  double f = 0.0;

  // initialize motion plan request
  moveit_msgs::msg::MotionPlanRequest req;
  if (!end_state_empty) {
    req = createReqFromJointSpaceGoal(state, end_state);
  } else if (!end_state_.empty()) {
    req = createReqFromJointSpaceGoal(state, end_state_);
  } else {
    req = createReqFromTransformedStartState(state);
  }

  // publish robot start state
  //  moveit_msgs::msg::DisplayRobotState robot_start_state_msg;
  //  robot_start_state_msg.state = req.start_state;
  //  robot_start_state_pub_->publish(robot_start_state_msg);

  ::planning_interface::MotionPlanResponse res;

  bool success = planner_->generatePlan(scene_, req, res);

  //  moveit_msgs::msg::DisplayRobotState robot_end_state_msg;
  //
  //  moveit::core::robotStateToRobotStateMsg(res.trajectory_->getLastWayPoint(),
  //                                          robot_end_state_msg.state);
  //
  //  robot_end_state_pub_->publish(robot_end_state_msg);

  if (success) {
    // set fraction
    f = 1.0;

    trajectory_processing::IterativeParabolicTimeParameterization timing;
    timing.computeTimeStamps(*res.trajectory_, max_velocity_scaling_factor_,
                             max_acceleration_scaling_factor_);

    res.trajectory_->getRobotTrajectoryMsg(moveit_trajectory);

    std::vector<double> end_state_vec;
    res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg_,
                                                               end_state_vec);
    end_state = [&] {
      std::map<std::string, double> ret;
      for (size_t i = 0; i < joint_names.size(); ++i) {
        ret[joint_names[i]] = end_state_vec[i];
      }
      return ret;
    }();
    fraction = f;
    return f;

  } else {
    fraction = 0.0;
    return {};
  }
}

std::vector<std::string> FreeSpacePathGeneration::getJointNames() const {
  return jmg_->getJointModelNames();
}

bool FreeSpacePathGeneration::isIKSolutionValid(
    moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
    const double* ik_solution) const {
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();

  const bool colliding =
      scene_->isStateColliding(*state, jmg->getName(), false);
  const bool too_close =
      (scene_->distanceToCollision(
           *state, scene_->getAllowedCollisionMatrix()) < distance_threshold_);
  return (!colliding && !too_close);
}

moveit_msgs::msg::MotionPlanRequest
FreeSpacePathGeneration::createReqFromTransformedStartState(
    const moveit::core::RobotState& start_state) {
  moveit_msgs::msg::MotionPlanRequest req;

  req.group_name = jmg_->getName();
  req.planner_id = planner_id_;
  req.allowed_planning_time = allowed_planning_time_;

  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  req.num_planning_attempts = num_planning_attempts_;
  req.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  req.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
  req.workspace_parameters = workspace_parameter_;

  req.goal_constraints.resize(1);

  Eigen::Isometry3d final_target;

  const Eigen::Isometry3d& target =
      start_state.getGlobalLinkTransform(tool_frame_);

  // transform in global and local frame
  final_target = global_transformation_ * target * tool_transformation_;

  geometry_msgs::msg::PoseStamped pose_to_plan;
  pose_to_plan.header.frame_id = poses_frame_id_;
  pose_to_plan.pose = tf2::toMsg(final_target);

  req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
      tool_frame_, pose_to_plan, goal_position_tolerance_,
      goal_orientation_tolerance_);

  return req;
}

moveit_msgs::msg::MotionPlanRequest
FreeSpacePathGeneration::createReqFromJointSpaceGoal(
    const moveit::core::RobotState& start_state,
    std::map<std::string, double> end_state) {
  moveit_msgs::msg::MotionPlanRequest req;

  req.group_name = jmg_->getName();
  req.planner_id = planner_id_;
  req.allowed_planning_time = allowed_planning_time_;

  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);

  req.num_planning_attempts = num_planning_attempts_;
  req.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  req.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
  req.workspace_parameters = workspace_parameter_;

  req.goal_constraints.resize(1);

  std::vector<double> end_state_subset;
  if (!utils::transcribeInputMap(end_state, jmg_->getActiveJointModelNames(),
                                 end_state_subset)) {
    auto LOGGER = rclcpp::get_logger(
        name_ + ".moveit_reach_plugins.FreeSpacePathGeneration");
    RCLCPP_ERROR_STREAM(
        LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
    return req;
  }
  moveit::core::RobotState end_robot_state(model_);
  // set end state
  end_robot_state.setJointGroupPositions(jmg_, end_state_subset);
  end_robot_state.update();

  req.goal_constraints[0] =
      kinematic_constraints::constructGoalConstraints(end_robot_state, jmg_);

  return req;
}

}  // namespace path
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::path::FreeSpacePathGeneration,
                       reach::plugins::PathBase)
