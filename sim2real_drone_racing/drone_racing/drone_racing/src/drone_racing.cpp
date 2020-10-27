#include "drone_racing/drone_racing.h"

#include <fstream>

#include <autopilot/autopilot_states.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <quadrotor_common/parameter_helper.h>

#include <gazebo_msgs/SetModelState.h>
#include <cstdlib>

#include "std_msgs/Float64MultiArray.h" 

namespace drone_racing {

DroneRacing::DroneRacing
    (const ros::NodeHandle& nh, const ros::NodeHandle& pnh) :
    nh_(nh), pnh_(pnh), desired_state_quad_(), desired_state_world_(), state_estimate_quad_(), state_estimate_world_(),
    state_machine_(State::kOff), copilot_is_in_feedthrough_(false), goal_selected_(false),
    distance_to_goal_(0.0), tf_listener_() {

  std::cout << "re-initiate DroneRacing!!!!!" << std::endl;
    
  state_client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");


  loadParameters();
  state_estimate_sub_ = nh_.subscribe("state_estimate", 1, &DroneRacing::stateEstimateCallback, this);
  start_navigation_sub_ = nh_.subscribe("start_navigation", 1, &DroneRacing::startNavigationCallback,
                                        this);
  hard_stop_sub_ = nh_.subscribe("hard_stop", 1, &DroneRacing::hardStopCallback, this);
  cnn_sub_ = nh_.subscribe("/cnn_out/traj", 1, &DroneRacing::networkCallback, this);
  test_time_sub_ = nh_.subscribe("only_network", 1, &DroneRacing::enableTestTimeCallback, this);

  run_idx_sub_ = nh_.subscribe("run_idx", 1, &DroneRacing::setRunIdxCallback, this);
  env_setup_sub_ = nh_.subscribe("setup_environment", 1, &DroneRacing::setupEnvironmentCallback, this);

  crashed_pub_ = nh_.advertise<std_msgs::Empty>("/crashed", 1);

  passed_gate_pub_ = nh_.advertise<std_msgs::Empty>("/passed_gate", 1);

  feedthrough_pub_ = nh_.advertise<std_msgs::Bool>("copilot/feedthrough", 1);
  desired_state_pub_ = nh_.advertise<quadrotor_msgs::TrajectoryPoint>("autopilot/reference_state", 1);

  divergence_pub_ = nh_.advertise<std_msgs::Float64>("divergence", 1);

  // DAgger_expand_pub_ = nh_.advertise<std_msgs::Bool>("/DAgger_threshold_expand", 1);

  force_hover_pub_ = nh_.advertise<std_msgs::Empty>("/hummingbird/autopilot/force_hover", 1);

  failed_trials_ = 0;
  model_based_frame_ = 0;
  executed_nn_traj_ = 0;

  data_saver_.reset(new DataSaver());
  visualizer_.reset(new Visualizer());
  trajectory_manager_.reset(new TrajectoryManager(pnh_));
  global_trajectory_.reset(new GlobalTrajectory(pnh_));
  gate_replacer_.reset(new gate_replacer::GateReplacer);
  gazebo_rviz_visualizer_.reset(new gazebo_rviz_visualizer::GazeboRvizVisualizer);
  environment_is_set_up_ = false;
  main_loop_timer_ = nh_.createTimer(ros::Duration(0.02), &DroneRacing::mainloop, this);

  // final version
  state_estimate_quad_prev_ = state_estimate_quad_;
  state_estimate_quad_prev_prev_ = state_estimate_quad_;
}

DroneRacing::~DroneRacing() = default;


void DroneRacing::setupEnvironmentCallback(const std_msgs::EmptyConstPtr& msg) {
  data_saver_->stopRecording();
  ROS_INFO("Reset environment!");
  quadrotor_common::getParam<int>("curr_goal_idx", curr_goal_index_, 0, pnh_);

  environment_is_set_up_ = false;
  goal_positions_.clear();
  gates_orientations.clear();

  gate_replacer_->replaceGatesExplicit();
  ros::Duration(2.0).sleep();
  goal_positions_ = gate_replacer_->getGoalPositions();
  gazebo_rviz_visualizer_->visualizeGates();
  global_trajectory_->generateGlobalTrajectory(visualizer_, goal_positions_);


  quadrotor_common::getParam<double>("/DAgger_threshold", DAgger_threshold_, 1.5, pnh_);
  quadrotor_common::getParam<double>("max_velocity", max_velocity_, 1.5, pnh_);


  gates_orientations = gate_replacer_->getGateOrientations();


  ros::param::get("/reverse_track", reverseTrack);
  if (reverseTrack == true) {
    float roll = 0, pitch = 0, yaw = 90.0/180.0*3.1416;  
    // float roll = 0, pitch = 0, yaw = gates_orientations[curr_goal_index_];
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    geometry_msgs::Pose start_pose;
    // start_pose.position.x = goal_positions_.at(0).x();
    // start_pose.position.y = goal_positions_.at(0).y();
    // start_pose.position.z = goal_positions_.at(0).z();

    start_pose.position.x = (goal_positions_.at(0).x() + goal_positions_.at(goal_positions_.size()-1).x()) / 2;
    start_pose.position.y = (goal_positions_.at(0).y() + goal_positions_.at(goal_positions_.size()-1).y()) / 2;
    start_pose.position.z = (goal_positions_.at(0).z() + goal_positions_.at(goal_positions_.size()-1).z()) / 2;

    start_pose.orientation.x = q.coeffs()[0];
    start_pose.orientation.y = q.coeffs()[1];
    start_pose.orientation.z = q.coeffs()[2];
    start_pose.orientation.w = q.coeffs()[3];

    geometry_msgs::Twist start_twist;
    start_twist.linear.x = 0.0;
    start_twist.linear.y = 0.0;
    start_twist.linear.z = 0.0;
    start_twist.angular.x = 0.0;
    start_twist.angular.y = 0.0;
    start_twist.angular.z = 0.0;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = (std::string) "hummingbird";
    modelstate.reference_frame = (std::string) "world";
    modelstate.pose = start_pose;
    modelstate.twist = start_twist;

    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;
    bool call_service = state_client.call(setmodelstate);
    if (call_service == true) {
      printf("\n!!!!!!!!!!!Right Circle!!!!!!!!!!!!!!!!!!!\n");
    }   
  }
  else {
    float roll = 0, pitch = 0, yaw = -90.0/180.0*3.1416;  
    // float roll = 0, pitch = 0, yaw = gates_orientations[curr_goal_index_];
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    geometry_msgs::Pose start_pose;
    start_pose.position.x = (goal_positions_.at(0).x() + goal_positions_.at(goal_positions_.size()-1).x()) / 2;
    start_pose.position.y = (goal_positions_.at(0).y() + goal_positions_.at(goal_positions_.size()-1).y()) / 2;
    start_pose.position.z = (goal_positions_.at(0).z() + goal_positions_.at(goal_positions_.size()-1).z()) / 2;

    // start_pose.position.x = goal_positions_.at(0).x();
    // start_pose.position.y = goal_positions_.at(0).y();
    // start_pose.position.z = goal_positions_.at(0).z();

    start_pose.orientation.x = q.coeffs()[0];
    start_pose.orientation.y = q.coeffs()[1];
    start_pose.orientation.z = q.coeffs()[2];
    start_pose.orientation.w = q.coeffs()[3];

    geometry_msgs::Twist start_twist;
    start_twist.linear.x = 0.0;
    start_twist.linear.y = 0.0;
    start_twist.linear.z = 0.0;
    start_twist.angular.x = 0.0;
    start_twist.angular.y = 0.0;
    start_twist.angular.z = 0.0;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = (std::string) "hummingbird";
    modelstate.reference_frame = (std::string) "world";
    modelstate.pose = start_pose;
    modelstate.twist = start_twist;

    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;
    bool call_service = state_client.call(setmodelstate);
    if (call_service == true) {
      printf("\n!!!!!!!!!!!Left Circle!!!!!!!!!!!!!!!!!!!\n");
    }   
  }



  goal_selected_ = false;
  state_machine_ = State::kHover;
  resetDesiredState();  
  // ros::Duration(0.3).sleep();
  publishDesiredState();

  ros::Duration(0.2).sleep();
  if (test_time_) {
    getEstimatedPos();
  } else {
    global_trajectory_->findStartIndexGlobalTrajectory(getEstimatedPos());
  }

  // ros::Duration(0.2).sleep();
  resetDesiredState();
  publishDesiredState();

  findInitGateIndex();

  last_goal_index_ = (curr_goal_index_ + goal_positions_.size() - 1) % goal_positions_.size();
  environment_is_set_up_ = true;
  goal_selected_ = true;


  ROS_INFO("Done resetting environment!");
  start_time_ = ros::WallTime::now();
  init_finished = false;


}

void DroneRacing::setRunIdxCallback(const std_msgs::Int16ConstPtr& msg) {
  directory_created_ = false;
}

void DroneRacing::mainloop(const ros::TimerEvent& time) {
  visualizer_->displayQuadrotor();
  if (environment_is_set_up_) {
    if (moving_gates_ && (ros::WallTime::now() - start_time_).toSec() > sec_no_record_at_start_) {
      gate_replacer_->makeGatesMove();
    }
    gazebo_rviz_visualizer_->visualizeGates();
    ros::WallTime start_mainloop = ros::WallTime::now();
    updateStateMachine();
    switch (state_machine_) {
      case State::kOff:
      case State::kHover:
      case State::kWaitingForFeedthroughActivated:
        // Desired state does not change so there is nothing to be published
        break;
      case State::kRacing: {
        desired_state_world_ = trajectory_manager_->getDesiredStateFromTrajectory();
        transformToQuadFrame();
        publishDesiredState();
        break;
      }
    }
    if ((ros::WallTime::now() - start_mainloop).toSec() > 0.02) {
      ROS_ERROR("[%s] Mainloop iteration took [%f] sec, state_machine_ = [%d]",
                ros::this_node::getName().c_str(), (ros::WallTime::now() - start_mainloop).toSec(),
                static_cast<int>(state_machine_));
    }
  }
}

void DroneRacing::updateStateMachine() {
  if (state_machine_ == State::kOff || state_machine_ == State::kHover) {
    visualizer_->displayGoalMarker(goal_positions_);
    // Switch to feedtrough if goal was selected
    if (setGoalWasTriggered()) {
      // ros::Duration(5.0).sleep();
      global_trajectory_->findStartIndexGlobalTrajectory(getEstimatedPos());
      global_trajectory_->findStartIndexNWGlobalTrajectory(this);
      ROS_INFO("[%s] Entering racing mode.", ros::this_node::getName().c_str());
      mainloop_iter_ = 0;
      prev_des_velocity_ = 0.0;
      state_machine_ = State::kRacing;
    }
  }

  if (state_machine_ == State::kRacing) {
    performNavigation();
    mainloop_iter_++;
  }
}

quadrotor_common::TrajectoryPoint DroneRacing::getEndState() {
  quadrotor_common::TrajectoryPoint end_state;

  double dist_to_next_gate = (desired_state_world_.position - goal_positions_.at(curr_goal_index_)).norm();
  double dist_to_last_gate = (desired_state_world_.position - goal_positions_.at(last_goal_index_)).norm();
  double horizon = std::max(horizon_min_, std::min(dist_to_next_gate, dist_to_last_gate));
  horizon = std::min(horizon_max_, horizon);

  end_state = global_trajectory_->getNextStateOnGlobalTraj(horizon);

  visualizer_->displayDebug(end_state.position.x(),
                            end_state.position.y(),
                            end_state.position.z(),
                            0,
                            visualizer_->Color::kYellow);

  return end_state;
}

void DroneRacing::performNavigation() {
  if (checkStateEstDesStateDivergence()) {
    data_saver_->stopRecording();
    ROS_WARN("[%s] Desired state and state estimate diverged.", ros::this_node::getName().c_str());
    ROS_WARN("Desired state: [%f, %f, %f]", desired_state_world_.position.x(),
             desired_state_world_.position.y(), desired_state_world_.position.z());
    ROS_WARN("State estimate: [%f, %f, %f]", state_estimate_world_.pose.pose.position.x,
             state_estimate_world_.pose.pose.position.y, state_estimate_world_.pose.pose.position.z);
    goal_selected_ = false;
    resetDesiredState();
    state_machine_ = State::kHover;
    publishDesiredState();
    data_saver_->labelAsBadData();
    crashed_pub_.publish(std_msgs::Empty());
    return;
  }

  if (test_time_) {
    executeTestTime();
  } else {
    collectData();
  }
}


// final version
void DroneRacing::collectData() {
  // only compute trajectories every n-th iteration of the mainloop
  quadrotor_common::TrajectoryPoint end_state;
  quadrotor_common::TrajectoryPoint next_goal;
  if (mainloop_iter_ % plan_every_n_iter_ == 0) {
    updateWaypointGoals();

    // final version 
    nav_msgs::Odometry state_estimate_world_current_ = state_estimate_world_;

    double desired_velocity_mb = getDesiredVelocity(); // normalized with max global velocity; [0,1]
    end_state = getEndState();
    next_goal.position = goal_positions_.at(curr_goal_index_);

    // log data
    // get vector from start_state to end_state
    Eigen::Vector3d goal_w = end_state.position;
    rpg::Pose T_W_goal;
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = goal_w.x();
    goal_pose.position.y = goal_w.y();
    goal_pose.position.z = goal_w.z();
    goal_pose.orientation.w = 1.0;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    tf::poseMsgToKindr(goal_pose, &T_W_goal);

    // transform vector to body frame of quad
    rpg::Pose T_W_B;
    geometry_msgs::Pose state_estimate_pose;
    state_estimate_pose.position.x = state_estimate_world_.pose.pose.position.x;
    state_estimate_pose.position.y = state_estimate_world_.pose.pose.position.y;
    state_estimate_pose.position.z = state_estimate_world_.pose.pose.position.z;
    state_estimate_pose.orientation = state_estimate_world_.pose.pose.orientation;
    tf::poseMsgToKindr(state_estimate_pose, &T_W_B);

    rpg::Pose T_B_goal = T_W_B.inverse() * T_W_goal;

    double yaw_to_end = std::atan2(T_B_goal.getPosition().y(),
                                   T_B_goal.getPosition().x());

    double traj_coor_right = -yaw_to_end / camera_semi_fov_rad_yaw_;
    traj_coor_right = std::min(1.0, std::max(-1.0, traj_coor_right));

    double dist_to_goal = T_B_goal.getPosition().norm();
    double pitch_to_end = std::atan2(T_B_goal.getPosition().z(),
                                     dist_to_goal);

    pitch_to_end = pitch_to_end / camera_semi_fov_rad_pitch_;

    double traj_coor_up = pitch_to_end;
    traj_coor_up = std::min(1.0, std::max(-1.0, traj_coor_up));

    // prepare trajectory label for later saving
    DataSaver::trajectory_info traj_info;
    traj_info.idx_right = traj_coor_right;
    traj_info.idx_up = traj_coor_up;
    traj_info.curr_vel = desired_velocity_mb;
    traj_info.max_vel = global_trajectory_->getMaxVelocity();
    data_saver_->setModelBasedPrediction(Eigen::Vector3d(traj_info.idx_right,
                                                         traj_info.idx_up,
                                                         traj_info.curr_vel));


    // get vector from start_state to end_state
    Eigen::Vector3d goal_w2 = next_goal.position;
    rpg::Pose T_W_goal2;
    geometry_msgs::Pose goal_pose2;
    goal_pose2.position.x = goal_w2.x();
    goal_pose2.position.y = goal_w2.y();
    goal_pose2.position.z = goal_w2.z();
    goal_pose2.orientation.w = 1.0;
    goal_pose2.orientation.x = 0.0;
    goal_pose2.orientation.y = 0.0;
    goal_pose2.orientation.z = 0.0;
    tf::poseMsgToKindr(goal_pose2, &T_W_goal2);

    // transform vector to body frame of quad
    rpg::Pose T_W_B2;
    geometry_msgs::Pose state_estimate_pose2;
    state_estimate_pose2.position.x = state_estimate_world_current_.pose.pose.position.x;
    state_estimate_pose2.position.y = state_estimate_world_current_.pose.pose.position.y;
    state_estimate_pose2.position.z = state_estimate_world_current_.pose.pose.position.z;
    state_estimate_pose2.orientation = state_estimate_world_current_.pose.pose.orientation;
    tf::poseMsgToKindr(state_estimate_pose2, &T_W_B2);

    rpg::Pose T_B_goal2 = T_W_B2.inverse() * T_W_goal2;

    double yaw_to_end2 = std::atan2(T_B_goal2.getPosition().y(),
                                   T_B_goal2.getPosition().x());

    double traj_coor_right2 = -yaw_to_end2 / camera_semi_fov_rad_yaw_;
    traj_coor_right2 = std::min(1.0, std::max(-1.0, traj_coor_right2));

    double dist_to_goal2 = T_B_goal2.getPosition().norm();
    double pitch_to_end2 = std::atan2(T_B_goal2.getPosition().z(),
                                     dist_to_goal2);

    pitch_to_end2 = pitch_to_end2 / camera_semi_fov_rad_pitch_;

    double traj_coor_up2 = pitch_to_end2;
    traj_coor_up2 = std::min(1.0, std::max(-1.0, traj_coor_up2));

    // prepare trajectory label for later saving
    DataSaver::trajectory_info traj_info2;
    traj_info2.idx_right = traj_coor_right2;
    traj_info2.idx_up = traj_coor_up2;

    Eigen::Vector3d dist_gate_vec;
    dist_gate_vec = Eigen::Vector3d(goal_positions_.at(curr_goal_index_).x() - state_estimate_world_current_.pose.pose.position.x,
                                    goal_positions_.at(curr_goal_index_).y() - state_estimate_world_current_.pose.pose.position.y,
                                    goal_positions_.at(curr_goal_index_).z() - state_estimate_world_current_.pose.pose.position.z);
    double dist_to_gate = dist_gate_vec.norm();

    if (dist_to_gate > 10) {
      dist_to_gate = 10;
    }
    dist_to_gate /= 10;

    bool save_data = false;
    if (dist_to_gate > 0.1) {
      save_data = true;
    }

    // std::cout << dist_to_gate << std::endl;

    double x_angle = 0 - (double) quadrotor_common::quaternionToEulerAnglesZYX(quadrotor_common::geometryToEigen(state_estimate_world_current_.pose.pose.orientation)).x();
    double y_angle = 0 - (double) quadrotor_common::quaternionToEulerAnglesZYX(quadrotor_common::geometryToEigen(state_estimate_world_current_.pose.pose.orientation)).y();
    double z_angle = gates_orientations[curr_goal_index_] - (double) quadrotor_common::quaternionToEulerAnglesZYX(quadrotor_common::geometryToEigen(state_estimate_world_current_.pose.pose.orientation)).z();

    x_angle = x_angle/3.1415926 * 180;
    y_angle = y_angle/3.1415926 * 180;
    z_angle = z_angle/3.1415926 * 180;

    ros::param::get("/reverse_track", reverseTrack);
    if (reverseTrack) {
      z_angle -= 180;
    }

    if (z_angle < -180.00) {
      z_angle += 360;
    }
    else if (z_angle > 180.00) {
      z_angle -= 360;
    }


    if (x_angle < -90 || x_angle > 90 || y_angle < -90 || y_angle > 90 || z_angle < -90 || z_angle > 90 ||
        traj_info2.idx_right < -0.95 || traj_info2.idx_right > 0.95 || traj_info2.idx_up < -0.95 || traj_info2.idx_up > 0.95) {
      x_angle = 90;
      y_angle = 90;
      z_angle = 90;
      traj_info2.idx_right = 1.0;
      traj_info2.idx_up = 1.0;
      dist_to_gate = 0.0;
    }

    x_angle /= 90.0;
    y_angle /= 90.0; 
    z_angle /= 90.0; 


    data_saver_->setGoalPosition(Eigen::Vector2d(traj_info2.idx_right,
                                                         traj_info2.idx_up));



    if (save_data) {
      data_saver_->saveImage(traj_info, model_based_frame_, state_estimate_quad_prev_prev_, state_estimate_quad_prev_, state_estimate_quad_, traj_info2, x_angle, y_angle, z_angle, dist_to_gate);
    }
    
    bool success;

    quadrotor_common::TrajectoryPoint des_state_proj = global_trajectory_->projectOnTrajectory(desired_state_world_);
    double divergence = (desired_state_world_.position - des_state_proj.position).norm();

    if (use_DAgger_  && divergence < DAgger_threshold_){
      if (init_finished == false) {
        asked_teacher_n_times_ = 0;
      }
      init_finished = true;

      getEstimatedPos();
      // generate network trajectory
      success = trajectory_manager_->computeTrajectory(this,
                                                              desired_state_world_,
                                                              state_estimate_world_,
                                                              network_selection_, false);

      if (success) {
        trajectory_manager_->setBestTraj(0, false);
        model_based_frame_ = 0;
        executed_nn_traj_++;
      }
    }
    else if (perturb_actions_) {
      Eigen::Vector3d random_perturbation = 2 * Eigen::Vector3d(0.1 * std::sin(0.5 * ros::WallTime::now().toSec()),
                                                            0.1 * std::sin(0.5 * ros::WallTime::now().toSec()),
                                                            0.05 * std::sin(0.5 * ros::WallTime::now().toSec()));
      success = trajectory_manager_->computeTrajectory(this, desired_state_world_, state_estimate_world_,
                                                       Eigen::Vector3d(traj_info.idx_right,
                                                                       traj_info.idx_up,
                                                                       traj_info.curr_vel) +
                                                       random_perturbation, true);
      if (success) {
        trajectory_manager_->setBestTraj(0,true);
        asked_teacher_n_times_++;
        model_based_frame_++;
      }                                                   
    } else {
      success = trajectory_manager_->computeTrajectory(this, desired_state_world_, state_estimate_world_,
                                                       Eigen::Vector3d(traj_info.idx_right,
                                                                       traj_info.idx_up,
                                                                       traj_info.curr_vel), true);
      if (success) {
        trajectory_manager_->setBestTraj(0,true);
        asked_teacher_n_times_++;
        model_based_frame_++;
      }
    }



    // start recording data after delay, do only if using model based
    if ((ros::WallTime::now() - start_time_).toSec() > sec_no_record_at_start_) {
      if (record_data_) {
        if (!directory_created_) {
          data_saver_->createDirectory();
          directory_created_ = true;
        }
        data_saver_->startRecording();
      }
    }


    // visualization
    if (success) {
      trajectory_manager_->visualizeTrajectory(this);
      failed_trials_ = 0;
      trajectory_manager_->setStartTime();
    } else {
      if (failed_trials_ < max_failed_trials_) {
        failed_trials_++;
        ROS_WARN("[%s] Failed trials: [%d].", ros::this_node::getName().c_str(), failed_trials_);
      } else {
        data_saver_->stopRecording();
        ROS_INFO("[%s] Going to Hover state.", ros::this_node::getName().c_str());
        resetDesiredState();
        crashed_pub_.publish(std_msgs::Empty());
        state_machine_ = State::kHover;
        goal_selected_ = false;
        publishDesiredState();
      }
    }

  }
}


void DroneRacing::resetDesiredState() {
  desired_state_quad_ = quadrotor_common::TrajectoryPoint();
  desired_state_quad_.time_from_start = ros::Duration(0.0); //ros::Time::now();
  desired_state_quad_.position = quadrotor_common::geometryToEigen(state_estimate_quad_.pose.pose.position);
  desired_state_quad_.velocity = Eigen::Vector3d::Zero();
  desired_state_quad_.acceleration = Eigen::Vector3d::Zero();
  desired_state_quad_.jerk = Eigen::Vector3d::Zero();
  desired_state_quad_.heading = (quadrotor_common::quaternionToEulerAnglesZYX(
      quadrotor_common::geometryToEigen(state_estimate_quad_.pose.pose.orientation))).z();

  transformToWorldFrame();
}

void DroneRacing::startNavigationCallback(const std_msgs::EmptyConstPtr& msg) {
  goal_selected_ = true;
  start_time_ = ros::WallTime::now();
  // publish message such that copilot will go to feedthrough
  resetDesiredState();
  publishDesiredState();
  std_msgs::Bool feedthrough_msg;
  feedthrough_msg.data = true;
  feedthrough_pub_.publish(feedthrough_msg);
}

void DroneRacing::hardStopCallback(const std_msgs::EmptyConstPtr& msg) {
  data_saver_->stopRecording();
  ROS_INFO("Asked teacher [%d] times for help and executed [%d] times NN selection.", asked_teacher_n_times_,
           executed_nn_traj_);

  // if (asked_teacher_n_times_ < 60 && executed_nn_traj_ > 300) {
  //   std::cout << "!!!!!!!!!!!!!!!! Expand DAgger_threshold !!!!!!!!!!!!!!!!!!\n"; 
  //   std_msgs::Bool DAgger_expand_msg;
  //   DAgger_expand_msg.data = true;
  //   DAgger_expand_pub_.publish(DAgger_expand_msg);
  // }
  
  asked_teacher_n_times_ = 0;
  executed_nn_traj_ = 0;
  model_based_frame_ = 0;
  goal_selected_ = false;
  resetDesiredState();
  state_machine_ = State::kHover;
  publishDesiredState();

}

void DroneRacing::stateEstimateCallback(const nav_msgs::OdometryConstPtr& msg) {
  state_estimate_quad_prev_prev_ = state_estimate_quad_prev_;
  state_estimate_quad_prev_ = state_estimate_quad_;
  state_estimate_quad_ = *msg;

  geometry_msgs::Pose pose_S_Q;
  pose_S_Q.position.x = state_estimate_quad_.pose.pose.position.x;
  pose_S_Q.position.y = state_estimate_quad_.pose.pose.position.y;
  pose_S_Q.position.z = state_estimate_quad_.pose.pose.position.z;
  pose_S_Q.orientation.w = state_estimate_quad_.pose.pose.orientation.w;
  pose_S_Q.orientation.x = state_estimate_quad_.pose.pose.orientation.x;
  pose_S_Q.orientation.y = state_estimate_quad_.pose.pose.orientation.y;
  pose_S_Q.orientation.z = state_estimate_quad_.pose.pose.orientation.z;
  tf::poseMsgToKindr(pose_S_Q, &T_S_Q_);


  // transform state estimate to world frame
  transformStateEstimateToWorldFrame();
  
}

void DroneRacing::networkCallback(const geometry_msgs::TwistStampedConstPtr& msg) {
  network_selection_ = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  data_saver_->setNetworkSelection(Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z));
}

void DroneRacing::publishDesiredState() {
  desired_state_pub_.publish(desired_state_quad_.toRosMessage());
}

bool DroneRacing::setGoalWasTriggered() const {
  return goal_selected_;
}

bool DroneRacing::checkStateEstDesStateDivergence() {
  Eigen::Vector3d divergence_vec;
  divergence_vec = Eigen::Vector3d(desired_state_world_.position.x() - state_estimate_world_.pose.pose.position.x,
                                   desired_state_world_.position.y() - state_estimate_world_.pose.pose.position.y,
                                   desired_state_world_.position.z() - state_estimate_world_.pose.pose.position.z);

                    

  divergence_pub_.publish(divergence_vec.norm());
  return (divergence_vec.norm() > max_divergence_);
}

Eigen::Vector3d DroneRacing::getEstimatedPos() {
  geometry_msgs::Pose pose_S_Q;
  pose_S_Q.position.x = state_estimate_quad_.pose.pose.position.x;
  pose_S_Q.position.y = state_estimate_quad_.pose.pose.position.y;
  pose_S_Q.position.z = state_estimate_quad_.pose.pose.position.z;
  pose_S_Q.orientation.w = state_estimate_quad_.pose.pose.orientation.w;
  pose_S_Q.orientation.x = state_estimate_quad_.pose.pose.orientation.x;
  pose_S_Q.orientation.y = state_estimate_quad_.pose.pose.orientation.y;
  pose_S_Q.orientation.z = state_estimate_quad_.pose.pose.orientation.z;
  tf::poseMsgToKindr(pose_S_Q, &T_S_Q_);

  geometry_msgs::Pose pose_W_S;
  pose_W_S.position.x = 0.0;
  pose_W_S.position.y = 0.0;
  pose_W_S.position.z = 0.0;
  pose_W_S.orientation.w = std::cos(0.0 / 2);
  pose_W_S.orientation.x = 0.0;
  pose_W_S.orientation.y = 0.0;
  pose_W_S.orientation.z = std::sin(0.0 / 2);
  tf::poseMsgToKindr(pose_W_S, &T_W_S_);

  T_W_Q_ = T_W_S_ * T_S_Q_;
  Eigen::Vector3d est_pos;
  est_pos = T_W_Q_.getPosition();

  visualizer_->displayDebug(est_pos.x(),
                            est_pos.y(),
                            est_pos.z(),
                            4,
                            visualizer_->Color::kGreen);

  return est_pos;
}

void DroneRacing::transformToQuadFrame() {
  // transform desired_state_world_ to desired_state_quad_
  Eigen::Quaterniond q_W_O = T_W_S_.getEigenQuaternion();
  const Eigen::Vector3d t_W_O = T_W_S_.getPosition();
  double scale_W_O = 1.0;
  desired_state_quad_ = desired_state_world_;
  desired_state_quad_.position = 1.0 / scale_W_O * (q_W_O.inverse() * (desired_state_world_.position - t_W_O));
  desired_state_quad_.velocity = 1.0 / scale_W_O * (q_W_O.inverse() * desired_state_world_.velocity);
  desired_state_quad_.acceleration = 1.0 / scale_W_O * (q_W_O.inverse() * desired_state_world_.acceleration);
  desired_state_quad_.jerk = 1.0 / scale_W_O * (q_W_O.inverse() * desired_state_world_.jerk);
  desired_state_quad_.snap = 1.0 / scale_W_O * (q_W_O.inverse() * desired_state_world_.snap);
  desired_state_quad_.heading =
      quadrotor_common::wrapMinusPiToPi(
          desired_state_world_.heading - quadrotor_common::quaternionToEulerAnglesZYX(q_W_O).z());
}

void DroneRacing::transformToWorldFrame() {
  // transform desired_state_quad_ to desired_state_world_
  Eigen::Quaterniond q_W_O = T_W_S_.getEigenQuaternion();
  const Eigen::Vector3d t_W_O = T_W_S_.getPosition();
  double scale_W_O = 1.0;
  desired_state_world_ = desired_state_quad_;
  desired_state_world_.position = 1.0 / scale_W_O * ((q_W_O * desired_state_quad_.position) + t_W_O);
  desired_state_world_.velocity = 1.0 / scale_W_O * (q_W_O * desired_state_quad_.velocity);
  desired_state_world_.acceleration = 1.0 / scale_W_O * (q_W_O * desired_state_quad_.acceleration);
  desired_state_world_.jerk = 1.0 / scale_W_O * (q_W_O * desired_state_quad_.jerk);
  desired_state_world_.snap = 1.0 / scale_W_O * (q_W_O * desired_state_quad_.snap);
  desired_state_world_.heading =
      quadrotor_common::wrapMinusPiToPi(
          desired_state_quad_.heading + quadrotor_common::quaternionToEulerAnglesZYX(q_W_O).z());
}

void DroneRacing::transformStateEstimateToWorldFrame() {
  // transform desired_state_quad_ to desired_state_world_
  Eigen::Quaterniond q_W_O = T_W_S_.getEigenQuaternion();
  const Eigen::Vector3d t_W_O = T_W_S_.getPosition();
  double scale_W_O = 1.0;
  state_estimate_world_.pose.pose.position = quadrotor_common::vectorToPoint(quadrotor_common::eigenToGeometry(
      1.0 / scale_W_O
      * ((q_W_O * quadrotor_common::geometryToEigen(state_estimate_quad_.pose.pose.position)) + t_W_O)));
  state_estimate_world_.twist.twist.linear = quadrotor_common::eigenToGeometry(
      1.0 / scale_W_O * (q_W_O * (quadrotor_common::geometryToEigen(state_estimate_quad_.twist.twist.linear))));
  state_estimate_world_.pose.pose.orientation =
      quadrotor_common::eigenToGeometry((q_W_O
                                         * quadrotor_common::geometryToEigen(
          state_estimate_quad_.pose.pose.orientation)));
}

double DroneRacing::getDesiredVelocity() {
  quadrotor_common::TrajectoryPoint next_des_state_mb = global_trajectory_->getNextStateOnGlobalTraj(0.5);
  double max_velocity_global = global_trajectory_->getMaxVelocity();
  double desired_velocity_mb = next_des_state_mb.velocity.norm() / max_velocity_global;
  return desired_velocity_mb;
}

void DroneRacing::findInitGateIndex() {
  global_trajectory_->projectOnTrajectory(desired_state_world_);
  int current_traj_idx = global_trajectory_->global_traj_idx_proj_;
  int gate_traj_idx[goal_positions_.size()];
  double closest_dist = 1e6;
  quadrotor_common::TrajectoryPoint temp_traj_pt = desired_state_world_;
  for(int i=0; i<goal_positions_.size(); i++){
    temp_traj_pt.position = goal_positions_.at(i);
    global_trajectory_->projectOnTrajectory(temp_traj_pt);
    gate_traj_idx[i] = global_trajectory_->global_traj_idx_proj_;
  }

  if (current_traj_idx >= gate_traj_idx[goal_positions_.size()-1]) {
    curr_goal_index_ = 0;
  }
  else{
    for(int i=0; i<goal_positions_.size(); i++){
      if (gate_traj_idx[i] > current_traj_idx){
        curr_goal_index_ = i;
        break;
      }
    }
  }
  global_trajectory_->projectOnTrajectory(desired_state_world_);
  curr_goal_index_ = curr_goal_index_ % goal_positions_.size();
  last_goal_index_ = (curr_goal_index_ + goal_positions_.size() - 1) % goal_positions_.size();
}

void DroneRacing::updateWaypointGoals() {
  visualizer_->displayDebug(desired_state_world_.position.x(),
                            desired_state_world_.position.y(),
                            desired_state_world_.position.z(),
                            3,
                            visualizer_->Color::kRed);

  quadrotor_common::TrajectoryPoint des_state_proj = global_trajectory_->projectOnTrajectory(desired_state_world_);
  visualizer_->displayDebug(des_state_proj.position.x(),
                            des_state_proj.position.y(),
                            des_state_proj.position.z(),
                            1,
                            visualizer_->Color::kBlue);

  
  findInitGateIndex();

  quadrotor_common::TrajectoryPoint next_des_state_mb = global_trajectory_->getNextStateOnGlobalTraj(0.5);


  visualizer_->displayDebug(goal_positions_.at(curr_goal_index_).x(),
                            goal_positions_.at(curr_goal_index_).y(),
                            goal_positions_.at(curr_goal_index_).z(),
                            27,
                            visualizer_->Color::kBlack);

  Eigen::Vector3d dist_to_next_gate;
  dist_to_next_gate = desired_state_world_.position - Eigen::Vector3d(goal_positions_.at(curr_goal_index_).x(),
                                                                      goal_positions_.at(curr_goal_index_).y(),
                                                                      goal_positions_.at(curr_goal_index_).z());

  if (dist_to_next_gate.norm() < d_replan_ && passed_gate == false) {
    passed_gate_pub_.publish(std_msgs::Empty());
    passed_gate = true;
  }
  if (dist_to_next_gate.norm() >= d_replan_) {
    passed_gate = false;
  }
}

void DroneRacing::enableTestTimeCallback(const std_msgs::BoolConstPtr& msg) {
  test_time_ = msg->data;
  if (test_time_) {
    ROS_INFO("[%s] Enabled test time execution.", ros::this_node::getName().c_str());
  }
}

double DroneRacing::constrain(const double& prev_value, const double& new_value, const double& threshold) {
  if (new_value > prev_value) {
    return std::min(new_value, prev_value + threshold);
  } else {
    return std::max(new_value, prev_value - threshold);
  }
}

void DroneRacing::executeTestTime() {
  // only compute trajectories every n-th iteration of the mainloop
  quadrotor_common::TrajectoryPoint end_state;
  if (mainloop_iter_ % plan_every_n_iter_ == 0) {
    updateWaypointGoals();
    // generate network trajectory
    bool success_nw = trajectory_manager_->computeTrajectory(this,
                                                             desired_state_world_,
                                                             state_estimate_world_,
                                                             network_selection_,false);
    // updateWaypointGoals();

    if (success_nw) {
      trajectory_manager_->setBestTraj(0, false);
      model_based_frame_ = 0;
      executed_nn_traj_++;
    }

    // // clear all generated trajectories (in theory not needed...)
    // trajectory_manager_->clearTrajectories(true);
    // trajectory_manager_->clearTrajectories(false);

    // visualization
    if (success_nw) {
      trajectory_manager_->visualizeTrajectory(this);
      failed_trials_ = 0;
      trajectory_manager_->setStartTime();
    } else {
      if (failed_trials_ < max_failed_trials_) {
        failed_trials_++;
        ROS_WARN("[%s] Failed trials: [%d].", ros::this_node::getName().c_str(), failed_trials_);
      } else {
        ROS_INFO("[%s] Going to Hover state.", ros::this_node::getName().c_str());
        resetDesiredState();

        state_machine_ = State::kHover;
        goal_selected_ = false;
        publishDesiredState();

        crashed_pub_.publish(std_msgs::Empty());
      }
    }
  }
}

void DroneRacing::loadParameters() {
  double camera_fov_deg_yaw;
  double camera_fov_deg_pitch;
  // General
  quadrotor_common::getParam<double>("trajectory_max_z", traj_max_z_, 2.5, pnh_);
  quadrotor_common::getParam<int>("plan_every_nth_iteration", plan_every_n_iter_, 1, pnh_);
  quadrotor_common::getParam<double>("max_velocity", max_velocity_, 1.0, pnh_);
  quadrotor_common::getParam<double>("min_velocity", min_velocity_, 1.0, pnh_);
  quadrotor_common::getParam<double>("max_divergence", max_divergence_, 0.2, pnh_);
  quadrotor_common::getParam<double>("max_error_to_global_traj_start", max_error_to_global_traj_start_, 1.0, pnh_);
  quadrotor_common::getParam<double>("max_error_to_global_traj_end", max_error_to_global_traj_end_, 1.0, pnh_);
  quadrotor_common::getParam<double>("d_replan", d_replan_, 1.0, pnh_);
  quadrotor_common::getParam<double>("camera_fov_yaw", camera_fov_deg_yaw, 45.0, pnh_);
  quadrotor_common::getParam<double>("camera_fov_pitch", camera_fov_deg_pitch, 45.0, pnh_);
  quadrotor_common::getParam<bool>("moving_gates", moving_gates_, false, pnh_);
  quadrotor_common::getParam<double>("gates_static_amplitude", gates_static_amp_, 0.0, pnh_);
  quadrotor_common::getParam<int>("max_failed_trials", max_failed_trials_, 5, pnh_);
  quadrotor_common::getParam<double>("wait_n_sec_till_record", sec_no_record_at_start_, 0.0, pnh_);
  quadrotor_common::getParam<bool>("record_data", record_data_, false, pnh_);
  quadrotor_common::getParam<bool>("perturb_actions", perturb_actions_, false, pnh_);
  quadrotor_common::getParam<std::string>("quad_frame", quad_frame_, "none", pnh_);
  quadrotor_common::getParam<int>("curr_goal_idx", curr_goal_index_, 0, pnh_);
  quadrotor_common::getParam<double>("gate_height", gate_height_, 1.0, pnh_);
  quadrotor_common::getParam<double>("horizon_min", horizon_min_, 1.0, pnh_);
  quadrotor_common::getParam<double>("horizon_max", horizon_max_, 10.0, pnh_);

  quadrotor_common::getParam<bool>("use_DAgger", use_DAgger_, false, pnh_);


  

  camera_semi_fov_rad_yaw_ = camera_fov_deg_yaw * (M_PI / 180.0);
  camera_semi_fov_rad_pitch_ = camera_fov_deg_pitch * (M_PI / 180.0);
}

} // namespace drone_racing
