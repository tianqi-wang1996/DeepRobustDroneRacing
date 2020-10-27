#pragma once

#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <ros/ros.h>

#include <mav_msgs/Actuators.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>

// final version
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/geometry_eigen_conversions.h>

# ifndef M_PI
#  define M_PI 3.14159265358979323846
# endif

namespace drone_racing {
class DataSaver {
 public:
  DataSaver(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

  DataSaver() :
      DataSaver(ros::NodeHandle(), ros::NodeHandle("~")) {
  }

  virtual ~DataSaver();

  struct trajectory_info {
    double idx_right;
    double idx_up;
    double curr_vel;
    double max_vel;
  };

  void saveImage(const DataSaver::trajectory_info traj_coor,
                          const int model_based_frame, nav_msgs::Odometry state_estimate_prev_prev,
                          nav_msgs::Odometry state_estimate_prev, nav_msgs::Odometry state_estimate, const DataSaver::trajectory_info goal_coor,
                          double roll, double pitch, double yaw, double dist_gate);

  void labelAsBadData();

  bool createDirectory();

  void setRunIdxCallback(const std_msgs::Int16ConstPtr &msg);

  void setModelBasedPrediction(Eigen::Vector3d prediction);
  void setNetworkSelection(Eigen::Vector3d selection);
  
  // final version
  void setGoalPosition(Eigen::Vector2d goal_selection);

  void startRecording();
  void stopRecording();

  bool record_data_ = false;

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  void stateEstimateCallback(const nav_msgs::OdometryConstPtr &msg);

  void addPredictionAndRepublish();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber state_estimate_sub_;
  ros::Subscriber motor_speed_sub_;
  ros::Subscriber control_command_sub_;
  ros::Subscriber cam_info_sub_;

  ros::Subscriber run_idx_sub_;
  ros::Subscriber image_sub_;

  ros::Publisher image_pub_;

  std::mutex mtx_image_save_;

  void loadParameters();

  nav_msgs::Odometry state_estimate_;
  mav_msgs::Actuators motor_speed_;
  quadrotor_msgs::ControlCommand control_command_;
  cv::Mat image_;

  int w_;
  int h_;
  Eigen::Matrix<double, 3, 3> K_;
  std::string cam_frame_;

  Eigen::Vector3d model_based_prediction_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d network_selection_ = Eigen::Vector3d::Zero();

  Eigen::Vector2d goal_position_ = Eigen::Vector2d::Zero();

  int run_idx_ = 8000;
  int frame_counter_ = 0;

  std::string root_dir_;
  double gate_x_;
  double gate_y_;
  float gate_rotation_;
  bool created_directory_ = false;

  double max_velocity_;
  int env_idx_;
  double camera_fov_deg_yaw_;
  double camera_fov_deg_pitch_;
  double last_dist_gate = 0;
};

} /* namespace drone_racing */
