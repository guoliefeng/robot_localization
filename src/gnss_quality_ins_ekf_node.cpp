/*
 * Copyright (c) 2026
 * All rights reserved.
 */

#include "robot_localization/gnss_quality_monitor.h"
#include "robot_localization/filter_common.h"
#include "robot_localization/ros_filter_types.h"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <algorithm>
#include <cstdlib>
#include <numeric>
#include <string>
#include <vector>

namespace
{

std::vector<int> makeDefaultInsConfig()
{
  std::vector<int> config(RobotLocalization::STATE_SIZE, 0);
  config[RobotLocalization::StateMemberX] = 1;
  config[RobotLocalization::StateMemberY] = 1;
  config[RobotLocalization::StateMemberYaw] = 1;
  config[RobotLocalization::StateMemberVx] = 1;
  config[RobotLocalization::StateMemberVy] = 1;
  config[RobotLocalization::StateMemberVyaw] = 1;
  return config;
}

int updateSum(const std::vector<int> &update_vector)
{
  return std::accumulate(update_vector.begin(), update_vector.end(), 0);
}

RobotLocalization::CallbackData makeCallbackData(
  const std::string &topic_name,
  const std::vector<int> &update_vector,
  const double rejection_threshold)
{
  return RobotLocalization::CallbackData(
    topic_name, update_vector, updateSum(update_vector), false, false, false, rejection_threshold);
}

class GnssQualityInsEkfNode
{
public:
  GnssQualityInsEkfNode()
    : nh_(),
      nh_priv_("~"),
      ekf_(nh_, nh_priv_, "gnss_quality_ins_ekf_node"),
      ins_queue_size_(100),
      tcp_no_delay_(true),
      ins_pose_cb_data_(makeCallbackData("ins_pose", std::vector<int>(RobotLocalization::STATE_SIZE, 0), 5.0)),
      ins_twist_cb_data_(makeCallbackData("ins_twist", std::vector<int>(RobotLocalization::STATE_SIZE, 0), 3.0))
  {
    loadParams();
    configureInsCallbacks();

    ekf_.initialize();

    gnss_quality_monitor_.initialize(nh_, nh_priv_);

    ins_sub_ = nh_.subscribe(
      ins_input_topic_,
      ins_queue_size_,
      &GnssQualityInsEkfNode::insCallback,
      this,
      ros::TransportHints().tcpNoDelay(tcp_no_delay_));

    ROS_INFO_STREAM("GNSS quality INS EKF node started. input=" << ins_input_topic_);
  }

private:
  void loadParams()
  {
    nh_priv_.param("ins_odom_topic", ins_input_topic_, std::string("/localization/ins"));
    nh_priv_.param("ins_queue_size", ins_queue_size_, 100);
    nh_priv_.param("tcp_no_delay", tcp_no_delay_, true);
    nh_priv_.param("ins_pose_rejection_threshold", ins_pose_rejection_threshold_, 5.0);
    nh_priv_.param("ins_twist_rejection_threshold", ins_twist_rejection_threshold_, 3.0);

    ins_config_ = makeDefaultInsConfig();
    std::vector<int> configured_ins_config;
    if (nh_priv_.getParam("ins_config", configured_ins_config))
    {
      if (configured_ins_config.size() == RobotLocalization::STATE_SIZE)
      {
        ins_config_ = configured_ins_config;
      }
      else
      {
        ROS_WARN_STREAM("ins_config must have " << RobotLocalization::STATE_SIZE <<
                        " entries. Using default INS config.");
      }
    }
  }

  void configureInsCallbacks()
  {
    std::vector<int> pose_update_vector = ins_config_;
    std::fill(
      pose_update_vector.begin() + RobotLocalization::POSITION_V_OFFSET,
      pose_update_vector.begin() + RobotLocalization::POSITION_V_OFFSET + RobotLocalization::TWIST_SIZE,
      0);
    std::fill(
      pose_update_vector.begin() + RobotLocalization::POSITION_A_OFFSET,
      pose_update_vector.begin() + RobotLocalization::POSITION_A_OFFSET + RobotLocalization::ACCELERATION_SIZE,
      0);

    std::vector<int> twist_update_vector = ins_config_;
    std::fill(
      twist_update_vector.begin() + RobotLocalization::POSITION_OFFSET,
      twist_update_vector.begin() + RobotLocalization::POSITION_OFFSET + RobotLocalization::POSE_SIZE,
      0);
    std::fill(
      twist_update_vector.begin() + RobotLocalization::POSITION_A_OFFSET,
      twist_update_vector.begin() + RobotLocalization::POSITION_A_OFFSET + RobotLocalization::ACCELERATION_SIZE,
      0);

    ins_pose_cb_data_ =
      makeCallbackData("ins_pose", pose_update_vector, ins_pose_rejection_threshold_);
    ins_twist_cb_data_ =
      makeCallbackData("ins_twist", twist_update_vector, ins_twist_rejection_threshold_);
  }

  void insCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    nav_msgs::OdometryPtr out(new nav_msgs::Odometry(*msg));
    gnss_quality_monitor_.applyToOdometry(*out);
    ekf_.odometryCallback(out, "ins_odom", ins_pose_cb_data_, ins_twist_cb_data_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  RobotLocalization::RosEkf ekf_;
  RobotLocalization::GnssQualityMonitor gnss_quality_monitor_;

  ros::Subscriber ins_sub_;

  std::string ins_input_topic_;
  int ins_queue_size_;
  bool tcp_no_delay_;
  double ins_pose_rejection_threshold_;
  double ins_twist_rejection_threshold_;
  std::vector<int> ins_config_;
  RobotLocalization::CallbackData ins_pose_cb_data_;
  RobotLocalization::CallbackData ins_twist_cb_data_;
};

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnss_quality_ins_ekf_node");
  GnssQualityInsEkfNode node;
  ros::spin();
  return EXIT_SUCCESS;
}
