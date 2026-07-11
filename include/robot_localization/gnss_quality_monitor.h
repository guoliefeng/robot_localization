/*
 * Copyright (c) 2026
 * All rights reserved.
 */

#ifndef ROBOT_LOCALIZATION_GNSS_QUALITY_MONITOR_H
#define ROBOT_LOCALIZATION_GNSS_QUALITY_MONITOR_H

#include "robot_localization/filter_common.h"

#include <chcnav_msgs/hcinspvatzcb.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mutex>
#include <string>

namespace RobotLocalization
{

class GnssQualityMonitor
{
public:
  enum class State
  {
    GNSS_GOOD,
    GNSS_DEGRADED,
    GNSS_BAD,
    GNSS_NO_DATA
  };

  GnssQualityMonitor();

  void initialize(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);

  bool applyToOdometry(nav_msgs::Odometry &odom);

  static const char *stateToString(State state);

private:
  struct CovarianceOutput
  {
    double sigma_xy = 5.0;
    double sigma_yaw = 0.0;
    double cov_x = 25.0;
    double cov_y = 25.0;
    double cov_yaw = 0.0;
  };

  struct Params
  {
    double devpvt_timeout = 0.2;
    double good_to_degraded_time = 0.5;
    double degraded_to_bad_time = 0.3;
    double bad_to_degraded_time = 1.0;
    double degraded_to_good_time = 2.0;
    double stat1_float_bad_duration = 10.0;
    double ns2_zero_bad_duration = 1.0;
    double min_good_sigma_xy = 0.05;
    double min_degraded_sigma_xy = 0.15;
    double min_stat1_float_sigma_xy = 0.20;
    double min_bad_sigma_xy = 0.50;
    double no_data_sigma_xy = 5.0;
    double min_good_sigma_yaw_deg = 0.5;
    double min_degraded_sigma_yaw_deg = 2.0;
    double min_stat1_float_sigma_yaw_deg = 3.0;
    double min_bad_sigma_yaw_deg = 10.0;
    double no_data_sigma_yaw_deg = 30.0;

    double stat1_float_to_degraded_time = 0.3;
    double stat1_float_age_bad_time = 0.5;
    double no_data_recovery_observe_time = 1.0;
    double debug_publish_period = 0.1;
  };

  struct Sample
  {
    uint8_t stat0 = 0;
    uint8_t stat1 = 0;
    uint16_t ns = 0;
    uint16_t ns2 = 0;
    double age = 0.0;
    double hdop = 0.0;
    double pdop = 0.0;
    double gdop = 0.0;
    double pos_std_h = 0.0;
    double yaw_std_deg = 0.0;
    uint16_t warning = 0;
    bool warning_changed = false;
    int score = 0;
    ros::Time stamp;
  };

  void loadParams();
  void devpvtCallback(const chcnav_msgs::hcinspvatzcb::ConstPtr &msg);
  Sample makeSample(const chcnav_msgs::hcinspvatzcb &msg);
  int computeScore(const Sample &sample) const;
  void updateStateFromSample(const ros::Time &now);
  void updateNoDataState(const ros::Time &now);
  CovarianceOutput computeCovariance() const;
  std::string buildDebugString(const CovarianceOutput &covariance) const;
  void publishDebugIfNeeded(const ros::Time &now, const std::string &debug_string);
  void transitionTo(State state, const std::string &reason, const ros::Time &now);
  void resetConditionTimers();

  bool held(bool condition, ros::Time &start_time, const ros::Time &now, double duration) const;
  double elapsedSince(const ros::Time &start_time, const ros::Time &now) const;
  double degToRad(double degrees) const;

  template<typename T>
  void paramWithFallback(const std::string &name, T &value, const T &default_value) const
  {
    if (!nh_priv_.getParam("gnss_quality_monitor/" + name, value))
    {
      nh_priv_.param(name, value, default_value);
    }
  }

  mutable std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::Subscriber devpvt_sub_;
  ros::Publisher debug_pub_;

  Params params_;
  bool enabled_;
  bool debug_log_;
  bool publish_debug_;
  std::string devpvt_topic_;
  std::string debug_topic_;

  State state_;
  ros::Time state_enter_time_;
  ros::Time last_devpvt_stamp_;
  ros::Time no_data_recovery_until_;
  ros::Time last_debug_publish_time_;

  bool has_sample_;
  bool has_last_warning_;
  uint16_t last_warning_;
  Sample sample_;
  CovarianceOutput last_covariance_;
  std::string last_transition_reason_;

  ros::Time stat1_float_start_;
  ros::Time score_lt_80_start_;
  ros::Time ns2_zero_start_;
  ros::Time stat1_other_start_;
  ros::Time float_age_gt_10_start_;
  ros::Time score_lt_40_start_;
  ros::Time good_candidate_start_;
  ros::Time bad_recovery_candidate_start_;
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_GNSS_QUALITY_MONITOR_H
