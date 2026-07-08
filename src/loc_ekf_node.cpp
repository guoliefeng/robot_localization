/*
 * Copyright (c) 2026
 * All rights reserved.
 */

#include "robot_localization/filter_common.h"
#include "robot_localization/ros_filter_types.h"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

namespace
{

enum class LocalizationStatus : unsigned int
{
  NORMAL = 0x00,
  DR = 0x01,
  LOST = 0x02,
  NOT_STABLE = 0x04,
  SECONDARY = 0x08
};

enum class LocalizationPolicy : unsigned long long
{
  NO_LOCALIZATION = 0,
  USE_LIDAR_LIO = 1,
  USE_FUSION_ODOM = 2,
  USE_GNSS = 4,
  USE_ODOM_VEL_IMU = 8,
  USE_ODOM_VEL = 16,
  USE_RELOC_GNSS = 32,
  USE_RELOC_X = 64
};

enum class SourceMode
{
  PRIMARY,
  WEAK,
  DEGRADED,
  DROP
};

double deg2rad(const double deg)
{
  return deg * M_PI / 180.0;
}

std::vector<int> makeEmptyUpdateVector()
{
  return std::vector<int>(RobotLocalization::STATE_SIZE, 0);
}

std::vector<int> makePoseUpdateVector()
{
  std::vector<int> update_vector = makeEmptyUpdateVector();
  update_vector[RobotLocalization::StateMemberX] = 1;
  update_vector[RobotLocalization::StateMemberY] = 1;
  update_vector[RobotLocalization::StateMemberYaw] = 1;
  return update_vector;
}

std::vector<int> makeTwistUpdateVector()
{
  std::vector<int> update_vector = makeEmptyUpdateVector();
  update_vector[RobotLocalization::StateMemberVx] = 1;
  update_vector[RobotLocalization::StateMemberVy] = 1;
  update_vector[RobotLocalization::StateMemberVyaw] = 1;
  return update_vector;
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

bool hasPolicy(const unsigned long long policy, const LocalizationPolicy flag)
{
  return (policy & static_cast<unsigned long long>(flag)) != 0;
}

void clearCovariances(nav_msgs::Odometry &odom)
{
  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
}

void setPoseCov(
  nav_msgs::Odometry &odom,
  const double xy_var,
  const double z_var,
  const double roll_pitch_var,
  const double yaw_var)
{
  odom.pose.covariance[0] = xy_var;
  odom.pose.covariance[7] = xy_var;
  odom.pose.covariance[14] = z_var;
  odom.pose.covariance[21] = roll_pitch_var;
  odom.pose.covariance[28] = roll_pitch_var;
  odom.pose.covariance[35] = yaw_var;
}

void setTwistCov(
  nav_msgs::Odometry &odom,
  const double vxy_var,
  const double vz_var,
  const double vroll_pitch_var,
  const double vyaw_var)
{
  odom.twist.covariance[0] = vxy_var;
  odom.twist.covariance[7] = vxy_var;
  odom.twist.covariance[14] = vz_var;
  odom.twist.covariance[21] = vroll_pitch_var;
  odom.twist.covariance[28] = vroll_pitch_var;
  odom.twist.covariance[35] = vyaw_var;
}

const char *statusToString(const LocalizationStatus status)
{
  switch (status)
  {
    case LocalizationStatus::NORMAL:
      return "NORMAL";
    case LocalizationStatus::DR:
      return "DR";
    case LocalizationStatus::LOST:
      return "LOST";
    case LocalizationStatus::NOT_STABLE:
      return "NOT_STABLE";
    case LocalizationStatus::SECONDARY:
      return "SECONDARY";
  }

  return "UNKNOWN";
}

const char *modeToString(const SourceMode mode)
{
  switch (mode)
  {
    case SourceMode::PRIMARY:
      return "PRIMARY";
    case SourceMode::WEAK:
      return "WEAK";
    case SourceMode::DEGRADED:
      return "DEGRADED";
    case SourceMode::DROP:
      return "DROP";
  }

  return "UNKNOWN";
}

}  // namespace

class LocEkfNode
{
public:
  LocEkfNode()
    : nh_(),
      nh_priv_("~"),
      ekf_(nh_, nh_priv_, "loc_ekf_node"),
      state_timeout_sec_(1.0),
      ins_pose_cb_data_(makeCallbackData("ins_pose", makePoseUpdateVector(), 5.0)),
      ins_twist_cb_data_(makeCallbackData("ins_twist", makeTwistUpdateVector(), 3.0)),
      fusion_pose_cb_data_(makeCallbackData("fusion_pose", makePoseUpdateVector(), 3.0)),
      fusion_twist_cb_data_(makeCallbackData("fusion_twist", makeTwistUpdateVector(), 3.0))
  {
    loadParams();
    configureCallbackData();

    ekf_.initialize();

    ins_sub_ = nh_.subscribe(ins_topic_, 100, &LocEkfNode::insCb, this);
    fusion_sub_ = nh_.subscribe(fusion_topic_, 100, &LocEkfNode::fusionCb, this);
    localization_status_sub_ = nh_.subscribe(
      localization_status_topic_, 10, &LocEkfNode::localizationStatusCb, this);
    loc_policy_sub_ = nh_.subscribe(loc_policy_topic_, 10, &LocEkfNode::locPolicyCb, this);
  }

private:
  struct LocDecision
  {
    LocalizationStatus status = LocalizationStatus::LOST;
    unsigned long long policy = static_cast<unsigned long long>(LocalizationPolicy::NO_LOCALIZATION);
    ros::Time last_status_stamp;
    ros::Time last_policy_stamp;
  };

  void loadParams()
  {
    nh_priv_.param("ins_odom_topic", ins_topic_, std::string("/localization/ins"));
    nh_priv_.param("fusion_odom_topic", fusion_topic_, std::string("/lio_loc_result"));
    nh_priv_.param("localization_status_topic", localization_status_topic_, std::string("/localization/status"));
    nh_priv_.param("loc_policy_topic", loc_policy_topic_, std::string("/localization/loc_policy_code"));
    nh_priv_.param("frame_id", frame_id_, std::string("map"));
    nh_priv_.param("child_frame_id", child_frame_id_, std::string("base_link"));
    nh_priv_.param("state_timeout_sec", state_timeout_sec_, 1.0);
    nh_priv_.param("ins_pose_rejection_threshold", ins_pose_rejection_threshold_, 5.0);
    nh_priv_.param("ins_twist_rejection_threshold", ins_twist_rejection_threshold_, 3.0);
    nh_priv_.param("fusion_pose_rejection_threshold", fusion_pose_rejection_threshold_, 3.0);
    nh_priv_.param("fusion_twist_rejection_threshold", fusion_twist_rejection_threshold_, 3.0);
  }

  void configureCallbackData()
  {
    const std::vector<int> pose_update_vector = makePoseUpdateVector();
    const std::vector<int> twist_update_vector = makeTwistUpdateVector();

    ins_pose_cb_data_ =
      makeCallbackData("ins_pose", pose_update_vector, ins_pose_rejection_threshold_);
    ins_twist_cb_data_ =
      makeCallbackData("ins_twist", twist_update_vector, ins_twist_rejection_threshold_);
    fusion_pose_cb_data_ =
      makeCallbackData("fusion_pose", pose_update_vector, fusion_pose_rejection_threshold_);
    fusion_twist_cb_data_ =
      makeCallbackData("fusion_twist", twist_update_vector, fusion_twist_rejection_threshold_);
  }

  void normalizeOdomFrame(nav_msgs::Odometry &odom) const
  {
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
  }

  LocDecision currentDecision() const
  {
    LocDecision decision = loc_decision_;
    const ros::Time now = ros::Time::now();
    const bool status_timeout =
      decision.last_status_stamp.isZero() ||
      (now - decision.last_status_stamp).toSec() > state_timeout_sec_;
    const bool policy_timeout =
      decision.last_policy_stamp.isZero() ||
      (now - decision.last_policy_stamp).toSec() > state_timeout_sec_;

    if (status_timeout || policy_timeout)
    {
      decision.status = LocalizationStatus::LOST;
      decision.policy = static_cast<unsigned long long>(LocalizationPolicy::NO_LOCALIZATION);
    }

    return decision;
  }

  SourceMode selectInsMode(const LocDecision &decision) const
  {
    if (decision.status == LocalizationStatus::LOST ||
        decision.status == LocalizationStatus::DR)
    {
      return SourceMode::DROP;
    }

    if (decision.status == LocalizationStatus::NORMAL &&
        hasPolicy(decision.policy, LocalizationPolicy::USE_FUSION_ODOM))
    {
      return SourceMode::DROP;
    }

    if (hasPolicy(decision.policy, LocalizationPolicy::USE_GNSS))
    {
      if (decision.status == LocalizationStatus::NORMAL ||
          decision.status == LocalizationStatus::SECONDARY)
      {
        return SourceMode::PRIMARY;
      }

      if (decision.status == LocalizationStatus::NOT_STABLE)
      {
        return SourceMode::DEGRADED;
      }
    }

    if (hasPolicy(decision.policy, LocalizationPolicy::USE_FUSION_ODOM))
    {
      return SourceMode::WEAK;
    }

    return SourceMode::DROP;
  }

  SourceMode selectFusionMode(const LocDecision &decision) const
  {
    if (decision.status == LocalizationStatus::LOST ||
        decision.status == LocalizationStatus::DR)
    {
      return SourceMode::DROP;
    }

    if (hasPolicy(decision.policy, LocalizationPolicy::USE_FUSION_ODOM))
    {
      if (decision.status == LocalizationStatus::NORMAL)
      {
        return SourceMode::PRIMARY;
      }

      if (decision.status == LocalizationStatus::NOT_STABLE)
      {
        return SourceMode::DEGRADED;
      }
    }

    if (hasPolicy(decision.policy, LocalizationPolicy::USE_GNSS))
    {
      return SourceMode::WEAK;
    }

    return SourceMode::DROP;
  }

  void applyInsCovariance(nav_msgs::Odometry &odom, const SourceMode mode) const
  {
    clearCovariances(odom);

    switch (mode)
    {
      case SourceMode::PRIMARY:
        setPoseCov(odom, 0.02, 100.0, 100.0, std::pow(deg2rad(0.5), 2.0));
        setTwistCov(odom, 0.05, 100.0, 100.0, std::pow(deg2rad(0.5), 2.0));
        break;
      case SourceMode::WEAK:
        setPoseCov(odom, 10.0, 100.0, 100.0, std::pow(deg2rad(10.0), 2.0));
        setTwistCov(odom, 0.50, 100.0, 100.0, std::pow(deg2rad(5.0), 2.0));
        break;
      case SourceMode::DEGRADED:
        setPoseCov(odom, 1.0, 100.0, 100.0, std::pow(deg2rad(3.0), 2.0));
        setTwistCov(odom, 0.20, 100.0, 100.0, std::pow(deg2rad(2.0), 2.0));
        break;
      case SourceMode::DROP:
        break;
    }
  }

  void applyFusionCovariance(nav_msgs::Odometry &odom, const SourceMode mode) const
  {
    clearCovariances(odom);

    switch (mode)
    {
      case SourceMode::PRIMARY:
        setPoseCov(odom, 0.05, 100.0, 100.0, std::pow(deg2rad(1.0), 2.0));
        setTwistCov(odom, 0.20, 100.0, 100.0, std::pow(deg2rad(1.0), 2.0));
        break;
      case SourceMode::WEAK:
        setPoseCov(odom, 10.0, 100.0, 100.0, std::pow(deg2rad(10.0), 2.0));
        setTwistCov(odom, 1.0, 100.0, 100.0, std::pow(deg2rad(8.0), 2.0));
        break;
      case SourceMode::DEGRADED:
        setPoseCov(odom, 5.0, 100.0, 100.0, std::pow(deg2rad(5.0), 2.0));
        setTwistCov(odom, 1.0, 100.0, 100.0, std::pow(deg2rad(5.0), 2.0));
        break;
      case SourceMode::DROP:
        break;
    }
  }

  void insCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    LocDecision decision;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      decision = currentDecision();
    }

    const SourceMode ins_mode = selectInsMode(decision);
    const SourceMode fusion_mode = selectFusionMode(decision);
    ++ins_count_;

    if (ins_mode == SourceMode::DROP)
    {
      ++ins_drop_count_;
      logStatus(decision, ins_mode, fusion_mode);
      return;
    }

    nav_msgs::OdometryPtr out(new nav_msgs::Odometry(*msg));
    normalizeOdomFrame(*out);
    applyInsCovariance(*out, ins_mode);

    ekf_.odometryCallback(out, "ins_odom", ins_pose_cb_data_, ins_twist_cb_data_);
    logStatus(decision, ins_mode, fusion_mode);
  }

  void fusionCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    LocDecision decision;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      decision = currentDecision();
    }

    const SourceMode ins_mode = selectInsMode(decision);
    const SourceMode fusion_mode = selectFusionMode(decision);
    ++fusion_count_;

    if (fusion_mode == SourceMode::DROP)
    {
      ++fusion_drop_count_;
      logStatus(decision, ins_mode, fusion_mode);
      return;
    }

    nav_msgs::OdometryPtr out(new nav_msgs::Odometry(*msg));
    normalizeOdomFrame(*out);
    applyFusionCovariance(*out, fusion_mode);

    ekf_.odometryCallback(out, "fusion_odom", fusion_pose_cb_data_, fusion_twist_cb_data_);
    logStatus(decision, ins_mode, fusion_mode);
  }

  void localizationStatusCb(const std_msgs::UInt8::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    loc_decision_.status = static_cast<LocalizationStatus>(msg->data);
    loc_decision_.last_status_stamp = ros::Time::now();
  }

  void locPolicyCb(const std_msgs::UInt64::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    loc_decision_.policy = msg->data;
    loc_decision_.last_policy_stamp = ros::Time::now();
  }

  void logStatus(
    const LocDecision &decision,
    const SourceMode ins_mode,
    const SourceMode fusion_mode) const
  {
    ROS_INFO_STREAM_THROTTLE(
      1.0,
      "YangpuLocEkf loc_state: status=" << statusToString(decision.status)
                                        << " policy=" << decision.policy
                                        << " ins=" << modeToString(ins_mode)
                                        << " fusion=" << modeToString(fusion_mode)
                                        << " ins_in=" << ins_count_
                                        << " ins_drop=" << ins_drop_count_
                                        << " fusion_in=" << fusion_count_
                                        << " fusion_drop=" << fusion_drop_count_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  RobotLocalization::RosEkf ekf_;

  ros::Subscriber ins_sub_;
  ros::Subscriber fusion_sub_;
  ros::Subscriber localization_status_sub_;
  ros::Subscriber loc_policy_sub_;

  mutable std::mutex state_mtx_;
  LocDecision loc_decision_;

  std::string ins_topic_;
  std::string fusion_topic_;
  std::string localization_status_topic_;
  std::string loc_policy_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  double state_timeout_sec_;
  double ins_pose_rejection_threshold_ = 5.0;
  double ins_twist_rejection_threshold_ = 3.0;
  double fusion_pose_rejection_threshold_ = 3.0;
  double fusion_twist_rejection_threshold_ = 3.0;

  RobotLocalization::CallbackData ins_pose_cb_data_;
  RobotLocalization::CallbackData ins_twist_cb_data_;
  RobotLocalization::CallbackData fusion_pose_cb_data_;
  RobotLocalization::CallbackData fusion_twist_cb_data_;

  uint64_t ins_count_ = 0;
  uint64_t ins_drop_count_ = 0;
  uint64_t fusion_count_ = 0;
  uint64_t fusion_drop_count_ = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_ekf_node");
  LocEkfNode node;
  ros::spin();

  return EXIT_SUCCESS;
}
