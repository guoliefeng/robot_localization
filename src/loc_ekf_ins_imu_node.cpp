/*
 * Copyright (c) 2026
 * All rights reserved.
 */

#include "robot_localization/filter_common.h"
#include "robot_localization/ros_filter_types.h"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

std::vector<int> makeImuPoseUpdateVector(const bool use_orientation)
{
  std::vector<int> update_vector = makeEmptyUpdateVector();
  if (use_orientation)
  {
    update_vector[RobotLocalization::StateMemberYaw] = 1;
  }
  return update_vector;
}

std::vector<int> makeImuTwistUpdateVector(const bool use_angular_velocity)
{
  std::vector<int> update_vector = makeEmptyUpdateVector();
  if (use_angular_velocity)
  {
    update_vector[RobotLocalization::StateMemberVyaw] = 1;
  }
  return update_vector;
}

std::vector<int> makeImuAccelUpdateVector(const bool use_linear_acceleration)
{
  std::vector<int> update_vector = makeEmptyUpdateVector();
  if (use_linear_acceleration)
  {
    update_vector[RobotLocalization::StateMemberAx] = 1;
    update_vector[RobotLocalization::StateMemberAy] = 1;
  }
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

geometry_msgs::Vector3 rotateVector(
  const geometry_msgs::Vector3 &vector,
  const tf2::Matrix3x3 &rotation)
{
  const tf2::Vector3 in(vector.x, vector.y, vector.z);
  const tf2::Vector3 out = rotation * in;
  geometry_msgs::Vector3 result;
  result.x = out.x();
  result.y = out.y();
  result.z = out.z();
  return result;
}

void scaleVector(geometry_msgs::Vector3 &vector, const double scale)
{
  vector.x *= scale;
  vector.y *= scale;
  vector.z *= scale;
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

class LocEkfInsImuNode
{
public:
  LocEkfInsImuNode()
    : nh_(),
      nh_priv_("~"),
      ekf_(nh_, nh_priv_, "loc_ekf_ins_imu_node"),
      state_timeout_sec_(1.0),
      ins_pose_cb_data_(makeCallbackData("ins_pose", makePoseUpdateVector(), 5.0)),
      ins_twist_cb_data_(makeCallbackData("ins_twist", makeTwistUpdateVector(), 3.0)),
      imu_pose_cb_data_(makeCallbackData("imu_pose", makeEmptyUpdateVector(), 3.0)),
      imu_twist_cb_data_(makeCallbackData("imu_twist", makeImuTwistUpdateVector(true), 3.0)),
      imu_accel_cb_data_(makeCallbackData("imu_accel", makeEmptyUpdateVector(), 3.0))
  {
    loadParams();
    configureCallbackData();

    ekf_.initialize();

    ins_sub_ = nh_.subscribe(ins_topic_, 100, &LocEkfInsImuNode::insCb, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 200, &LocEkfInsImuNode::imuCb, this);
    localization_status_sub_ = nh_.subscribe(
      localization_status_topic_, 10, &LocEkfInsImuNode::localizationStatusCb, this);
    loc_policy_sub_ = nh_.subscribe(loc_policy_topic_, 10, &LocEkfInsImuNode::locPolicyCb, this);
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
    nh_priv_.param("imu_topic", imu_topic_, std::string("/ins_driver/imu"));
    nh_priv_.param("localization_status_topic", localization_status_topic_, std::string("/localization/status"));
    nh_priv_.param("loc_policy_topic", loc_policy_topic_, std::string("/localization/loc_policy_code"));
    nh_priv_.param("frame_id", frame_id_, std::string("map"));
    nh_priv_.param("child_frame_id", child_frame_id_, std::string("base_link"));
    nh_priv_.param("state_timeout_sec", state_timeout_sec_, 1.0);

    nh_priv_.param("ins_pose_rejection_threshold", ins_pose_rejection_threshold_, 5.0);
    nh_priv_.param("ins_twist_rejection_threshold", ins_twist_rejection_threshold_, 3.0);
    nh_priv_.param("imu_pose_rejection_threshold", imu_pose_rejection_threshold_, 3.0);
    nh_priv_.param("imu_twist_rejection_threshold", imu_twist_rejection_threshold_, 3.0);
    nh_priv_.param("imu_accel_rejection_threshold", imu_accel_rejection_threshold_, 3.0);

    nh_priv_.param("imu_use_orientation", imu_use_orientation_, false);
    nh_priv_.param("imu_use_angular_velocity", imu_use_angular_velocity_, true);
    nh_priv_.param("imu_use_linear_acceleration", imu_use_linear_acceleration_, false);
    nh_priv_.param("imu_extrinsic_roll", imu_extrinsic_roll_, 0.0);
    nh_priv_.param("imu_extrinsic_pitch", imu_extrinsic_pitch_, 0.0);
    nh_priv_.param("imu_extrinsic_yaw", imu_extrinsic_yaw_, -1.57079632679);
    nh_priv_.param("imu_linear_acceleration_scale", imu_linear_acceleration_scale_, 9.806);
    nh_priv_.param("imu_primary_yaw_rate_std_deg", imu_primary_yaw_rate_std_deg_, 1.0);
    nh_priv_.param("imu_weak_yaw_rate_std_deg", imu_weak_yaw_rate_std_deg_, 180.0);
    nh_priv_.param("imu_degraded_yaw_rate_std_deg", imu_degraded_yaw_rate_std_deg_, 30.0);
    nh_priv_.param("imu_primary_linear_acceleration_std", imu_primary_linear_acceleration_std_, 20.0);
    nh_priv_.param("imu_weak_linear_acceleration_std", imu_weak_linear_acceleration_std_, 20.0);
    nh_priv_.param("imu_degraded_linear_acceleration_std", imu_degraded_linear_acceleration_std_, 20.0);

  }

  void configureCallbackData()
  {
    const std::vector<int> pose_update_vector = makePoseUpdateVector();
    const std::vector<int> twist_update_vector = makeTwistUpdateVector();
    const std::vector<int> imu_pose_update_vector = makeImuPoseUpdateVector(imu_use_orientation_);
    const std::vector<int> imu_twist_update_vector = makeImuTwistUpdateVector(imu_use_angular_velocity_);
    const std::vector<int> imu_accel_update_vector = makeImuAccelUpdateVector(imu_use_linear_acceleration_);

    ins_pose_cb_data_ =
      makeCallbackData("ins_pose", pose_update_vector, ins_pose_rejection_threshold_);
    ins_twist_cb_data_ =
      makeCallbackData("ins_twist", twist_update_vector, ins_twist_rejection_threshold_);
    imu_pose_cb_data_ =
      makeCallbackData("imu_pose", imu_pose_update_vector, imu_pose_rejection_threshold_);
    imu_twist_cb_data_ =
      makeCallbackData("imu_twist", imu_twist_update_vector, imu_twist_rejection_threshold_);
    imu_accel_cb_data_ =
      makeCallbackData("imu_accel", imu_accel_update_vector, imu_accel_rejection_threshold_);

    imu_to_base_quat_.setRPY(imu_extrinsic_roll_, imu_extrinsic_pitch_, imu_extrinsic_yaw_);
    imu_to_base_quat_.normalize();
    imu_to_base_rotation_.setRotation(imu_to_base_quat_);
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

  bool hasValidLocDecision(const LocDecision &decision) const
  {
    return !decision.last_status_stamp.isZero() &&
           !decision.last_policy_stamp.isZero() &&
           decision.status != LocalizationStatus::LOST &&
           decision.policy != static_cast<unsigned long long>(LocalizationPolicy::NO_LOCALIZATION);
  }

  bool isAbsolutePoseInitialized() const
  {
    return absolute_pose_initialized_;
  }

  void markAbsolutePoseInitialized(const ros::Time &stamp)
  {
    if (!absolute_pose_initialized_)
    {
      absolute_pose_initialized_ = true;
      first_absolute_pose_stamp_ = stamp;
      ++absolute_init_count_;
      ROS_WARN_STREAM(
        "LocEkfInsImu initialized by first absolute pose at t=" << stamp.toSec());
    }
  }

  SourceMode selectInsMode(const LocDecision &decision) const
  {
    if (decision.status == LocalizationStatus::LOST)
    {
      return SourceMode::DROP;
    }

    if (decision.status == LocalizationStatus::NORMAL ||
        decision.status == LocalizationStatus::SECONDARY)
    {
      return SourceMode::PRIMARY;
    }

    if (decision.status == LocalizationStatus::NOT_STABLE)
    {
      return SourceMode::DEGRADED;
    }

    if (hasPolicy(decision.policy, LocalizationPolicy::USE_GNSS) ||
        hasPolicy(decision.policy, LocalizationPolicy::USE_RELOC_GNSS) ||
        hasPolicy(decision.policy, LocalizationPolicy::USE_RELOC_X))
    {
      return SourceMode::WEAK;
    }

    return SourceMode::DROP;
  }

  SourceMode selectImuMode(const LocDecision &decision) const
  {
    if (decision.status == LocalizationStatus::LOST)
    {
      return SourceMode::DROP;
    }

    if (decision.status == LocalizationStatus::DR &&
        hasPolicy(decision.policy, LocalizationPolicy::USE_ODOM_VEL_IMU))
    {
      return SourceMode::PRIMARY;
    }

    if (decision.status == LocalizationStatus::DR)
    {
      return SourceMode::WEAK;
    }

    if (decision.status == LocalizationStatus::NORMAL ||
        decision.status == LocalizationStatus::NOT_STABLE ||
        decision.status == LocalizationStatus::SECONDARY)
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

  void applyImuCovariance(sensor_msgs::Imu &imu, const SourceMode mode) const
  {
    std::fill(imu.orientation_covariance.begin(), imu.orientation_covariance.end(), 0.0);
    std::fill(imu.angular_velocity_covariance.begin(), imu.angular_velocity_covariance.end(), 0.0);
    std::fill(imu.linear_acceleration_covariance.begin(), imu.linear_acceleration_covariance.end(), 0.0);

    imu.orientation_covariance[0] = -1.0;

    switch (mode)
    {
      case SourceMode::PRIMARY:
        imu.angular_velocity_covariance[8] = std::pow(deg2rad(imu_primary_yaw_rate_std_deg_), 2.0);
        if (imu_use_linear_acceleration_)
        {
          imu.linear_acceleration_covariance[0] = std::pow(imu_primary_linear_acceleration_std_, 2.0);
          imu.linear_acceleration_covariance[4] = std::pow(imu_primary_linear_acceleration_std_, 2.0);
          imu.linear_acceleration_covariance[8] = 10000.0;
        }
        break;
      case SourceMode::WEAK:
        imu.angular_velocity_covariance[8] = std::pow(deg2rad(imu_weak_yaw_rate_std_deg_), 2.0);
        if (imu_use_linear_acceleration_)
        {
          imu.linear_acceleration_covariance[0] = std::pow(imu_weak_linear_acceleration_std_, 2.0);
          imu.linear_acceleration_covariance[4] = std::pow(imu_weak_linear_acceleration_std_, 2.0);
          imu.linear_acceleration_covariance[8] = 10000.0;
        }
        break;
      case SourceMode::DEGRADED:
        imu.angular_velocity_covariance[8] = std::pow(deg2rad(imu_degraded_yaw_rate_std_deg_), 2.0);
        if (imu_use_linear_acceleration_)
        {
          imu.linear_acceleration_covariance[0] = std::pow(imu_degraded_linear_acceleration_std_, 2.0);
          imu.linear_acceleration_covariance[4] = std::pow(imu_degraded_linear_acceleration_std_, 2.0);
          imu.linear_acceleration_covariance[8] = 10000.0;
        }
        break;
      case SourceMode::DROP:
        break;
    }

    if (!imu_use_linear_acceleration_)
    {
      imu.linear_acceleration_covariance[0] = -1.0;
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
    const SourceMode imu_mode = selectImuMode(decision);
    ++ins_count_;

    if (!hasValidLocDecision(decision))
    {
      ++ins_drop_count_;
      ROS_WARN_STREAM_THROTTLE(1.0, "Drop INS before valid localization status/policy.");
      logStatus(decision, ins_mode, imu_mode);
      return;
    }

    if (ins_mode == SourceMode::DROP)
    {
      ++ins_drop_count_;
      logStatus(decision, ins_mode, imu_mode);
      return;
    }

    nav_msgs::OdometryPtr out(new nav_msgs::Odometry(*msg));
    normalizeOdomFrame(*out);
    applyInsCovariance(*out, ins_mode);

    markAbsolutePoseInitialized(out->header.stamp);
    ekf_.odometryCallback(out, "ins_odom", ins_pose_cb_data_, ins_twist_cb_data_);
    logStatus(decision, ins_mode, imu_mode);
  }

  void imuCb(const sensor_msgs::Imu::ConstPtr &msg)
  {
    LocDecision decision;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      decision = currentDecision();
    }

    const SourceMode ins_mode = selectInsMode(decision);
    const SourceMode imu_mode = selectImuMode(decision);
    ++imu_count_;

    if (imu_mode == SourceMode::DROP)
    {
      ++imu_drop_count_;
      logStatus(decision, ins_mode, imu_mode);
      return;
    }

    if (!isAbsolutePoseInitialized())
    {
      ++imu_drop_count_;
      ++preinit_imu_drop_count_;
      ROS_WARN_STREAM_THROTTLE(
        1.0,
        "Drop IMU before first absolute pose initialization.");
      logStatus(decision, ins_mode, imu_mode);
      return;
    }

    sensor_msgs::ImuPtr out(new sensor_msgs::Imu(*msg));
    out->header.frame_id = child_frame_id_;
    out->angular_velocity = rotateVector(msg->angular_velocity, imu_to_base_rotation_);
    out->linear_acceleration = rotateVector(msg->linear_acceleration, imu_to_base_rotation_);
    scaleVector(out->linear_acceleration, imu_linear_acceleration_scale_);

    tf2::Quaternion imu_orientation;
    tf2::fromMsg(msg->orientation, imu_orientation);
    tf2::Quaternion base_orientation = imu_orientation * imu_to_base_quat_.inverse();
    base_orientation.normalize();
    out->orientation = tf2::toMsg(base_orientation);
    applyImuCovariance(*out, imu_mode);

    ekf_.imuCallback(out, "imu", imu_pose_cb_data_, imu_twist_cb_data_, imu_accel_cb_data_);
    logStatus(decision, ins_mode, imu_mode);
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
    const SourceMode imu_mode) const
  {
    ROS_INFO_STREAM_THROTTLE(
      1.0,
      "LocEkfInsImu state: status=" << statusToString(decision.status)
                                         << " policy=" << decision.policy
                                         << " ins=" << modeToString(ins_mode)
                                         << " imu=" << modeToString(imu_mode)
                                         << " abs_init=" << absolute_pose_initialized_
                                         << " abs_init_count=" << absolute_init_count_
                                         << " ins_in=" << ins_count_
                                         << " ins_drop=" << ins_drop_count_
                                         << " imu_in=" << imu_count_
                                         << " imu_drop=" << imu_drop_count_
                                         << " preinit_imu_drop=" << preinit_imu_drop_count_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  RobotLocalization::RosEkf ekf_;

  ros::Subscriber ins_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber localization_status_sub_;
  ros::Subscriber loc_policy_sub_;

  mutable std::mutex state_mtx_;
  LocDecision loc_decision_;

  std::string ins_topic_;
  std::string imu_topic_;
  std::string localization_status_topic_;
  std::string loc_policy_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  double state_timeout_sec_;
  double ins_pose_rejection_threshold_ = 5.0;
  double ins_twist_rejection_threshold_ = 3.0;
  double imu_pose_rejection_threshold_ = 3.0;
  double imu_twist_rejection_threshold_ = 3.0;
  double imu_accel_rejection_threshold_ = 3.0;

  bool imu_use_orientation_ = false;
  bool imu_use_angular_velocity_ = true;
  bool imu_use_linear_acceleration_ = false;
  double imu_extrinsic_roll_ = 0.0;
  double imu_extrinsic_pitch_ = 0.0;
  double imu_extrinsic_yaw_ = -1.57079632679;
  double imu_linear_acceleration_scale_ = 9.806;
  double imu_primary_yaw_rate_std_deg_ = 1.0;
  double imu_weak_yaw_rate_std_deg_ = 180.0;
  double imu_degraded_yaw_rate_std_deg_ = 30.0;
  double imu_primary_linear_acceleration_std_ = 20.0;
  double imu_weak_linear_acceleration_std_ = 20.0;
  double imu_degraded_linear_acceleration_std_ = 20.0;
  tf2::Quaternion imu_to_base_quat_;
  tf2::Matrix3x3 imu_to_base_rotation_;

  RobotLocalization::CallbackData ins_pose_cb_data_;
  RobotLocalization::CallbackData ins_twist_cb_data_;
  RobotLocalization::CallbackData imu_pose_cb_data_;
  RobotLocalization::CallbackData imu_twist_cb_data_;
  RobotLocalization::CallbackData imu_accel_cb_data_;

  bool absolute_pose_initialized_ = false;
  ros::Time first_absolute_pose_stamp_;

  uint64_t ins_count_ = 0;
  uint64_t ins_drop_count_ = 0;
  uint64_t imu_count_ = 0;
  uint64_t imu_drop_count_ = 0;
  uint64_t preinit_imu_drop_count_ = 0;
  uint64_t absolute_init_count_ = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_ekf_ins_imu_node");
  LocEkfInsImuNode node;
  ros::spin();

  return EXIT_SUCCESS;
}
