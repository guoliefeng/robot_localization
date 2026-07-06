/*
 * Copyright (c) 2026
 * All rights reserved.
 */

#include "robot_localization/filter_common.h"
#include "robot_localization/ros_filter_types.h"

#include <geometry_msgs/TwistWithCovarianceStamped.h>
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

enum class InputQuality
{
  GOOD,
  DEGRADED,
  BAD
};

double deg2rad(const double deg)
{
  return deg * M_PI / 180.0;
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
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

void setWheelTwistCov(
  geometry_msgs::TwistWithCovarianceStamped &twist,
  const double vxy_var,
  const double vz_var,
  const double vroll_pitch_var,
  const double vyaw_var)
{
  std::fill(twist.twist.covariance.begin(), twist.twist.covariance.end(), 0.0);
  twist.twist.covariance[0] = vxy_var;
  twist.twist.covariance[7] = vxy_var;
  twist.twist.covariance[14] = vz_var;
  twist.twist.covariance[21] = vroll_pitch_var;
  twist.twist.covariance[28] = vroll_pitch_var;
  twist.twist.covariance[35] = vyaw_var;
}

double yawFromOdom(const nav_msgs::Odometry &odom)
{
  tf2::Quaternion q;
  tf2::fromMsg(odom.pose.pose.orientation, q);
  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
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

const char *qualityToString(const InputQuality quality)
{
  switch (quality)
  {
    case InputQuality::GOOD:
      return "GOOD";
    case InputQuality::DEGRADED:
      return "DEGRADED";
    case InputQuality::BAD:
      return "BAD";
  }

  return "UNKNOWN";
}

}  // namespace

class LocEkfMultiSensorNode
{
public:
  LocEkfMultiSensorNode()
    : nh_(),
      nh_priv_("~"),
      ekf_(nh_, nh_priv_, "loc_ekf_multi_sensor_node"),
      state_timeout_sec_(1.0),
      ins_pose_cb_data_(makeCallbackData("ins_pose", makePoseUpdateVector(), 5.0)),
      ins_twist_cb_data_(makeCallbackData("ins_twist", makeTwistUpdateVector(), 3.0)),
      fusion_pose_cb_data_(makeCallbackData("fusion_pose", makePoseUpdateVector(), 3.0)),
      fusion_twist_cb_data_(makeCallbackData("fusion_twist", makeTwistUpdateVector(), 3.0)),
      wheel_twist_cb_data_(makeCallbackData("wheel_twist", makeTwistUpdateVector(), 3.0)),
      imu_pose_cb_data_(makeCallbackData("imu_pose", makeEmptyUpdateVector(), 3.0)),
      imu_twist_cb_data_(makeCallbackData("imu_twist", makeImuTwistUpdateVector(true), 3.0)),
      imu_accel_cb_data_(makeCallbackData("imu_accel", makeEmptyUpdateVector(), 3.0))
  {
    loadParams();
    configureCallbackData();

    ekf_.initialize();

    ins_sub_ = nh_.subscribe(ins_topic_, 100, &LocEkfMultiSensorNode::insCb, this);
    fusion_sub_ = nh_.subscribe(fusion_topic_, 100, &LocEkfMultiSensorNode::fusionCb, this);
    wheel_sub_ = nh_.subscribe(wheel_odom_topic_, 100, &LocEkfMultiSensorNode::wheelCb, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 200, &LocEkfMultiSensorNode::imuCb, this);
    localization_status_sub_ = nh_.subscribe(
      localization_status_topic_, 10, &LocEkfMultiSensorNode::localizationStatusCb, this);
    loc_policy_sub_ = nh_.subscribe(loc_policy_topic_, 10, &LocEkfMultiSensorNode::locPolicyCb, this);
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
    nh_priv_.param("wheel_odom_topic", wheel_odom_topic_, std::string("/wheel_odom"));
    nh_priv_.param("imu_topic", imu_topic_, std::string("/ins_driver/imu"));
    nh_priv_.param("localization_status_topic", localization_status_topic_, std::string("/localization/status"));
    nh_priv_.param("loc_policy_topic", loc_policy_topic_, std::string("/localization/loc_policy_code"));
    nh_priv_.param("frame_id", frame_id_, std::string("map"));
    nh_priv_.param("child_frame_id", child_frame_id_, std::string("base_link"));
    nh_priv_.param("state_timeout_sec", state_timeout_sec_, 1.0);

    nh_priv_.param("ins_pose_rejection_threshold", ins_pose_rejection_threshold_, 5.0);
    nh_priv_.param("ins_twist_rejection_threshold", ins_twist_rejection_threshold_, 3.0);
    nh_priv_.param("fusion_pose_rejection_threshold", fusion_pose_rejection_threshold_, 3.0);
    nh_priv_.param("fusion_twist_rejection_threshold", fusion_twist_rejection_threshold_, 3.0);
    nh_priv_.param("wheel_twist_rejection_threshold", wheel_twist_rejection_threshold_, 3.0);
    nh_priv_.param("imu_pose_rejection_threshold", imu_pose_rejection_threshold_, 3.0);
    nh_priv_.param("imu_twist_rejection_threshold", imu_twist_rejection_threshold_, 3.0);
    nh_priv_.param("imu_accel_rejection_threshold", imu_accel_rejection_threshold_, 3.0);

    nh_priv_.param("imu_use_orientation", imu_use_orientation_, false);
    nh_priv_.param("imu_use_angular_velocity", imu_use_angular_velocity_, true);
    nh_priv_.param("imu_use_linear_acceleration", imu_use_linear_acceleration_, false);
    nh_priv_.param("imu_extrinsic_roll", imu_extrinsic_roll_, 0.0);
    nh_priv_.param("imu_extrinsic_pitch", imu_extrinsic_pitch_, 0.0);
    nh_priv_.param("imu_extrinsic_yaw", imu_extrinsic_yaw_, 0.0);

    nh_priv_.param("wheel_max_dt", wheel_max_dt_, 0.5);
    nh_priv_.param("wheel_reset_max_delta_xy", wheel_reset_max_delta_xy_, 3.0);
    nh_priv_.param("wheel_reset_max_delta_yaw", wheel_reset_max_delta_yaw_, deg2rad(45.0));
    nh_priv_.param("wheel_good_vxy_var", wheel_good_vxy_var_, 0.05);
    nh_priv_.param("wheel_good_vyaw_var", wheel_good_vyaw_var_, std::pow(deg2rad(1.0), 2.0));
    nh_priv_.param("wheel_degraded_vxy_var", wheel_degraded_vxy_var_, 0.50);
    nh_priv_.param("wheel_degraded_vyaw_var", wheel_degraded_vyaw_var_, std::pow(deg2rad(5.0), 2.0));
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
    fusion_pose_cb_data_ =
      makeCallbackData("fusion_pose", pose_update_vector, fusion_pose_rejection_threshold_);
    fusion_twist_cb_data_ =
      makeCallbackData("fusion_twist", twist_update_vector, fusion_twist_rejection_threshold_);
    wheel_twist_cb_data_ =
      makeCallbackData("wheel_twist", twist_update_vector, wheel_twist_rejection_threshold_);
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

  InputQuality insQuality(const LocDecision &decision) const
  {
    if (!hasPolicy(decision.policy, LocalizationPolicy::USE_GNSS))
    {
      return InputQuality::BAD;
    }

    switch (decision.status)
    {
      case LocalizationStatus::NORMAL:
        return InputQuality::GOOD;
      case LocalizationStatus::NOT_STABLE:
      case LocalizationStatus::SECONDARY:
        return InputQuality::DEGRADED;
      case LocalizationStatus::DR:
      case LocalizationStatus::LOST:
        return InputQuality::BAD;
    }

    return InputQuality::BAD;
  }

  InputQuality fusionQuality(const LocDecision &decision) const
  {
    if (!hasPolicy(decision.policy, LocalizationPolicy::USE_FUSION_ODOM))
    {
      return InputQuality::BAD;
    }

    switch (decision.status)
    {
      case LocalizationStatus::NORMAL:
        return InputQuality::GOOD;
      case LocalizationStatus::NOT_STABLE:
        return InputQuality::DEGRADED;
      case LocalizationStatus::SECONDARY:
      case LocalizationStatus::DR:
      case LocalizationStatus::LOST:
        return InputQuality::BAD;
    }

    return InputQuality::BAD;
  }

  InputQuality wheelImuQuality(const LocDecision &decision) const
  {
    if (decision.policy == static_cast<unsigned long long>(LocalizationPolicy::NO_LOCALIZATION) ||
        decision.status == LocalizationStatus::LOST)
    {
      return InputQuality::BAD;
    }

    switch (decision.status)
    {
      case LocalizationStatus::NORMAL:
      case LocalizationStatus::DR:
        return InputQuality::GOOD;
      case LocalizationStatus::NOT_STABLE:
      case LocalizationStatus::SECONDARY:
        return InputQuality::DEGRADED;
      case LocalizationStatus::LOST:
        return InputQuality::BAD;
    }

    return InputQuality::BAD;
  }

  void applyInsCovariance(nav_msgs::Odometry &odom, const InputQuality quality) const
  {
    clearCovariances(odom);

    switch (quality)
    {
      case InputQuality::GOOD:
        setPoseCov(odom, 0.02, 100.0, 100.0, std::pow(deg2rad(0.5), 2.0));
        setTwistCov(odom, 0.05, 100.0, 100.0, std::pow(deg2rad(0.5), 2.0));
        break;
      case InputQuality::DEGRADED:
        setPoseCov(odom, 1.0, 100.0, 100.0, std::pow(deg2rad(3.0), 2.0));
        setTwistCov(odom, 0.20, 100.0, 100.0, std::pow(deg2rad(2.0), 2.0));
        break;
      case InputQuality::BAD:
        setPoseCov(odom, 100.0, 100.0, 100.0, std::pow(deg2rad(20.0), 2.0));
        setTwistCov(odom, 0.50, 100.0, 100.0, std::pow(deg2rad(5.0), 2.0));
        break;
    }
  }

  void applyFusionCovariance(nav_msgs::Odometry &odom, const InputQuality quality) const
  {
    clearCovariances(odom);

    switch (quality)
    {
      case InputQuality::GOOD:
        setPoseCov(odom, 0.05, 100.0, 100.0, std::pow(deg2rad(1.0), 2.0));
        setTwistCov(odom, 0.20, 100.0, 100.0, std::pow(deg2rad(1.0), 2.0));
        break;
      case InputQuality::DEGRADED:
        setPoseCov(odom, 5.0, 100.0, 100.0, std::pow(deg2rad(5.0), 2.0));
        setTwistCov(odom, 1.0, 100.0, 100.0, std::pow(deg2rad(5.0), 2.0));
        break;
      case InputQuality::BAD:
        break;
    }
  }

  void applyWheelCovariance(
    geometry_msgs::TwistWithCovarianceStamped &twist,
    const InputQuality quality) const
  {
    switch (quality)
    {
      case InputQuality::GOOD:
        setWheelTwistCov(twist, wheel_good_vxy_var_, 100.0, 100.0, wheel_good_vyaw_var_);
        break;
      case InputQuality::DEGRADED:
        setWheelTwistCov(twist, wheel_degraded_vxy_var_, 100.0, 100.0, wheel_degraded_vyaw_var_);
        break;
      case InputQuality::BAD:
        setWheelTwistCov(twist, 100.0, 100.0, 100.0, std::pow(deg2rad(20.0), 2.0));
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

    const InputQuality quality = insQuality(decision);
    ++ins_count_;

    if (quality == InputQuality::BAD)
    {
      ++ins_drop_count_;
      logStatus(decision, quality, fusionQuality(decision), wheelImuQuality(decision));
      return;
    }

    nav_msgs::OdometryPtr out(new nav_msgs::Odometry(*msg));
    normalizeOdomFrame(*out);
    applyInsCovariance(*out, quality);

    ekf_.odometryCallback(out, "ins_odom", ins_pose_cb_data_, ins_twist_cb_data_);
    logStatus(decision, quality, fusionQuality(decision), wheelImuQuality(decision));
  }

  void fusionCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    LocDecision decision;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      decision = currentDecision();
    }

    const InputQuality quality = fusionQuality(decision);
    ++fusion_count_;

    if (quality == InputQuality::BAD)
    {
      ++fusion_drop_count_;
      logStatus(decision, insQuality(decision), quality, wheelImuQuality(decision));
      return;
    }

    nav_msgs::OdometryPtr out(new nav_msgs::Odometry(*msg));
    normalizeOdomFrame(*out);
    applyFusionCovariance(*out, quality);

    ekf_.odometryCallback(out, "fusion_odom", fusion_pose_cb_data_, fusion_twist_cb_data_);
    logStatus(decision, insQuality(decision), quality, wheelImuQuality(decision));
  }

  void wheelCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    LocDecision decision;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      decision = currentDecision();
    }

    const InputQuality quality = wheelImuQuality(decision);
    ++wheel_count_;

    if (quality == InputQuality::BAD)
    {
      ++wheel_drop_count_;
      resetWheelReference(*msg);
      logStatus(decision, insQuality(decision), fusionQuality(decision), quality);
      return;
    }

    if (!have_last_wheel_)
    {
      resetWheelReference(*msg);
      return;
    }

    const double dt = (msg->header.stamp - last_wheel_stamp_).toSec();
    const double dx = msg->pose.pose.position.x - last_wheel_odom_.pose.pose.position.x;
    const double dy = msg->pose.pose.position.y - last_wheel_odom_.pose.pose.position.y;
    const double prev_yaw = yawFromOdom(last_wheel_odom_);
    const double curr_yaw = yawFromOdom(*msg);
    const double dyaw = normalizeAngle(curr_yaw - prev_yaw);
    const double delta_xy = std::hypot(dx, dy);

    if (dt <= 0.0 || dt > wheel_max_dt_ ||
        delta_xy > wheel_reset_max_delta_xy_ ||
        std::fabs(dyaw) > wheel_reset_max_delta_yaw_)
    {
      ++wheel_reset_count_;
      resetWheelReference(*msg);
      logStatus(decision, insQuality(decision), fusionQuality(decision), quality);
      return;
    }

    geometry_msgs::TwistWithCovarianceStampedPtr twist(new geometry_msgs::TwistWithCovarianceStamped());
    twist->header.stamp = msg->header.stamp;
    twist->header.frame_id = child_frame_id_;
    twist->twist.twist.linear.x = (std::cos(prev_yaw) * dx + std::sin(prev_yaw) * dy) / dt;
    twist->twist.twist.linear.y = (-std::sin(prev_yaw) * dx + std::cos(prev_yaw) * dy) / dt;
    twist->twist.twist.angular.z = dyaw / dt;
    applyWheelCovariance(*twist, quality);

    ekf_.twistCallback(twist, wheel_twist_cb_data_, child_frame_id_);
    resetWheelReference(*msg);
    logStatus(decision, insQuality(decision), fusionQuality(decision), quality);
  }

  void imuCb(const sensor_msgs::Imu::ConstPtr &msg)
  {
    LocDecision decision;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      decision = currentDecision();
    }

    const InputQuality quality = wheelImuQuality(decision);
    ++imu_count_;

    if (quality == InputQuality::BAD)
    {
      ++imu_drop_count_;
      logStatus(decision, insQuality(decision), fusionQuality(decision), quality);
      return;
    }

    sensor_msgs::ImuPtr out(new sensor_msgs::Imu(*msg));
    out->header.frame_id = child_frame_id_;
    out->angular_velocity = rotateVector(msg->angular_velocity, imu_to_base_rotation_);
    out->linear_acceleration = rotateVector(msg->linear_acceleration, imu_to_base_rotation_);

    tf2::Quaternion imu_orientation;
    tf2::fromMsg(msg->orientation, imu_orientation);
    tf2::Quaternion base_orientation = imu_orientation * imu_to_base_quat_.inverse();
    base_orientation.normalize();
    out->orientation = tf2::toMsg(base_orientation);

    ekf_.imuCallback(out, "imu", imu_pose_cb_data_, imu_twist_cb_data_, imu_accel_cb_data_);
    logStatus(decision, insQuality(decision), fusionQuality(decision), quality);
  }

  void resetWheelReference(const nav_msgs::Odometry &msg)
  {
    last_wheel_odom_ = msg;
    last_wheel_stamp_ = msg.header.stamp;
    have_last_wheel_ = true;
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
    const InputQuality ins_quality,
    const InputQuality fusion_quality,
    const InputQuality wheel_imu_quality) const
  {
    ROS_INFO_STREAM_THROTTLE(
      1.0,
      "LocEkfMultiSensor state: status=" << statusToString(decision.status)
                                         << " policy=" << decision.policy
                                         << " ins=" << qualityToString(ins_quality)
                                         << " fusion=" << qualityToString(fusion_quality)
                                         << " wheel_imu=" << qualityToString(wheel_imu_quality)
                                         << " ins_in=" << ins_count_
                                         << " ins_drop=" << ins_drop_count_
                                         << " fusion_in=" << fusion_count_
                                         << " fusion_drop=" << fusion_drop_count_
                                         << " wheel_in=" << wheel_count_
                                         << " wheel_drop=" << wheel_drop_count_
                                         << " wheel_reset=" << wheel_reset_count_
                                         << " imu_in=" << imu_count_
                                         << " imu_drop=" << imu_drop_count_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  RobotLocalization::RosEkf ekf_;

  ros::Subscriber ins_sub_;
  ros::Subscriber fusion_sub_;
  ros::Subscriber wheel_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber localization_status_sub_;
  ros::Subscriber loc_policy_sub_;

  mutable std::mutex state_mtx_;
  LocDecision loc_decision_;

  std::string ins_topic_;
  std::string fusion_topic_;
  std::string wheel_odom_topic_;
  std::string imu_topic_;
  std::string localization_status_topic_;
  std::string loc_policy_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  double state_timeout_sec_;
  double ins_pose_rejection_threshold_ = 5.0;
  double ins_twist_rejection_threshold_ = 3.0;
  double fusion_pose_rejection_threshold_ = 3.0;
  double fusion_twist_rejection_threshold_ = 3.0;
  double wheel_twist_rejection_threshold_ = 3.0;
  double imu_pose_rejection_threshold_ = 3.0;
  double imu_twist_rejection_threshold_ = 3.0;
  double imu_accel_rejection_threshold_ = 3.0;

  bool imu_use_orientation_ = false;
  bool imu_use_angular_velocity_ = true;
  bool imu_use_linear_acceleration_ = false;
  double imu_extrinsic_roll_ = 0.0;
  double imu_extrinsic_pitch_ = 0.0;
  double imu_extrinsic_yaw_ = 0.0;
  tf2::Quaternion imu_to_base_quat_;
  tf2::Matrix3x3 imu_to_base_rotation_;

  double wheel_max_dt_ = 0.5;
  double wheel_reset_max_delta_xy_ = 3.0;
  double wheel_reset_max_delta_yaw_ = deg2rad(45.0);
  double wheel_good_vxy_var_ = 0.05;
  double wheel_good_vyaw_var_ = std::pow(deg2rad(1.0), 2.0);
  double wheel_degraded_vxy_var_ = 0.50;
  double wheel_degraded_vyaw_var_ = std::pow(deg2rad(5.0), 2.0);

  RobotLocalization::CallbackData ins_pose_cb_data_;
  RobotLocalization::CallbackData ins_twist_cb_data_;
  RobotLocalization::CallbackData fusion_pose_cb_data_;
  RobotLocalization::CallbackData fusion_twist_cb_data_;
  RobotLocalization::CallbackData wheel_twist_cb_data_;
  RobotLocalization::CallbackData imu_pose_cb_data_;
  RobotLocalization::CallbackData imu_twist_cb_data_;
  RobotLocalization::CallbackData imu_accel_cb_data_;

  bool have_last_wheel_ = false;
  ros::Time last_wheel_stamp_;
  nav_msgs::Odometry last_wheel_odom_;

  uint64_t ins_count_ = 0;
  uint64_t ins_drop_count_ = 0;
  uint64_t fusion_count_ = 0;
  uint64_t fusion_drop_count_ = 0;
  uint64_t wheel_count_ = 0;
  uint64_t wheel_drop_count_ = 0;
  uint64_t wheel_reset_count_ = 0;
  uint64_t imu_count_ = 0;
  uint64_t imu_drop_count_ = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_ekf_multi_sensor_node");
  LocEkfMultiSensorNode node;
  ros::spin();

  return EXIT_SUCCESS;
}
