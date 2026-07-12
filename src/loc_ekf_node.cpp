/*
 * Copyright (c) 2026
 * All rights reserved.
 */

#include "robot_localization/filter_common.h"
#include "robot_localization/ros_filter_types.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>
#include <tf2/utils.h>
#include <udi_msgs/LocalizationEstimate.h>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
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

double standardDeviation(const double variance)
{
  return std::sqrt(std::max(0.0, variance));
}

bool isValidAbsolutePose(const nav_msgs::Odometry &odom)
{
  const geometry_msgs::Point &position = odom.pose.pose.position;
  const geometry_msgs::Quaternion &orientation = odom.pose.pose.orientation;
  const double quaternion_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;

  return !odom.header.stamp.isZero() &&
         std::isfinite(position.x) &&
         std::isfinite(position.y) &&
         std::isfinite(position.z) &&
         std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) &&
         std::isfinite(orientation.w) &&
         quaternion_norm_squared > 1e-12;
}

bool isFiniteOdometry(const nav_msgs::Odometry &odom)
{
  const geometry_msgs::Vector3 &linear = odom.twist.twist.linear;
  const geometry_msgs::Vector3 &angular = odom.twist.twist.angular;
  return isValidAbsolutePose(odom) &&
         std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z) &&
         std::isfinite(angular.x) && std::isfinite(angular.y) && std::isfinite(angular.z);
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

    localization_estimate_pub_ =
      nh_.advertise<udi_msgs::LocalizationEstimate>(
        localization_estimate_topic_, output_queue_size_);
    filtered_odom_pub_ =
      nh_.advertise<nav_msgs::Odometry>("odometry/filtered", output_queue_size_);
    localization_estimate_timer_ = nh_.createTimer(
      ros::Duration(1.0 / localization_estimate_frequency_),
      &LocEkfNode::publishLocalizationEstimate,
      this);

    const ros::TransportHints low_latency_hints = ros::TransportHints().tcpNoDelay(true);
    ins_sub_ = nh_.subscribe(
      ins_topic_, input_queue_size_, &LocEkfNode::insCb, this, low_latency_hints);
    fusion_sub_ = nh_.subscribe(
      fusion_topic_, input_queue_size_, &LocEkfNode::fusionCb, this, low_latency_hints);
    localization_status_sub_ = nh_.subscribe(
      localization_status_topic_, 5, &LocEkfNode::localizationStatusCb, this, low_latency_hints);
    loc_policy_sub_ = nh_.subscribe(
      loc_policy_topic_, 5, &LocEkfNode::locPolicyCb, this, low_latency_hints);
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
    nh_priv_.param(
      "localization_estimate_topic", localization_estimate_topic_,
      std::string("/localization_estimate1"));
    nh_priv_.param(
      "localization_estimate_frequency", localization_estimate_frequency_, 100.0);
    nh_priv_.param("input_queue_size", input_queue_size_, 5);
    nh_priv_.param("output_queue_size", output_queue_size_, 5);
    nh_priv_.param("max_input_age_sec", max_input_age_sec_, 0.5);
    nh_priv_.param("max_future_stamp_sec", max_future_stamp_sec_, 0.05);
    nh_priv_.param("hard_stale_warn_sec", hard_stale_warn_sec_, 1.0);
    nh_priv_.param("fusion_primary_xy_variance", fusion_primary_xy_variance_, 0.0005);
    nh_priv_.param("fusion_primary_vxy_variance", fusion_primary_vxy_variance_, 0.005);
    nh_priv_.param("fusion_primary_yaw_std_deg", fusion_primary_yaw_std_deg_, 0.5);
    nh_priv_.param("fusion_primary_vyaw_std_deg", fusion_primary_vyaw_std_deg_, 0.5);
    input_queue_size_ = std::max(1, input_queue_size_);
    output_queue_size_ = std::max(1, output_queue_size_);
    if (localization_estimate_frequency_ <= 0.0)
    {
      ROS_WARN("localization_estimate_frequency must be positive; using 100 Hz");
      localization_estimate_frequency_ = 100.0;
    }
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

  bool validateInput(
    const nav_msgs::Odometry &odom,
    const char *source_name,
    std::atomic<uint64_t> &invalid_count,
    std::atomic<uint64_t> &stale_count,
    std::atomic<double> &last_age_sec) const
  {
    if (!isFiniteOdometry(odom))
    {
      ++invalid_count;
      ROS_WARN_STREAM_THROTTLE(1.0, "Dropping invalid " << source_name << " odometry");
      return false;
    }

    const double age_sec = (ros::Time::now() - odom.header.stamp).toSec();
    last_age_sec.store(age_sec, std::memory_order_relaxed);
    if (age_sec > max_input_age_sec_)
    {
      ++stale_count;
      ROS_WARN_STREAM_THROTTLE(
        1.0, "Dropping stale " << source_name << ": age=" << age_sec << " s");
      return false;
    }
    if (age_sec < -max_future_stamp_sec_)
    {
      ++invalid_count;
      ROS_WARN_STREAM_THROTTLE(
        1.0, "Dropping future-dated " << source_name << ": age=" << age_sec << " s");
      return false;
    }
    return true;
  }

  bool initializeFromAbsolutePose(
    const nav_msgs::Odometry &odom,
    const std::string &source_name)
  {
    if (!isValidAbsolutePose(odom))
    {
      ROS_WARN_STREAM_THROTTLE(
        1.0,
        "Waiting for a valid absolute pose from " << source_name
                                                   << " before initializing EKF");
      return false;
    }

    geometry_msgs::PoseWithCovarianceStampedPtr initial_pose(
      new geometry_msgs::PoseWithCovarianceStamped());
    initial_pose->header = odom.header;
    initial_pose->pose = odom.pose;
    ekf_.setPoseCallback(initial_pose);
    absolute_pose_initialized_ = true;

    ROS_INFO_STREAM(
      "Initialized EKF from " << source_name
                               << " absolute pose at stamp=" << odom.header.stamp
                               << " x=" << odom.pose.pose.position.x
                               << " y=" << odom.pose.pose.position.y);
    return true;
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

    // SECONDARY + USE_GNSS means localization_switcher has rejected the
    // /lio_loc_result primary source and selected trusted INS. Do not keep a
    // weak LIO observation in the EKF: a slow LIO drift can otherwise keep
    // pulling final_odom away from the selected INS source.
    if (decision.status == LocalizationStatus::SECONDARY &&
        hasPolicy(decision.policy, LocalizationPolicy::USE_GNSS))
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
        setPoseCov(
          odom, fusion_primary_xy_variance_, 100.0, 100.0,
          std::pow(deg2rad(fusion_primary_yaw_std_deg_), 2.0));
        setTwistCov(
          odom, fusion_primary_vxy_variance_, 100.0, 100.0,
          std::pow(deg2rad(fusion_primary_vyaw_std_deg_), 2.0));
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

  bool initializeIfNeeded(
    const nav_msgs::Odometry &odom,
    const std::string &source_name,
    const bool is_ins,
    bool &initialized_now)
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    initialized_now = false;
    if (absolute_pose_initialized_)
    {
      return true;
    }

    nav_msgs::Odometry initial_pose(odom);
    // Initialization must not depend on startup status/policy messages. Use
    // the trusted absolute-pose covariance for the selected source.
    if (is_ins)
    {
      applyInsCovariance(initial_pose, SourceMode::PRIMARY);
    }
    else
    {
      applyFusionCovariance(initial_pose, SourceMode::PRIMARY);
    }

    initialized_now = initializeFromAbsolutePose(initial_pose, source_name);
    return initialized_now;
  }

  void insCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (!msg || !validateInput(
        *msg, "INS", ins_invalid_count_, ins_stale_count_, last_ins_age_sec_))
    {
      return;
    }
    last_ins_accepted_stamp_sec_.store(msg->header.stamp.toSec(), std::memory_order_relaxed);
    nav_msgs::OdometryPtr out(new nav_msgs::Odometry(*msg));
    normalizeOdomFrame(*out);

    bool initialized_now = false;
    if (!initializeIfNeeded(*out, "INS", true, initialized_now))
    {
      return;
    }
    if (initialized_now)
    {
      return;
    }

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

    applyInsCovariance(*out, ins_mode);

    ekf_.odometryCallback(out, "ins_odom", ins_pose_cb_data_, ins_twist_cb_data_);
    logStatus(decision, ins_mode, fusion_mode);
  }

  void fusionCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (!msg || !validateInput(
        *msg, "LIO", fusion_invalid_count_, fusion_stale_count_, last_fusion_age_sec_))
    {
      return;
    }
    last_fusion_accepted_stamp_sec_.store(msg->header.stamp.toSec(), std::memory_order_relaxed);
    nav_msgs::OdometryPtr out(new nav_msgs::Odometry(*msg));
    normalizeOdomFrame(*out);

    bool initialized_now = false;
    if (!initializeIfNeeded(*out, "LIO", false, initialized_now))
    {
      return;
    }
    if (initialized_now)
    {
      return;
    }

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

  void publishLocalizationEstimate(const ros::TimerEvent &)
  {
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    const ros::Time cycle_time = ros::Time::now();
    if (!last_cycle_time_.isZero() && cycle_time < last_cycle_time_)
    {
      // rosbag loops and repeated simulated-time tests legitimately jump back.
      // RosFilter resets its history; reset the wrapper's monotonic guard too.
      last_output_stamp_ = ros::Time();
    }
    last_cycle_time_ = cycle_time;
    nav_msgs::Odometry odom;
    geometry_msgs::AccelWithCovarianceStamped acceleration;
    if (!ekf_.processMeasurementsAndGetState(cycle_time, odom, &acceleration))
    {
      return;
    }

    if (!last_output_stamp_.isZero() && odom.header.stamp < last_output_stamp_)
    {
      ++output_regression_count_;
      ROS_ERROR_STREAM_THROTTLE(1.0, "Suppressing time-regressing final_odom");
      return;
    }
    last_output_stamp_ = odom.header.stamp;
    const bool has_acceleration = true;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    tf2::getEulerYPR(odom.pose.pose.orientation, yaw, pitch, roll);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    // nav_msgs/Odometry twist is expressed in child_frame_id. Convert it to
    // the world frame for the corresponding UDI pose fields.
    const double vx_vrf = odom.twist.twist.linear.x;
    const double vy_vrf = odom.twist.twist.linear.y;
    const double vz_vrf = odom.twist.twist.linear.z;
    const double vx_map = cos_yaw * vx_vrf - sin_yaw * vy_vrf;
    const double vy_map = sin_yaw * vx_vrf + cos_yaw * vy_vrf;

    udi_msgs::LocalizationEstimate estimate;
    estimate.header.timestamp_sec = odom.header.stamp.toSec();
    estimate.header.sequence_num = localization_estimate_sequence_++;
    estimate.header.frame_id = odom.header.frame_id;

    estimate.pose.position.x = odom.pose.pose.position.x;
    estimate.pose.position.y = odom.pose.pose.position.y;
    estimate.pose.position.z = odom.pose.pose.position.z;
    estimate.pose.orientation.qx = odom.pose.pose.orientation.x;
    estimate.pose.orientation.qy = odom.pose.pose.orientation.y;
    estimate.pose.orientation.qz = odom.pose.pose.orientation.z;
    estimate.pose.orientation.qw = odom.pose.pose.orientation.w;
    estimate.pose.heading = yaw;
    estimate.pose.euler_angles.x = yaw;
    estimate.pose.euler_angles.y = pitch;
    estimate.pose.euler_angles.z = roll;

    estimate.pose.linear_velocity.x = vx_map;
    estimate.pose.linear_velocity.y = vy_map;
    estimate.pose.linear_velocity.z = vz_vrf;
    estimate.pose.linear_velocity_vrf.x = vx_vrf;
    estimate.pose.linear_velocity_vrf.y = vy_vrf;
    estimate.pose.linear_velocity_vrf.z = vz_vrf;

    estimate.pose.angular_velocity.x = odom.twist.twist.angular.x;
    estimate.pose.angular_velocity.y = odom.twist.twist.angular.y;
    estimate.pose.angular_velocity.z = odom.twist.twist.angular.z;
    estimate.pose.angular_velocity_vrf = estimate.pose.angular_velocity;

    if (has_acceleration)
    {
      const double ax_vrf = acceleration.accel.accel.linear.x;
      const double ay_vrf = acceleration.accel.accel.linear.y;
      const double az_vrf = acceleration.accel.accel.linear.z;
      estimate.pose.linear_acceleration.x = cos_yaw * ax_vrf - sin_yaw * ay_vrf;
      estimate.pose.linear_acceleration.y = sin_yaw * ax_vrf + cos_yaw * ay_vrf;
      estimate.pose.linear_acceleration.z = az_vrf;
      estimate.pose.linear_acceleration_vrf.x = ax_vrf;
      estimate.pose.linear_acceleration_vrf.y = ay_vrf;
      estimate.pose.linear_acceleration_vrf.z = az_vrf;
      estimate.pose.angular_velocity_vrf.x = acceleration.accel.accel.angular.x;
      estimate.pose.angular_velocity_vrf.y = acceleration.accel.accel.angular.y;
      estimate.pose.angular_velocity_vrf.z = acceleration.accel.accel.angular.z;
    }

    estimate.uncertainty.position_std_dev.x = standardDeviation(odom.pose.covariance[0]);
    estimate.uncertainty.position_std_dev.y = standardDeviation(odom.pose.covariance[7]);
    estimate.uncertainty.position_std_dev.z = standardDeviation(odom.pose.covariance[14]);
    estimate.uncertainty.orientation_std_dev.x = standardDeviation(odom.pose.covariance[21]);
    estimate.uncertainty.orientation_std_dev.y = standardDeviation(odom.pose.covariance[28]);
    estimate.uncertainty.orientation_std_dev.z = standardDeviation(odom.pose.covariance[35]);
    estimate.uncertainty.linear_velocity_std_dev.x = standardDeviation(odom.twist.covariance[0]);
    estimate.uncertainty.linear_velocity_std_dev.y = standardDeviation(odom.twist.covariance[7]);
    estimate.uncertainty.linear_velocity_std_dev.z = standardDeviation(odom.twist.covariance[14]);
    estimate.uncertainty.angular_velocity_std_dev.x = standardDeviation(odom.twist.covariance[21]);
    estimate.uncertainty.angular_velocity_std_dev.y = standardDeviation(odom.twist.covariance[28]);
    estimate.uncertainty.angular_velocity_std_dev.z = standardDeviation(odom.twist.covariance[35]);

    filtered_odom_pub_.publish(odom);

    if (has_acceleration)
    {
      estimate.uncertainty.linear_acceleration_std_dev.x =
        standardDeviation(acceleration.accel.covariance[0]);
      estimate.uncertainty.linear_acceleration_std_dev.y =
        standardDeviation(acceleration.accel.covariance[7]);
      estimate.uncertainty.linear_acceleration_std_dev.z =
        standardDeviation(acceleration.accel.covariance[14]);
    }

    localization_estimate_pub_.publish(estimate);

    ++output_count_;
    const double execution_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    last_execution_ms_.store(execution_ms, std::memory_order_relaxed);
    if (execution_ms > 1000.0 / localization_estimate_frequency_)
    {
      ++loop_overrun_count_;
    }
    logRuntimeStatus(cycle_time, odom);
  }

  void logRuntimeStatus(const ros::Time &now, const nav_msgs::Odometry &odom)
  {
    const double ins_stamp = last_ins_accepted_stamp_sec_.load(std::memory_order_relaxed);
    const double fusion_stamp = last_fusion_accepted_stamp_sec_.load(std::memory_order_relaxed);
    const double newest_stamp = std::max(ins_stamp, fusion_stamp);
    const double accepted_age = newest_stamp > 0.0 ? now.toSec() - newest_stamp : -1.0;
    if (accepted_age > hard_stale_warn_sec_)
    {
      ROS_ERROR_STREAM_THROTTLE(
        1.0, "All accepted final_odom inputs are hard-stale: age=" << accepted_age << " s");
    }

    ROS_INFO_STREAM_THROTTLE(
      1.0, "final_odom runtime: output_age=" << (now - odom.header.stamp).toSec()
        << " ins_age=" << last_ins_age_sec_.load(std::memory_order_relaxed)
        << " lio_age=" << last_fusion_age_sec_.load(std::memory_order_relaxed)
        << " loop_ms=" << last_execution_ms_.load(std::memory_order_relaxed)
        << " outputs=" << output_count_.load()
        << " overruns=" << loop_overrun_count_.load()
        << " stamp_regressions=" << output_regression_count_.load()
        << " ins_invalid/stale=" << ins_invalid_count_.load() << "/" << ins_stale_count_.load()
        << " lio_invalid/stale=" << fusion_invalid_count_.load() << "/" << fusion_stale_count_.load());
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
  ros::Publisher localization_estimate_pub_;
  ros::Publisher filtered_odom_pub_;
  ros::Timer localization_estimate_timer_;

  mutable std::mutex state_mtx_;
  LocDecision loc_decision_;
  bool absolute_pose_initialized_ = false;

  std::string ins_topic_;
  std::string fusion_topic_;
  std::string localization_status_topic_;
  std::string loc_policy_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string localization_estimate_topic_;

  double state_timeout_sec_;
  double localization_estimate_frequency_ = 100.0;
  int input_queue_size_ = 5;
  int output_queue_size_ = 5;
  double max_input_age_sec_ = 0.5;
  double max_future_stamp_sec_ = 0.05;
  double hard_stale_warn_sec_ = 1.0;
  double fusion_primary_xy_variance_ = 0.0005;
  double fusion_primary_vxy_variance_ = 0.005;
  double fusion_primary_yaw_std_deg_ = 0.5;
  double fusion_primary_vyaw_std_deg_ = 0.5;
  double ins_pose_rejection_threshold_ = 5.0;
  double ins_twist_rejection_threshold_ = 3.0;
  double fusion_pose_rejection_threshold_ = 3.0;
  double fusion_twist_rejection_threshold_ = 3.0;

  RobotLocalization::CallbackData ins_pose_cb_data_;
  RobotLocalization::CallbackData ins_twist_cb_data_;
  RobotLocalization::CallbackData fusion_pose_cb_data_;
  RobotLocalization::CallbackData fusion_twist_cb_data_;

  std::atomic<uint64_t> ins_count_{0};
  std::atomic<uint64_t> ins_drop_count_{0};
  std::atomic<uint64_t> fusion_count_{0};
  std::atomic<uint64_t> fusion_drop_count_{0};
  std::atomic<uint64_t> ins_invalid_count_{0};
  std::atomic<uint64_t> ins_stale_count_{0};
  std::atomic<uint64_t> fusion_invalid_count_{0};
  std::atomic<uint64_t> fusion_stale_count_{0};
  std::atomic<uint64_t> output_count_{0};
  std::atomic<uint64_t> loop_overrun_count_{0};
  std::atomic<uint64_t> output_regression_count_{0};
  std::atomic<double> last_ins_age_sec_{-1.0};
  std::atomic<double> last_fusion_age_sec_{-1.0};
  std::atomic<double> last_ins_accepted_stamp_sec_{0.0};
  std::atomic<double> last_fusion_accepted_stamp_sec_{0.0};
  std::atomic<double> last_execution_ms_{0.0};
  ros::Time last_output_stamp_;
  ros::Time last_cycle_time_;
  uint32_t localization_estimate_sequence_ = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_ekf_node");
  LocEkfNode node;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return EXIT_SUCCESS;
}
