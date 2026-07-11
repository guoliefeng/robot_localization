/*
 * Copyright (c) 2026
 * All rights reserved.
 */

#include "robot_localization/gnss_quality_monitor.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace RobotLocalization
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

int clampScore(const int score)
{
  return std::max(0, std::min(100, score));
}
}  // namespace

GnssQualityMonitor::GnssQualityMonitor()
  : enabled_(true),
    debug_log_(false),
    publish_debug_(false),
    devpvt_topic_("/chcnav/devpvt"),
    debug_topic_("gnss_quality_debug"),
    state_(State::GNSS_NO_DATA),
    state_enter_time_(0),
    no_data_recovery_until_(0),
    last_debug_publish_time_(0),
    has_sample_(false),
    has_last_warning_(false),
    last_warning_(0),
    last_transition_reason_("no devpvt received")
{
  last_covariance_.sigma_yaw = degToRad(params_.no_data_sigma_yaw_deg);
  last_covariance_.cov_yaw = last_covariance_.sigma_yaw * last_covariance_.sigma_yaw;
}

void GnssQualityMonitor::initialize(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
{
  std::lock_guard<std::mutex> lock(mutex_);
  nh_ = nh;
  nh_priv_ = nh_priv;
  loadParams();

  if (enabled_)
  {
    devpvt_sub_ = nh_.subscribe(devpvt_topic_, 100, &GnssQualityMonitor::devpvtCallback, this);
    if (publish_debug_)
    {
      debug_pub_ = nh_priv_.advertise<std_msgs::String>(debug_topic_, 10);
    }

    ROS_INFO_STREAM("GNSS quality monitor enabled. devpvt_topic=" << devpvt_topic_);
  }
  else
  {
    devpvt_sub_.shutdown();
    debug_pub_.shutdown();
    ROS_INFO_STREAM("GNSS quality monitor disabled.");
  }
}

void GnssQualityMonitor::loadParams()
{
  paramWithFallback("enabled", enabled_, true);
  paramWithFallback("debug", debug_log_, false);
  paramWithFallback("publish_debug", publish_debug_, false);
  paramWithFallback("devpvt_topic", devpvt_topic_, std::string("/chcnav/devpvt"));
  paramWithFallback("debug_topic", debug_topic_, std::string("gnss_quality_debug"));

  paramWithFallback("devpvt_timeout", params_.devpvt_timeout, 0.2);
  paramWithFallback("good_to_degraded_time", params_.good_to_degraded_time, 0.5);
  paramWithFallback("degraded_to_bad_time", params_.degraded_to_bad_time, 0.3);
  paramWithFallback("bad_to_degraded_time", params_.bad_to_degraded_time, 1.0);
  paramWithFallback("degraded_to_good_time", params_.degraded_to_good_time, 2.0);
  paramWithFallback("stat1_float_bad_duration", params_.stat1_float_bad_duration, 10.0);
  paramWithFallback("ns2_zero_bad_duration", params_.ns2_zero_bad_duration, 1.0);
  paramWithFallback("min_good_sigma_xy", params_.min_good_sigma_xy, 0.05);
  paramWithFallback("min_degraded_sigma_xy", params_.min_degraded_sigma_xy, 0.15);
  paramWithFallback("min_stat1_float_sigma_xy", params_.min_stat1_float_sigma_xy, 0.20);
  paramWithFallback("min_bad_sigma_xy", params_.min_bad_sigma_xy, 0.50);
  paramWithFallback("no_data_sigma_xy", params_.no_data_sigma_xy, 5.0);
  paramWithFallback("min_good_sigma_yaw_deg", params_.min_good_sigma_yaw_deg, 0.5);
  paramWithFallback("min_degraded_sigma_yaw_deg", params_.min_degraded_sigma_yaw_deg, 2.0);
  paramWithFallback("min_stat1_float_sigma_yaw_deg", params_.min_stat1_float_sigma_yaw_deg, 3.0);
  paramWithFallback("min_bad_sigma_yaw_deg", params_.min_bad_sigma_yaw_deg, 10.0);
  paramWithFallback("no_data_sigma_yaw_deg", params_.no_data_sigma_yaw_deg, 30.0);

  paramWithFallback("stat1_float_to_degraded_time", params_.stat1_float_to_degraded_time, 0.3);
  paramWithFallback("stat1_float_age_bad_time", params_.stat1_float_age_bad_time, 0.5);
  paramWithFallback("no_data_recovery_observe_time", params_.no_data_recovery_observe_time, 1.0);
  paramWithFallback("debug_publish_period", params_.debug_publish_period, 0.1);
}

bool GnssQualityMonitor::applyToOdometry(nav_msgs::Odometry &odom)
{
  std::string debug_string;
  ros::Time now = odom.header.stamp;
  if (now.isZero())
  {
    now = ros::Time::now();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_)
    {
      return false;
    }

    updateNoDataState(now);
    last_covariance_ = computeCovariance();

    odom.pose.covariance[0] = last_covariance_.cov_x;
    odom.pose.covariance[7] = last_covariance_.cov_y;
    odom.pose.covariance[35] = last_covariance_.cov_yaw;

    debug_string = buildDebugString(last_covariance_);
    publishDebugIfNeeded(now, debug_string);
  }

  if (debug_log_)
  {
    ROS_INFO_STREAM_THROTTLE(0.5, debug_string);
  }

  return true;
}

const char *GnssQualityMonitor::stateToString(const State state)
{
  switch (state)
  {
    case State::GNSS_GOOD:
      return "GNSS_GOOD";
    case State::GNSS_DEGRADED:
      return "GNSS_DEGRADED";
    case State::GNSS_BAD:
      return "GNSS_BAD";
    case State::GNSS_NO_DATA:
      return "GNSS_NO_DATA";
  }

  return "GNSS_UNKNOWN";
}

void GnssQualityMonitor::devpvtCallback(const chcnav_msgs::hcinspvatzcb::ConstPtr &msg)
{
  std::string debug_string;
  ros::Time now = msg->header.stamp;
  if (now.isZero())
  {
    now = ros::Time::now();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    sample_ = makeSample(*msg);
    sample_.stamp = now;
    sample_.score = computeScore(sample_);

    has_sample_ = true;
    last_devpvt_stamp_ = now;

    updateStateFromSample(now);
    last_covariance_ = computeCovariance();

    debug_string = buildDebugString(last_covariance_);
    publishDebugIfNeeded(now, debug_string);
  }

  if (debug_log_)
  {
    ROS_INFO_STREAM_THROTTLE(0.5, debug_string);
  }
}

GnssQualityMonitor::Sample GnssQualityMonitor::makeSample(const chcnav_msgs::hcinspvatzcb &msg)
{
  Sample sample;
  sample.stamp = msg.header.stamp;
  sample.stat0 = msg.stat[0];
  sample.stat1 = msg.stat[1];
  sample.ns = msg.ns;
  sample.ns2 = msg.ns2;
  sample.age = msg.age;
  sample.hdop = msg.hdop;
  sample.pdop = msg.pdop;
  sample.gdop = msg.gdop;
  sample.pos_std_h = std::hypot(msg.position_stdev[0], msg.position_stdev[1]);
  sample.yaw_std_deg = msg.euler_stdev[2];
  sample.warning = msg.warning;
  sample.warning_changed = has_last_warning_ && msg.warning != last_warning_;
  last_warning_ = msg.warning;
  has_last_warning_ = true;
  return sample;
}

int GnssQualityMonitor::computeScore(const Sample &sample) const
{
  int score = 100;

  if (sample.stat0 != 2)
  {
    score -= 50;
  }

  if (sample.stat1 == 5)
  {
    score -= 25;
  }
  else if (sample.stat1 != 4)
  {
    score -= 60;
  }

  if (sample.ns < 40)
  {
    score -= 10;
  }
  if (sample.ns < 35)
  {
    score -= 20;
  }

  if (sample.ns2 == 0)
  {
    score -= 25;
  }
  else if (sample.ns2 < 35)
  {
    score -= 15;
  }

  if (sample.age > 2.0)
  {
    score -= 10;
  }
  if (sample.age > 5.0)
  {
    score -= 20;
  }
  if (sample.age > 10.0)
  {
    score -= 30;
  }

  if (sample.pos_std_h > 0.05)
  {
    score -= 10;
  }
  if (sample.pos_std_h > 0.10)
  {
    score -= 25;
  }

  if (sample.yaw_std_deg > 0.03)
  {
    score -= 10;
  }
  if (sample.yaw_std_deg > 0.10)
  {
    score -= 25;
  }

  if (sample.hdop > 1.0 || sample.pdop > 2.5 || sample.gdop > 4.0)
  {
    score -= 5;
  }
  if (sample.hdop > 1.5 || sample.pdop > 4.0 || sample.gdop > 6.0)
  {
    score -= 15;
  }

  if (sample.warning_changed)
  {
    score -= 10;
  }

  return clampScore(score);
}

void GnssQualityMonitor::updateStateFromSample(const ros::Time &now)
{
  if (state_ == State::GNSS_NO_DATA)
  {
    no_data_recovery_until_ = now + ros::Duration(params_.no_data_recovery_observe_time);
    resetConditionTimers();
    transitionTo(State::GNSS_DEGRADED, "devpvt restored, observe before GOOD", now);
  }

  const bool stat1_float = sample_.stat1 == 5;
  const bool stat1_other = sample_.stat1 != 4 && sample_.stat1 != 5;
  const bool ns2_zero = sample_.ns2 == 0;
  const bool score_lt_80 = sample_.score < 80;
  const bool score_lt_40 = sample_.score < 40;
  const bool float_age_gt_10 = stat1_float && sample_.age > 10.0;
  const bool good_candidate =
    sample_.stat0 == 2 &&
    sample_.stat1 == 4 &&
    sample_.score >= 85 &&
    sample_.ns2 > 35 &&
    sample_.age < 2.0 &&
    sample_.pos_std_h < 0.05 &&
    sample_.yaw_std_deg < 0.03;
  const bool bad_recovery_candidate =
    sample_.stat0 == 2 &&
    (sample_.stat1 == 4 || sample_.stat1 == 5) &&
    sample_.score >= 40;

  const bool float_held_for_bad =
    held(stat1_float, stat1_float_start_, now, params_.stat1_float_bad_duration);
  const bool float_held_for_degraded =
    elapsedSince(stat1_float_start_, now) >= params_.stat1_float_to_degraded_time;
  const bool score_lt_80_held =
    held(score_lt_80, score_lt_80_start_, now, params_.good_to_degraded_time);
  const bool ns2_zero_held =
    held(ns2_zero, ns2_zero_start_, now, params_.good_to_degraded_time);
  const bool stat1_other_held =
    held(stat1_other, stat1_other_start_, now, params_.degraded_to_bad_time);
  const bool float_age_gt_10_held =
    held(float_age_gt_10, float_age_gt_10_start_, now, params_.stat1_float_age_bad_time);
  const bool ns2_zero_for_bad =
    stat1_float && elapsedSince(ns2_zero_start_, now) >= params_.ns2_zero_bad_duration;
  const bool score_lt_40_held =
    held(score_lt_40, score_lt_40_start_, now, params_.degraded_to_bad_time);
  const bool good_candidate_held =
    held(good_candidate, good_candidate_start_, now, params_.degraded_to_good_time);
  const bool bad_recovery_held =
    held(bad_recovery_candidate, bad_recovery_candidate_start_, now, params_.bad_to_degraded_time);

  const bool float_bad_std =
    stat1_float &&
    sample_.pos_std_h > 0.10 &&
    sample_.yaw_std_deg > 0.10;

  bool bad_condition = false;
  std::string bad_reason;
  if (sample_.stat0 != 2)
  {
    bad_condition = true;
    bad_reason = "stat0 != 2";
  }
  else if (stat1_other_held)
  {
    bad_condition = true;
    bad_reason = "stat1 neither fixed nor float for degraded_to_bad_time";
  }
  else if (float_held_for_bad)
  {
    bad_condition = true;
    bad_reason = "stat1 float duration exceeded";
  }
  else if (float_age_gt_10_held)
  {
    bad_condition = true;
    bad_reason = "stat1 float with age > 10s";
  }
  else if (ns2_zero_for_bad)
  {
    bad_condition = true;
    bad_reason = "stat1 float with ns2 == 0";
  }
  else if (float_bad_std)
  {
    bad_condition = true;
    bad_reason = "stat1 float with high position and yaw std";
  }
  else if (score_lt_40_held)
  {
    bad_condition = true;
    bad_reason = "score < 40 for degraded_to_bad_time";
  }

  if (bad_condition)
  {
    transitionTo(State::GNSS_BAD, bad_reason, now);
    return;
  }

  switch (state_)
  {
    case State::GNSS_GOOD:
      if (float_held_for_degraded)
      {
        transitionTo(State::GNSS_DEGRADED, "stat1 float for 0.3s", now);
      }
      else if (score_lt_80_held)
      {
        transitionTo(State::GNSS_DEGRADED, "score < 80 for good_to_degraded_time", now);
      }
      else if (ns2_zero_held)
      {
        transitionTo(State::GNSS_DEGRADED, "ns2 == 0 for good_to_degraded_time", now);
      }
      break;

    case State::GNSS_DEGRADED:
      if (good_candidate_held && now >= no_data_recovery_until_)
      {
        transitionTo(State::GNSS_GOOD, "GOOD criteria held for degraded_to_good_time", now);
      }
      break;

    case State::GNSS_BAD:
      if (bad_recovery_held)
      {
        transitionTo(State::GNSS_DEGRADED, "fixed/float and score >= 40 recovered", now);
      }
      break;

    case State::GNSS_NO_DATA:
      break;
  }
}

void GnssQualityMonitor::updateNoDataState(const ros::Time &now)
{
  if (!has_sample_)
  {
    transitionTo(State::GNSS_NO_DATA, "no devpvt received", now);
    return;
  }

  if (last_devpvt_stamp_.isZero())
  {
    transitionTo(State::GNSS_NO_DATA, "devpvt stamp is zero", now);
    return;
  }

  const double dt = (now - last_devpvt_stamp_).toSec();
  if (dt > params_.devpvt_timeout)
  {
    std::ostringstream reason;
    reason << "devpvt timeout dt=" << std::fixed << std::setprecision(3) << dt << "s";
    transitionTo(State::GNSS_NO_DATA, reason.str(), now);
    resetConditionTimers();
  }
}

GnssQualityMonitor::CovarianceOutput GnssQualityMonitor::computeCovariance() const
{
  CovarianceOutput output;
  double sigma_xy = params_.no_data_sigma_xy;
  double sigma_yaw = degToRad(params_.no_data_sigma_yaw_deg);

  if (state_ == State::GNSS_GOOD)
  {
    sigma_xy = std::max(params_.min_good_sigma_xy, 1.5 * sample_.pos_std_h);
    sigma_yaw = std::max(degToRad(params_.min_good_sigma_yaw_deg), 5.0 * degToRad(sample_.yaw_std_deg));
  }
  else if (state_ == State::GNSS_DEGRADED)
  {
    sigma_xy = std::max(params_.min_degraded_sigma_xy, 3.0 * sample_.pos_std_h);
    sigma_yaw = std::max(degToRad(params_.min_degraded_sigma_yaw_deg), 15.0 * degToRad(sample_.yaw_std_deg));
  }
  else if (state_ == State::GNSS_BAD)
  {
    sigma_xy = std::max(params_.min_bad_sigma_xy, 8.0 * sample_.pos_std_h);
    sigma_yaw = std::max(degToRad(params_.min_bad_sigma_yaw_deg), 50.0 * degToRad(sample_.yaw_std_deg));
  }

  if (has_sample_)
  {
    if (sample_.stat1 == 5)
    {
      sigma_xy = std::max(sigma_xy, params_.min_stat1_float_sigma_xy);
      sigma_yaw = std::max(sigma_yaw, degToRad(params_.min_stat1_float_sigma_yaw_deg));
    }

    if (sample_.ns2 == 0)
    {
      sigma_yaw = std::max(sigma_yaw, degToRad(2.0));
    }

    if (sample_.age > 5.0)
    {
      sigma_xy *= 1.5;
      sigma_yaw *= 1.5;
    }
    if (sample_.age > 10.0)
    {
      sigma_xy *= 2.0;
      sigma_yaw *= 2.0;
    }
  }

  output.sigma_xy = sigma_xy;
  output.sigma_yaw = sigma_yaw;
  output.cov_x = sigma_xy * sigma_xy;
  output.cov_y = output.cov_x;
  output.cov_yaw = sigma_yaw * sigma_yaw;
  return output;
}

std::string GnssQualityMonitor::buildDebugString(const CovarianceOutput &covariance) const
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6)
         << "state=" << stateToString(state_)
         << " score=" << (has_sample_ ? sample_.score : -1)
         << " stat0=" << (has_sample_ ? static_cast<int>(sample_.stat0) : -1)
         << " stat1=" << (has_sample_ ? static_cast<int>(sample_.stat1) : -1)
         << " ns=" << (has_sample_ ? static_cast<int>(sample_.ns) : -1)
         << " ns2=" << (has_sample_ ? static_cast<int>(sample_.ns2) : -1)
         << " age=" << (has_sample_ ? sample_.age : 0.0)
         << " pos_std_h=" << (has_sample_ ? sample_.pos_std_h : 0.0)
         << " yaw_std_deg=" << (has_sample_ ? sample_.yaw_std_deg : 0.0)
         << " sigma_xy=" << covariance.sigma_xy
         << " sigma_yaw_rad=" << covariance.sigma_yaw
         << " sigma_yaw_deg=" << covariance.sigma_yaw / kDegToRad
         << " cov_x=" << covariance.cov_x
         << " cov_y=" << covariance.cov_y
         << " cov_yaw=" << covariance.cov_yaw
         << " reason=\"" << last_transition_reason_ << "\"";
  return stream.str();
}

void GnssQualityMonitor::publishDebugIfNeeded(const ros::Time &now, const std::string &debug_string)
{
  if (!publish_debug_ || !debug_pub_)
  {
    return;
  }

  if (!last_debug_publish_time_.isZero() &&
      (now - last_debug_publish_time_).toSec() < params_.debug_publish_period)
  {
    return;
  }

  std_msgs::String msg;
  msg.data = debug_string;
  debug_pub_.publish(msg);
  last_debug_publish_time_ = now;
}

void GnssQualityMonitor::transitionTo(const State state, const std::string &reason, const ros::Time &now)
{
  if (state_ == state)
  {
    last_transition_reason_ = reason;
    return;
  }

  const State old_state = state_;
  state_ = state;
  state_enter_time_ = now;
  last_transition_reason_ = reason;

  ROS_WARN_STREAM("GNSS quality state changed " << stateToString(old_state) << " -> " <<
                  stateToString(state_) << ": " << reason);
}

void GnssQualityMonitor::resetConditionTimers()
{
  stat1_float_start_ = ros::Time(0);
  score_lt_80_start_ = ros::Time(0);
  ns2_zero_start_ = ros::Time(0);
  stat1_other_start_ = ros::Time(0);
  float_age_gt_10_start_ = ros::Time(0);
  score_lt_40_start_ = ros::Time(0);
  good_candidate_start_ = ros::Time(0);
  bad_recovery_candidate_start_ = ros::Time(0);
}

bool GnssQualityMonitor::held(bool condition,
                              ros::Time &start_time,
                              const ros::Time &now,
                              const double duration) const
{
  if (!condition)
  {
    start_time = ros::Time(0);
    return false;
  }

  if (start_time.isZero())
  {
    start_time = now;
  }

  return elapsedSince(start_time, now) >= duration;
}

double GnssQualityMonitor::elapsedSince(const ros::Time &start_time, const ros::Time &now) const
{
  if (start_time.isZero())
  {
    return 0.0;
  }

  return std::max(0.0, (now - start_time).toSec());
}

double GnssQualityMonitor::degToRad(const double degrees) const
{
  return degrees * kDegToRad;
}

}  // namespace RobotLocalization
