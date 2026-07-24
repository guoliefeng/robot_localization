/*
 * Copyright (c) 2026
 * All rights reserved.
 */

/*
 * 模块功能：
 *
 * 本节点是在 robot_localization::RosEkf 外部增加的一层定位数据管理封装。
 * 节点本身不重新实现 EKF 的预测方程和观测更新方程，而是负责：
 *
 * 1. 接收 INS、LIO 和轮速输出的 nav_msgs::Odometry；
 * 2. 检查输入时间戳、位置、姿态和速度字段是否合法；
 * 3. 根据 localization status 和 localization policy 决定当前参与融合的数据源；
 * 4. 根据 PRIMARY、DEGRADED、WEAK 等模式设置观测协方差；
 * 5. 将选中的 pose/twist 观测送入 robot_localization EKF；
 * 6. 定时读取 EKF 状态并输出 final_odom 和 LocalizationEstimate。
 *
 * 当前节点属于单 EKF 全局定位融合结构：
 *
 *   INS/LIO 绝对位姿 + wheel vx
 *                 ↓
 *   状态与策略选源
 *          ↓
 *   设置观测协方差
 *          ↓
 *   robot_localization EKF
 *          ↓
 *   final_odom
 *
 * 注意：
 * - 当前 INS 只保证 pose 有效，INS twist 长期为零，因此默认禁止融合；
 * - /wheel_odom 只融合 child frame 中的前向 vx，不融合其 pose、vy 或 yaw rate；
 * - PRIMARY LIO 下 wheel 默认时间对齐并以 25% 权重修正 LIO vx；
 * - wheel 对短时异常做一致性门控，LIO 不可用时可作为独立速度后备；
 * - 可选 wheel 独占 vx；启用后轮速超时会自动恢复 LIO vx；
 * - frame 标签归一化不等同于真正的 TF 坐标变换；
 * - 本节点输出位于 map 世界坐标系，定位源切换时可能受到绝对位姿差异影响。
 */

#include "robot_localization/filter_common.h"
#include "robot_localization/ros_filter_types.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
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
#include <deque>
#include <iterator>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

namespace
{

// localization_switcher 输出的定位健康状态。
// 该状态用于描述当前主定位结果是否正常、降级、丢失或处于备用源状态。
// 它不直接等价于某一个具体传感器是否有消息。
enum class LocalizationStatus : unsigned int
{
  NORMAL = 0x00,      // 当前定位结果正常
  DR = 0x01,          // 当前处于航位推算或无绝对位姿校正状态
  LOST = 0x02,        // 当前没有可信定位结果
  NOT_STABLE = 0x04,  // 当前定位源存在异常或尚未稳定
  SECONDARY = 0x08    // 当前已切换到备用定位源
};

// localization_switcher 输出的定位策略位掩码。
// 一个 policy 数值可以同时包含多个标志，因此判断策略时必须使用位运算，
// 不能将 policy 简单理解为互斥枚举值。
enum class LocalizationPolicy : unsigned long long
{
  NO_LOCALIZATION = 0,  // 当前策略不提供可用定位源
  USE_LIDAR_LIO = 1,    // 策略包含激光 LIO 标志
  USE_FUSION_ODOM = 2,  // 策略包含融合里程计标志
  USE_GNSS = 4,         // 策略包含 GNSS/INS 标志
  USE_ODOM_VEL_IMU = 8, // 策略包含里程计速度和 IMU 标志
  USE_ODOM_VEL = 16,    // 策略包含里程计速度标志
  USE_RELOC_GNSS = 32,  // 策略包含 GNSS 重定位标志
  USE_RELOC_X = 64      // 策略包含 X 方向重定位标志
};

// 某一路观测送入 EKF 时采用的可信度模式。
// 不同模式通过设置不同的观测协方差影响 Kalman 增益。
// DROP 表示该数据源当前不进入 EKF measurement queue。
enum class SourceMode
{
  PRIMARY,   // 当前主观测源，使用较小观测方差
  WEAK,      // 弱约束观测源，只对状态提供较轻微修正
  DEGRADED,  // 质量下降但仍允许参与融合
  DROP       // 当前完全不使用该观测源
};

double deg2rad(const double deg)
{
  return deg * M_PI / 180.0;
}

// 创建长度为 STATE_SIZE 的全零 update vector。
// 某一状态对应元素为 0 时，该 measurement 不会直接观测该状态变量。
std::vector<int> makeEmptyUpdateVector()
{
  return std::vector<int>(RobotLocalization::STATE_SIZE, 0);
}

// 当前绝对位姿观测只融合二维平面中的 x、y 和 yaw。
// z、roll、pitch 以及所有速度和加速度状态不由该 pose measurement 直接观测。
// two_d_mode 会在 robot_localization 内部进一步约束三维状态。
std::vector<int> makePoseUpdateVector()
{
  std::vector<int> update_vector = makeEmptyUpdateVector();
  update_vector[RobotLocalization::StateMemberX] = 1;
  update_vector[RobotLocalization::StateMemberY] = 1;
  update_vector[RobotLocalization::StateMemberYaw] = 1;
  return update_vector;
}

// 当前 twist 观测配置为 vx、vy 和 yaw rate。
// nav_msgs::Odometry 中的 twist 按 ROS 语义应表达在 child_frame_id 坐标系。
// 如果数据实际不位于 base_link，必须先完成正确的坐标变换，不能只修改 frame 字符串。
std::vector<int> makeTwistUpdateVector()
{
  std::vector<int> update_vector = makeEmptyUpdateVector();
  update_vector[RobotLocalization::StateMemberVx] = 1;
  update_vector[RobotLocalization::StateMemberVy] = 1;
  update_vector[RobotLocalization::StateMemberVyaw] = 1;
  return update_vector;
}

// Legacy 独占模式可按运行状态选择是否保留 LIO vx。默认同步 blend 模式
// 不使用该分支，而是保留 LIO vx 并在送入 EKF 前修改为 time-aligned blend。
std::vector<int> makeFusionTwistUpdateVector(const bool use_vx)
{
  std::vector<int> update_vector = makeTwistUpdateVector();
  update_vector[RobotLocalization::StateMemberVx] = use_vx ? 1 : 0;
  return update_vector;
}

// 轮速只直接观测车辆前向速度 vx。
std::vector<int> makeWheelVxUpdateVector()
{
  std::vector<int> update_vector = makeEmptyUpdateVector();
  update_vector[RobotLocalization::StateMemberVx] = 1;
  return update_vector;
}

// 统计 update vector 中启用的状态数量。
// CallbackData 使用该数量判断 pose 或 twist measurement 是否包含有效观测分量。
int updateSum(const std::vector<int> &update_vector)
{
  return std::accumulate(update_vector.begin(), update_vector.end(), 0);
}

// 构造 robot_localization 的 measurement callback 配置。
// 当前 pose/twist 均按绝对、非 relative、非 differential 方式进入 EKF。
// rejection_threshold 为 Mahalanobis 距离门限，不是普通的米或弧度误差门限。
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

// 将 EKF 输出的 variance 转换为标准差，供 LocalizationEstimate 不确定度字段使用。
// max(0, variance) 用于避免数值误差产生轻微负方差后执行 sqrt 导致 NaN。
double standardDeviation(const double variance)
{
  return std::sqrt(std::max(0.0, variance));
}

// 检查绝对位姿是否具备进入 EKF 或用于初始化的基本条件：
// - 时间戳非零；
// - position 各分量为有限数；
// - quaternion 各分量为有限数；
// - quaternion 模长不能接近零。
//
// 此处只做格式和数值合法性检查，不判断定位结果的实际精度、跳变或漂移。
bool isValidAbsolutePose(const nav_msgs::Odometry &odom)
{
  const geometry_msgs::Point &position = odom.pose.pose.position;
  const geometry_msgs::Quaternion &orientation = odom.pose.pose.orientation;
  // 不要求输入四元数在这里严格归一化，但必须排除全零或接近全零四元数。
  // robot_localization 在后续处理中会对轻微未归一化的四元数进行归一化。
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

// 检查整条 Odometry 的 pose 和 twist 字段是否均为有限数。
// 当前实现仍按整条消息进行校验：即使某个 twist 分量未参与融合，
// 该分量若为 NaN，也会导致整条消息被拒绝。
// 当前上游 INS 使用数值 0 填充 twist，因此不会触发该问题。
bool isFiniteOdometry(const nav_msgs::Odometry &odom)
{
  const geometry_msgs::Vector3 &linear = odom.twist.twist.linear;
  const geometry_msgs::Vector3 &angular = odom.twist.twist.angular;
  return isValidAbsolutePose(odom) &&
         std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z) &&
         std::isfinite(angular.x) && std::isfinite(angular.y) && std::isfinite(angular.z);
}

// 清除上游消息携带的完整 pose/twist covariance。
// 后续将根据 SourceMode 写入本节点定义的对角观测方差。
//
// 影响：
// - 上游动态质量变化不会直接保留；
// - 上游提供的非对角相关项会被清除；
// - 最终 Kalman 增益主要由本节点配置的固定方差决定。
void clearCovariances(nav_msgs::Odometry &odom)
{
  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
}

// 设置 pose measurement 的对角观测方差。
// 参数均为 variance，而不是 standard deviation：
// - xy_var 单位为 m²；
// - yaw_var 单位为 rad²；
// - z、roll、pitch 当前不作为主要二维观测，使用较大方差占位。
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

// 设置 twist measurement 的对角观测方差。
// 参数均为 variance，而不是 standard deviation：
// - vxy_var 单位为 (m/s)²；
// - vyaw_var 单位为 (rad/s)²。
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

// 构造只包含 vx 的标量轮速观测 covariance。
// bag 中 /wheel_odom 的完整 6x6 covariance 存在非对称项，不能整体传入 EKF；
// 这里只写入经过调参的 vx variance，其他分量不会被 update vector 使用。
void setWheelVxCov(
  geometry_msgs::TwistWithCovarianceStamped &twist,
  const double vx_var)
{
  std::fill(twist.twist.covariance.begin(), twist.twist.covariance.end(), 0.0);
  twist.twist.covariance[0] = vx_var;
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
  // 默认关闭 INS twist 融合。
  // 当前 /localization/ins 的 pose 有效，但 twist 长期为零，
  // 该零值不是可靠的车辆静止观测。
  LocEkfNode()
    : nh_(),
      nh_priv_("~"),
      ekf_(nh_, nh_priv_, "loc_ekf_node"),
      state_timeout_sec_(1.0),
      // INS 默认提供 x/y/yaw 绝对位姿观测。
      ins_pose_cb_data_(makeCallbackData("ins_pose", makePoseUpdateVector(), 5.0)),
      // INS twist 默认使用全零 update vector，不向 EKF 添加速度观测。
      ins_twist_cb_data_(makeCallbackData("ins_twist", makeEmptyUpdateVector(), 3.0)),
      // LIO 当前提供 x/y/yaw 绝对位姿观测。
      fusion_pose_cb_data_(makeCallbackData("fusion_pose", makePoseUpdateVector(), 3.0)),
      // LIO 当前提供 vx/vy/yaw rate 速度观测。
      fusion_twist_cb_data_(makeCallbackData("fusion_twist", makeTwistUpdateVector(), 3.0)),
      // wheel 健康时使用该配置，仅保留 LIO 的 vy/yaw rate。
      fusion_twist_without_vx_cb_data_(
        makeCallbackData("fusion_twist", makeFusionTwistUpdateVector(false), 3.0)),
      // /wheel_odom 只观测车辆坐标系前向 vx。
      wheel_vx_cb_data_(makeCallbackData("wheel_vx", makeWheelVxUpdateVector(), 5.0))
  {
    loadParams();
    configureCallbackData();

    ROS_INFO_STREAM("INS pose fusion: enabled");
    ROS_INFO_STREAM(
      "INS twist fusion: " << (fuse_ins_twist_ ? "enabled" : "disabled"));
    ROS_INFO_STREAM(
      "Wheel vx fusion: enabled=" << std::boolalpha << wheel_vx_enabled_
                                   << ", replace_lio_vx=" << wheel_vx_replace_lio_vx_
                                   << ", synchronized_blend=" << effectiveWheelBlend());

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
    wheel_sub_ = nh_.subscribe(
      wheel_topic_, input_queue_size_, &LocEkfNode::wheelCb, this, low_latency_hints);
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

  struct WheelSample
  {
    double stamp = 0.0;
    double vx = 0.0;
  };

  // 加载节点私有参数。
  // 由于 run_wheel_vx.launch 会先加载 YAML，再可能通过 <param> 覆盖同名参数，
  // 分析现场实际配置时应以 rosparam 中最终值为准，不能只查看 YAML 文件。
  void loadParams()
  {
    nh_priv_.param("ins_odom_topic", ins_topic_, std::string("/localization/ins"));
    nh_priv_.param("fusion_odom_topic", fusion_topic_, std::string("/lio_loc_result"));
    nh_priv_.param("wheel_odom_topic", wheel_topic_, std::string("/wheel_odom"));
    nh_priv_.param("localization_status_topic", localization_status_topic_, std::string("/localization/status"));
    nh_priv_.param("loc_policy_topic", loc_policy_topic_, std::string("/localization/loc_policy_code"));
    nh_priv_.param("frame_id", frame_id_, std::string("map"));
    nh_priv_.param("child_frame_id", child_frame_id_, std::string("base_link"));
    nh_priv_.param("state_timeout_sec", state_timeout_sec_, 1.0);
    nh_priv_.param("ins_pose_rejection_threshold", ins_pose_rejection_threshold_, 5.0);
    nh_priv_.param("ins_twist_rejection_threshold", ins_twist_rejection_threshold_, 3.0);
    // 仅当上游 INS 确实提供经过验证、坐标系正确且时间同步的速度时才能开启。
    // 不能仅因为字段存在或数值为零就认为该 twist 是有效观测。
    nh_priv_.param("fuse_ins_twist", fuse_ins_twist_, false);
    nh_priv_.param("fusion_pose_rejection_threshold", fusion_pose_rejection_threshold_, 3.0);
    nh_priv_.param("fusion_twist_rejection_threshold", fusion_twist_rejection_threshold_, 3.0);
    nh_priv_.param("wheel_vx_enabled", wheel_vx_enabled_, true);
    nh_priv_.param("wheel_vx_rejection_threshold", wheel_vx_rejection_threshold_, 5.0);
    nh_priv_.param("wheel_vx_replace_lio_vx", wheel_vx_replace_lio_vx_, false);
    nh_priv_.param("wheel_vx_timeout_sec", wheel_vx_timeout_sec_, 0.15);
    nh_priv_.param("wheel_vx_max_abs", wheel_vx_max_abs_, 3.5);
    nh_priv_.param("wheel_lio_vx_blend", wheel_lio_vx_blend_, 0.25);
    nh_priv_.param("wheel_lio_sync_tolerance_sec", wheel_lio_sync_tolerance_sec_, 0.03);
    nh_priv_.param("wheel_lio_vx_gate", wheel_lio_vx_gate_, 0.03);
    nh_priv_.param("wheel_lio_vx_gate_max_dt", wheel_lio_vx_gate_max_dt_, 0.10);
    nh_priv_.param("wheel_primary_vx_variance", wheel_primary_vx_variance_, 0.05);
    nh_priv_.param(
      "localization_estimate_topic", localization_estimate_topic_,
      std::string("/localization_estimate"));
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
    wheel_vx_timeout_sec_ = std::max(0.01, wheel_vx_timeout_sec_);
    wheel_vx_max_abs_ = std::max(0.1, wheel_vx_max_abs_);
    wheel_lio_vx_blend_ = std::max(0.0, std::min(1.0, wheel_lio_vx_blend_));
    wheel_lio_sync_tolerance_sec_ = std::max(0.001, wheel_lio_sync_tolerance_sec_);
    wheel_lio_vx_gate_ = std::max(0.0, wheel_lio_vx_gate_);
    wheel_lio_vx_gate_max_dt_ = std::max(0.01, wheel_lio_vx_gate_max_dt_);
    wheel_primary_vx_variance_ = std::max(1e-6, wheel_primary_vx_variance_);
    // 防止除零或创建非法周期定时器。
    // 参数异常时回退到 100 Hz 默认输出频率。
    if (localization_estimate_frequency_ <= 0.0)
    {
      ROS_WARN("localization_estimate_frequency must be positive; using 100 Hz");
      localization_estimate_frequency_ = 100.0;
    }
  }

  // 根据当前参数生成各数据源的 pose/twist update vector。
  // update vector 决定某一 measurement 能够直接校正 EKF 中的哪些状态。
  void configureCallbackData()
  {
    const std::vector<int> pose_update_vector = makePoseUpdateVector();
    const std::vector<int> fusion_twist_update_vector = makeTwistUpdateVector();
    const std::vector<int> fusion_twist_without_vx_update_vector =
      makeFusionTwistUpdateVector(false);
    // 关闭 INS twist 时必须使用全零 update vector，
    // 而不是给零速度配置一个很大的 covariance。
    // 全零 update vector 表示该 measurement 不存在，
    // 大 covariance 仍然表示存在一个低权重的零速度观测。
    const std::vector<int> ins_twist_update_vector =
      fuse_ins_twist_ ? makeTwistUpdateVector() : makeEmptyUpdateVector();

    ins_pose_cb_data_ =
      makeCallbackData("ins_pose", pose_update_vector, ins_pose_rejection_threshold_);
    ins_twist_cb_data_ =
      makeCallbackData("ins_twist", ins_twist_update_vector, ins_twist_rejection_threshold_);
    fusion_pose_cb_data_ =
      makeCallbackData("fusion_pose", pose_update_vector, fusion_pose_rejection_threshold_);
    fusion_twist_cb_data_ =
      makeCallbackData("fusion_twist", fusion_twist_update_vector, fusion_twist_rejection_threshold_);
    fusion_twist_without_vx_cb_data_ =
      makeCallbackData(
        "fusion_twist", fusion_twist_without_vx_update_vector,
        fusion_twist_rejection_threshold_);
    wheel_vx_cb_data_ =
      makeCallbackData("wheel_vx", makeWheelVxUpdateVector(), wheel_vx_rejection_threshold_);

    // 当前默认观测关系：
    //   INS  → x、y、yaw
    //   LIO  → x、y、yaw、vy、yaw rate；wheel 不健康时回退融合 vx
    //   wheel → vx
    //
    // INS 速度由连续 pose 创新及 position-velocity cross covariance 间接估计，
    // 不再被上游无效零 twist 持续拉回零。
  }

  void normalizeOdomFrame(nav_msgs::Odometry &odom) const
  {
    // This only normalizes frame labels; it does not transform twist values.
    // INS twist is disabled by default, isolating it from this known frame risk.
    // 注意：该函数只统一消息中的 frame 标签，不执行真正的坐标变换。
    //
    // 修改 header.frame_id 并不会自动变换 pose 数值；
    // 修改 child_frame_id 也不会自动旋转 twist、处理杆臂速度或转换 covariance。
    //
    // 只有在确认：
    //   1. 输入 pose 数值已经表达在 frame_id_ 对应坐标系；
    //   2. 启用的 twist 数值已经表达在 child_frame_id_ 对应坐标系；
    // 才能安全地只修改标签。
    //
    // 当前 INS twist 默认关闭，因此 INS 的零速度不会因重写 child_frame_id
    // 而被当成 base_link 速度送入 EKF。
    // LIO twist 仍启用，必须保证上游 LIO 输出的速度语义符合 base_link。
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
  }

  // 对输入消息执行基础保护，避免非法或时间异常数据进入 EKF。
  //
  // 检查顺序：
  // 1. pose/twist 是否包含 NaN 或 Inf；
  // 2. 消息时间是否比当前时间落后过多；
  // 3. 消息时间是否明显位于未来。
  //
  // age 使用 ros::Time::now() 与消息 header.stamp 计算，
  // 因此要求 header.stamp 表示真实测量时间，而不是回调接收时间。
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
    // 消息延迟超过 max_input_age_sec_ 时直接丢弃，
    // 防止严重滞后的绝对位姿对当前状态产生大幅回拉。
    if (age_sec > max_input_age_sec_)
    {
      ++stale_count;
      ROS_WARN_STREAM_THROTTLE(
        1.0, "Dropping stale " << source_name << ": age=" << age_sec << " s");
      return false;
    }
    // 允许少量时钟和调度误差，但拒绝明显超前于本机 ROS 时间的测量。
    // 大量 future-dated 消息通常意味着设备时钟或时间同步配置异常。
    if (age_sec < -max_future_stamp_sec_)
    {
      ++invalid_count;
      ROS_WARN_STREAM_THROTTLE(
        1.0, "Dropping future-dated " << source_name << ": age=" << age_sec << " s");
      return false;
    }
    return true;
  }

  // /wheel_odom 这里只消费 header、child_frame_id 和 twist.linear.x。
  // 不调用 isFiniteOdometry()，因为 wheel pose 和未使用的 covariance 不应决定
  // 一个有效的标量 vx 是否可用。
  bool validateWheelInput(const nav_msgs::Odometry &odom)
  {
    const double vx = odom.twist.twist.linear.x;
    if (odom.header.stamp.isZero() || !std::isfinite(vx))
    {
      ++wheel_invalid_count_;
      ROS_WARN_STREAM_THROTTLE(1.0, "Dropping wheel vx with invalid stamp/value");
      return false;
    }

    if (!odom.child_frame_id.empty() && odom.child_frame_id != child_frame_id_)
    {
      ++wheel_invalid_count_;
      ROS_WARN_STREAM_THROTTLE(
        1.0, "Dropping wheel vx in unexpected child frame '"
          << odom.child_frame_id << "' (expected '" << child_frame_id_ << "')");
      return false;
    }

    const double age_sec = (ros::Time::now() - odom.header.stamp).toSec();
    last_wheel_age_sec_.store(age_sec, std::memory_order_relaxed);
    if (age_sec > max_input_age_sec_)
    {
      ++wheel_stale_count_;
      ROS_WARN_STREAM_THROTTLE(1.0, "Dropping stale wheel vx: age=" << age_sec << " s");
      return false;
    }
    if (age_sec < -max_future_stamp_sec_)
    {
      ++wheel_invalid_count_;
      ROS_WARN_STREAM_THROTTLE(
        1.0, "Dropping future-dated wheel vx: age=" << age_sec << " s");
      return false;
    }
    if (std::fabs(vx) > wheel_vx_max_abs_)
    {
      ++wheel_outlier_count_;
      ROS_WARN_STREAM_THROTTLE(
        1.0, "Dropping unreasonable wheel vx=" << vx
                                                << " m/s, limit=" << wheel_vx_max_abs_);
      return false;
    }
    return true;
  }

  // 使用第一条通过检查的绝对 pose 初始化 EKF。
  // 初始化只设置绝对位姿，不使用同一条消息中的 twist。
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
    // 通过 robot_localization 标准 set_pose 接口初始化状态和协方差，
    // 避免在自定义节点中直接操作 EKF 内部状态。
    ekf_.setPoseCallback(initial_pose);
    absolute_pose_initialized_ = true;

    ROS_INFO_STREAM(
      "Initialized EKF from " << source_name
                               << " absolute pose at stamp=" << odom.header.stamp
                               << " x=" << odom.pose.pose.position.x
                               << " y=" << odom.pose.pose.position.y);
    return true;
  }

  // 获取当前定位状态和策略快照，并检查其新鲜度。
  // 状态或策略任一超时，都按 LOST + NO_LOCALIZATION 处理，
  // 防止控制话题停止更新后继续沿用历史选源结果。
  LocDecision currentDecision() const
  {
    LocDecision decision = loc_decision_;
    const ros::Time now = ros::Time::now();
    // last_status_stamp 和 last_policy_stamp 记录的是本节点收到控制消息的 ROS 时间，
    // 不是消息内部测量时间，因为 UInt8/UInt64 消息没有 header。
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

  // 根据定位状态和策略决定 INS pose 的观测权重。
  //
  // 主要逻辑：
  // - LOST/DR：不使用 INS 绝对 pose；
  // - NORMAL + USE_FUSION_ODOM：LIO 为主，INS 不参与；
  // - USE_GNSS 且 NORMAL/SECONDARY：INS 为主；
  // - USE_GNSS 且 NOT_STABLE：INS 降级参与；
  // - USE_FUSION_ODOM 的其他允许状态：INS 作为弱约束。
  //
  // 当前函数返回的是整条 INS 数据源的模式，但由于 INS twist 默认关闭，
  // 实际受该模式影响的主要是 INS x/y/yaw pose covariance。
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

  // 根据定位状态和策略决定 LIO pose/twist 的观测权重。
  //
  // SECONDARY + USE_GNSS 时明确 DROP LIO，避免已被 switcher 判定异常的
  // LIO 仍作为弱观测持续拉动 final_odom。
  // NORMAL + USE_FUSION_ODOM 时 LIO 为主；
  // NOT_STABLE 时使用 DEGRADED；
  // GNSS 策略下通常只作为 WEAK 后备约束。
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

  // wheel vx 与 localization status/policy 完全解耦。只要人工开关启用，
  // wheel 始终按 PRIMARY 使用；消息质量由 validateWheelInput()、时间同步、
  // wheel/LIO 一致性门控和 Mahalanobis 门限独立判断。
  SourceMode selectWheelMode() const
  {
    return wheel_vx_enabled_ ? SourceMode::PRIMARY : SourceMode::DROP;
  }

  // 按当前 SourceMode 重写 INS 观测协方差。
  // covariance 越小，EKF 越信任该观测，校正幅度通常越大；
  // covariance 越大，观测对预测状态的拉动越弱。
  //
  // 当前 INS twist 默认关闭，因此 setTwistCov 写入的数值通常不会参与校正，
  // 但予以保留，以支持未来人工开启 fuse_ins_twist。
  void applyInsCovariance(nav_msgs::Odometry &odom, const SourceMode mode) const
  {
    clearCovariances(odom);

    switch (mode)
    {
      case SourceMode::PRIMARY:
        // PRIMARY：可信 INS 绝对位姿，允许较强校正。
        setPoseCov(odom, 0.02, 100.0, 100.0, std::pow(deg2rad(0.5), 2.0));
        setTwistCov(odom, 0.05, 100.0, 100.0, std::pow(deg2rad(0.5), 2.0));
        break;
      case SourceMode::WEAK:
        // WEAK：仅作为弱约束，避免与主定位源明显竞争。
        setPoseCov(odom, 10.0, 100.0, 100.0, std::pow(deg2rad(10.0), 2.0));
        setTwistCov(odom, 0.50, 100.0, 100.0, std::pow(deg2rad(5.0), 2.0));
        break;
      case SourceMode::DEGRADED:
        // DEGRADED：INS 质量下降，方差介于 PRIMARY 与 WEAK 之间。
        setPoseCov(odom, 1.0, 100.0, 100.0, std::pow(deg2rad(3.0), 2.0));
        setTwistCov(odom, 0.20, 100.0, 100.0, std::pow(deg2rad(2.0), 2.0));
        break;
      case SourceMode::DROP:
        // DROP：调用方会在进入此函数前直接丢弃消息。
        break;
    }
  }

  // 按当前 SourceMode 重写 LIO pose 和 twist 的观测协方差。
  // PRIMARY 模式的关键方差由 YAML 参数提供，便于现场调试；
  // WEAK 和 DEGRADED 当前使用代码中的固定值。
  //
  // 当前会清除上游 LIO 原始 covariance，
  // 因此上游算法实时输出的质量变化不会直接保留到 EKF。
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

  void applyWheelCovariance(geometry_msgs::TwistWithCovarianceStamped &twist) const
  {
    setWheelVxCov(twist, wheel_primary_vx_variance_);
  }

  bool isAbsolutePoseInitialized() const
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    return absolute_pose_initialized_;
  }

  bool wheelVxIsRecentAt(const ros::Time &stamp) const
  {
    const double wheel_stamp =
      last_wheel_accepted_stamp_sec_.load(std::memory_order_relaxed);
    return wheel_stamp > 0.0 &&
           std::fabs(stamp.toSec() - wheel_stamp) <= wheel_vx_timeout_sec_;
  }

  double effectiveWheelBlend() const
  {
    return wheel_vx_replace_lio_vx_ ? 1.0 : wheel_lio_vx_blend_;
  }

  // Keep a short, timestamp-sorted wheel history. /wheel_odom is normally
  // received about 20 ms before the LIO message for the same measurement time,
  // so the history usually brackets the incoming LIO stamp.
  void cacheWheelSample(const double stamp, const double vx)
  {
    std::lock_guard<std::mutex> lock(wheel_history_mtx_);
    const WheelSample sample{stamp, vx};
    if (wheel_history_.empty() || stamp >= wheel_history_.back().stamp)
    {
      wheel_history_.push_back(sample);
    }
    else
    {
      const auto position = std::lower_bound(
        wheel_history_.begin(), wheel_history_.end(), stamp,
        [](const WheelSample &entry, const double value) { return entry.stamp < value; });
      wheel_history_.insert(position, sample);
    }

    const double oldest_allowed = stamp - std::max(1.0, 4.0 * wheel_vx_timeout_sec_);
    while (!wheel_history_.empty() && wheel_history_.front().stamp < oldest_allowed)
    {
      wheel_history_.pop_front();
    }
  }

  // Interpolate wheel vx at the LIO measurement time. Falling back to a nearby
  // endpoint is allowed only inside the configured synchronization tolerance.
  bool wheelVxAt(const double stamp, double &vx) const
  {
    std::lock_guard<std::mutex> lock(wheel_history_mtx_);
    if (wheel_history_.empty())
    {
      return false;
    }

    const auto after = std::lower_bound(
      wheel_history_.begin(), wheel_history_.end(), stamp,
      [](const WheelSample &entry, const double value) { return entry.stamp < value; });
    if (after == wheel_history_.begin())
    {
      if (std::fabs(after->stamp - stamp) > wheel_lio_sync_tolerance_sec_)
      {
        return false;
      }
      vx = after->vx;
      return true;
    }
    if (after == wheel_history_.end())
    {
      const WheelSample &before = wheel_history_.back();
      if (std::fabs(stamp - before.stamp) > wheel_lio_sync_tolerance_sec_)
      {
        return false;
      }
      vx = before.vx;
      return true;
    }

    const WheelSample &before = *std::prev(after);
    const double before_dt = stamp - before.stamp;
    const double after_dt = after->stamp - stamp;
    if (before_dt > wheel_lio_sync_tolerance_sec_ ||
        after_dt > wheel_lio_sync_tolerance_sec_ || after->stamp <= before.stamp)
    {
      return false;
    }
    const double ratio = before_dt / (after->stamp - before.stamp);
    vx = before.vx + ratio * (after->vx - before.vx);
    return std::isfinite(vx);
  }

  // 启动阶段尚未获得绝对 pose 时，INS 或 LIO 中先到达的有效消息都可能触发初始化。
  // 初始化暂时不依赖 status/policy，避免状态和策略话题尚未到达时 EKF 无法启动。
  //
  // 当前行为意味着初始化数据源由消息到达顺序决定，
  // 并不保证一定由当前策略选择的 PRIMARY 数据源完成初始化。
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
    // 初始化使用该数据源的 PRIMARY pose covariance，
    // 避免用 WEAK 或 DEGRADED 方差初始化出过大的初始不确定度。
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

  // INS 数据处理流程：
  // 1. 检查消息合法性和时间新鲜度；
  // 2. 复制消息并统一 frame 标签；
  // 3. 必要时使用 INS pose 初始化 EKF；
  // 4. 根据当前 status/policy 选择 INS 模式；
  // 5. DROP 时不送入 EKF；
  // 6. 重写 covariance；
  // 7. 通过 RosEkf::odometryCallback 送入 measurement queue。
  //
  // fuse_ins_twist=false 时，odometryCallback 虽然仍接收 twist CallbackData，
  // 但其 update vector 全部为零，不会形成有效的 INS 速度观测。
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
    const SourceMode wheel_mode = selectWheelMode();
    ++ins_count_;

    if (ins_mode == SourceMode::DROP)
    {
      ++ins_drop_count_;
      logStatus(decision, ins_mode, fusion_mode, wheel_mode);
      return;
    }

    applyInsCovariance(*out, ins_mode);

    // 这里设置的 twist covariance 不代表 twist 一定会被融合。
    // 是否融合最终由 ins_twist_cb_data_ 中的 update vector 决定。

    ekf_.odometryCallback(out, "ins_odom", ins_pose_cb_data_, ins_twist_cb_data_);
    logStatus(decision, ins_mode, fusion_mode, wheel_mode);
  }

  // LIO 数据处理流程与 INS 基本一致。
  // 当前 LIO pose 和 twist 均启用，因此必须确保二者时间戳、坐标系和单位一致。
  // 如果 pose 和 twist 来源于同一个 LIO 内部估计器，二者可能存在信息相关性。
  void fusionCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (!msg || !validateInput(
        *msg, "LIO", fusion_invalid_count_, fusion_stale_count_, last_fusion_age_sec_))
    {
      return;
    }
    last_fusion_accepted_stamp_sec_.store(msg->header.stamp.toSec(), std::memory_order_relaxed);
    last_lio_raw_stamp_sec_.store(msg->header.stamp.toSec(), std::memory_order_relaxed);
    last_lio_raw_vx_.store(msg->twist.twist.linear.x, std::memory_order_relaxed);
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
    const SourceMode wheel_mode = selectWheelMode();
    ++fusion_count_;

    if (fusion_mode == SourceMode::DROP)
    {
      ++fusion_drop_count_;
      logStatus(decision, ins_mode, fusion_mode, wheel_mode);
      return;
    }

    applyFusionCovariance(*out, fusion_mode);

    // LIO and wheel vx are strongly correlated. In PRIMARY mode, align wheel
    // to the LIO stamp and form one blended vx observation instead of feeding
    // two asynchronous corrections into the EKF. Keeping the LIO covariance
    // avoids claiming false information gain from correlated sensors.
    const double wheel_blend = effectiveWheelBlend();
    if (wheel_mode == SourceMode::PRIMARY &&
        fusion_mode == SourceMode::PRIMARY && wheel_blend > 0.0)
    {
      double aligned_wheel_vx = 0.0;
      if (wheelVxAt(out->header.stamp.toSec(), aligned_wheel_vx))
      {
        const double lio_vx = out->twist.twist.linear.x;
        const double innovation = std::fabs(aligned_wheel_vx - lio_vx);
        if (wheel_lio_vx_gate_ <= 0.0 || innovation <= wheel_lio_vx_gate_)
        {
          out->twist.twist.linear.x =
            (1.0 - wheel_blend) * lio_vx + wheel_blend * aligned_wheel_vx;
          ++wheel_blended_count_;
        }
        else
        {
          ++wheel_outlier_count_;
          ROS_WARN_STREAM_THROTTLE(
            1.0, "Not blending wheel vx inconsistent with time-aligned primary LIO: wheel="
              << aligned_wheel_vx << " lio=" << lio_vx
              << " innovation=" << innovation << " m/s");
        }
      }

    }

    // 当前一次 LIO Odometry 会被 robot_localization 拆分成 pose 和 twist 两条 measurement，
    // 两者使用同一个 header.stamp，并分别执行观测校正。
    const bool omit_lio_vx = wheel_blend <= 0.0 &&
      wheel_vx_replace_lio_vx_ && wheelVxIsRecentAt(out->header.stamp);
    const RobotLocalization::CallbackData &fusion_twist_data =
      omit_lio_vx ? fusion_twist_without_vx_cb_data_ : fusion_twist_cb_data_;
    ekf_.odometryCallback(out, "fusion_odom", fusion_pose_cb_data_, fusion_twist_data);
    logStatus(decision, ins_mode, fusion_mode, wheel_mode);
  }

  // 轮速回调仅消费 base_link vx 标量：
  // - 不使用 wheel pose（它位于 odom，而 EKF pose 位于 map）；
  // - 不复制上游非对称的完整 covariance；
  // - PRIMARY LIO 可用时缓存并由 fusionCb 时间对齐、门控、blend；
  // - 其他模式下才构造独立 twist measurement 作为速度后备。
  void wheelCb(const nav_msgs::Odometry::ConstPtr &msg)
  {
    if (!msg || !validateWheelInput(*msg))
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
    const SourceMode wheel_mode = selectWheelMode();
    ++wheel_count_;

    if (wheel_mode == SourceMode::DROP)
    {
      ++wheel_drop_count_;
      logStatus(decision, ins_mode, fusion_mode, wheel_mode);
      return;
    }
    if (!isAbsolutePoseInitialized())
    {
      ++wheel_drop_count_;
      ++wheel_preinit_drop_count_;
      ROS_WARN_STREAM_THROTTLE(1.0, "Dropping wheel vx before absolute pose initialization");
      logStatus(decision, ins_mode, fusion_mode, wheel_mode);
      return;
    }

    cacheWheelSample(msg->header.stamp.toSec(), msg->twist.twist.linear.x);
    last_wheel_accepted_stamp_sec_.store(
      msg->header.stamp.toSec(), std::memory_order_relaxed);
    ++wheel_accepted_count_;

    // PRIMARY LIO consumes this cached sample at the matching LIO timestamp.
    // Do not also enqueue an independent wheel measurement: that would
    // double-count correlated velocity and can advance the filter ahead of a
    // delayed LIO pose/twist update.
    if (fusion_mode == SourceMode::PRIMARY && effectiveWheelBlend() > 0.0)
    {
      logStatus(decision, ins_mode, fusion_mode, wheel_mode);
      return;
    }

    const double lio_stamp = last_lio_raw_stamp_sec_.load(std::memory_order_relaxed);
    const double lio_vx = last_lio_raw_vx_.load(std::memory_order_relaxed);
    const double lio_dt = std::fabs(msg->header.stamp.toSec() - lio_stamp);
    const double wheel_lio_innovation = std::fabs(msg->twist.twist.linear.x - lio_vx);
    if (fusion_mode == SourceMode::PRIMARY && wheel_lio_vx_gate_ > 0.0 &&
        lio_stamp > 0.0 && lio_dt <= wheel_lio_vx_gate_max_dt_ &&
        wheel_lio_innovation > wheel_lio_vx_gate_)
    {
      ++wheel_drop_count_;
      ++wheel_outlier_count_;
      ROS_WARN_STREAM_THROTTLE(
        1.0, "Dropping wheel vx inconsistent with primary LIO: wheel="
          << msg->twist.twist.linear.x << " lio=" << lio_vx
          << " innovation=" << wheel_lio_innovation << " m/s");
      logStatus(decision, ins_mode, fusion_mode, wheel_mode);
      return;
    }

    geometry_msgs::TwistWithCovarianceStampedPtr wheel_twist(
      new geometry_msgs::TwistWithCovarianceStamped());
    wheel_twist->header.stamp = msg->header.stamp;
    wheel_twist->header.frame_id = child_frame_id_;
    wheel_twist->twist.twist.linear.x = msg->twist.twist.linear.x;
    applyWheelCovariance(*wheel_twist);
    ekf_.twistCallback(wheel_twist, wheel_vx_cb_data_, child_frame_id_);

    logStatus(decision, ins_mode, fusion_mode, wheel_mode);
  }

  // 更新定位健康状态。
  // 使用互斥锁保证状态回调、策略回调和传感器回调读取到一致的决策数据。
  void localizationStatusCb(const std_msgs::UInt8::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    loc_decision_.status = static_cast<LocalizationStatus>(msg->data);
    loc_decision_.last_status_stamp = ros::Time::now();
  }

  // 更新定位策略位掩码。
  // 实际选源逻辑在传感器消息到达时执行，而不是在策略消息回调中立即修改 EKF。
  void locPolicyCb(const std_msgs::UInt64::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    loc_decision_.policy = msg->data;
    loc_decision_.last_policy_stamp = ros::Time::now();
  }

  // 定时处理 EKF measurement queue，并发布当前滤波状态。
  //
  // processMeasurementsAndGetState() 会：
  // 1. 按测量时间戳顺序处理已经入队的 pose/twist；
  // 2. 在相邻测量时间之间执行预测；
  // 3. 对每条观测执行 Kalman 校正；
  // 4. 根据 predict_to_current_time 配置决定是否继续预测到 cycle_time。
  void publishLocalizationEstimate(const ros::TimerEvent &)
  {
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    const ros::Time cycle_time = ros::Time::now();
    // rosbag loop 或仿真时间重置可能导致 ROS 时间倒退。
    // 此时清空 wrapper 层的输出时间单调性记录，让底层 RosFilter 自行处理时间重置。
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
      // EKF 尚未初始化或当前无法生成有效状态时，本周期不发布。
      return;
    }

    // 禁止发布时间戳小于上一输出的 final_odom，
    // 防止下游控制、轨迹缓存或 TF 消费者接收到时间倒序数据。
    // 当前判断允许相同时间戳再次发布。
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
    // nav_msgs::Odometry 约定：
    // - pose 表达在 header.frame_id；
    // - twist 表达在 child_frame_id。
    //
    // EKF 输出 twist 当前按 base_link/车辆坐标系解释：
    //   vx_vrf：车辆前向速度
    //   vy_vrf：车辆横向速度
    //
    // LocalizationEstimate 同时需要地图坐标系速度，
    // 因此使用当前 yaw 将二维车体系速度旋转到 map：
    //   vx_map = cos(yaw) * vx_vrf - sin(yaw) * vy_vrf
    //   vy_map = sin(yaw) * vx_vrf + cos(yaw) * vy_vrf
    //
    // 这里假设 child_frame_id 与 base_link 对齐，
    // 不包含额外传感器安装角或杆臂补偿。
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
    estimate.pose.euler_angles.x = roll;
    estimate.pose.euler_angles.y = pitch;
    estimate.pose.euler_angles.z = yaw;

    estimate.pose.linear_velocity.x = vx_map;
    estimate.pose.linear_velocity.y = vy_map;
    estimate.pose.linear_velocity.z = vz_vrf;
    estimate.pose.linear_velocity_vrf.x = vx_vrf;
    estimate.pose.linear_velocity_vrf.y = vy_vrf;
    estimate.pose.linear_velocity_vrf.z = vz_vrf;

    // 当前角速度直接使用 EKF 输出值。
    // 对二维车辆主要关注 angular.z，即 yaw rate。
    estimate.pose.angular_velocity.x = odom.twist.twist.angular.x;
    estimate.pose.angular_velocity.y = odom.twist.twist.angular.y;
    estimate.pose.angular_velocity.z = odom.twist.twist.angular.z;
    estimate.pose.angular_velocity_vrf = estimate.pose.angular_velocity;

    // EKF 内部线加速度状态按车辆坐标系解释，
    // 与速度相同，使用 yaw 旋转得到 map 坐标系加速度。
    //
    // angular 字段在这里用于传递 EKF 估计的角运动相关量，
    // 不应与独立 IMU 原始角加速度混淆。
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

    // nav_msgs::Odometry covariance 对角线保存的是方差；
    // LocalizationEstimate uncertainty 字段需要标准差，
    // 因此统一执行 sqrt 转换。
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

  // 输出节点运行健康信息，包括：
  // - final_odom 相对当前时间的延迟；
  // - 最近接收的 INS/LIO/wheel 消息延迟；
  // - 单次输出回调耗时；
  // - 输出次数和超周期次数；
  // - 时间戳回退次数；
  // - 非法或超时输入计数。
  //
  // 该日志主要反映数据链路与计算调度状态，
  // 不直接表示定位精度是否满足业务要求。
  void logRuntimeStatus(const ros::Time &now, const nav_msgs::Odometry &odom)
  {
    const double ins_stamp = last_ins_accepted_stamp_sec_.load(std::memory_order_relaxed);
    const double fusion_stamp = last_fusion_accepted_stamp_sec_.load(std::memory_order_relaxed);
    const double wheel_stamp = last_wheel_accepted_stamp_sec_.load(std::memory_order_relaxed);
    const double newest_stamp = std::max(std::max(ins_stamp, fusion_stamp), wheel_stamp);
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
        << " wheel_age=" << last_wheel_age_sec_.load(std::memory_order_relaxed)
        << " loop_ms=" << last_execution_ms_.load(std::memory_order_relaxed)
        << " outputs=" << output_count_.load()
        << " overruns=" << loop_overrun_count_.load()
        << " stamp_regressions=" << output_regression_count_.load()
        << " ins_invalid/stale=" << ins_invalid_count_.load() << "/" << ins_stale_count_.load()
        << " lio_invalid/stale=" << fusion_invalid_count_.load() << "/" << fusion_stale_count_.load()
        << " wheel_invalid/stale/outlier=" << wheel_invalid_count_.load() << "/"
        << wheel_stale_count_.load() << "/" << wheel_outlier_count_.load());
  }

  // 节流输出当前状态、策略以及 INS/LIO/wheel 选源结果。
  // ins_in/fusion_in/wheel_in 表示通过基础校验后进入策略判断的消息数量，
  // drop 表示因当前定位策略而未送入 EKF 的消息数量。
  void logStatus(
    const LocDecision &decision,
    const SourceMode ins_mode,
    const SourceMode fusion_mode,
    const SourceMode wheel_mode) const
  {
    ROS_INFO_STREAM_THROTTLE(
      1.0,
      "YangpuLocEkf loc_state: status=" << statusToString(decision.status)
                                        << " policy=" << decision.policy
                                        << " ins=" << modeToString(ins_mode)
                                        << " fusion=" << modeToString(fusion_mode)
                                        << " wheel=" << modeToString(wheel_mode)
                                        << " ins_in=" << ins_count_
                                        << " ins_drop=" << ins_drop_count_
                                        << " fusion_in=" << fusion_count_
                                        << " fusion_drop=" << fusion_drop_count_
                                        << " wheel_in=" << wheel_count_
                                        << " wheel_accepted=" << wheel_accepted_count_
                                        << " wheel_blended=" << wheel_blended_count_
                                        << " wheel_drop=" << wheel_drop_count_
                                        << " wheel_outlier=" << wheel_outlier_count_);
  }

  // ROS 通信对象。
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // robot_localization EKF 实例。
  RobotLocalization::RosEkf ekf_;

  ros::Subscriber ins_sub_;
  ros::Subscriber fusion_sub_;
  ros::Subscriber wheel_sub_;
  ros::Subscriber localization_status_sub_;
  ros::Subscriber loc_policy_sub_;
  ros::Publisher localization_estimate_pub_;
  ros::Publisher filtered_odom_pub_;
  ros::Timer localization_estimate_timer_;

  // 定位状态与策略共享数据。
  mutable std::mutex state_mtx_;
  LocDecision loc_decision_;
  bool absolute_pose_initialized_ = false;
  bool fuse_ins_twist_ = false;

  // Topic 与 frame 配置。
  std::string ins_topic_;
  std::string fusion_topic_;
  std::string wheel_topic_;
  std::string localization_status_topic_;
  std::string loc_policy_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string localization_estimate_topic_;

  // 输入有效性、状态新鲜度和时间保护参数。
  double state_timeout_sec_;
  double localization_estimate_frequency_ = 100.0;
  int input_queue_size_ = 5;
  int output_queue_size_ = 5;
  double max_input_age_sec_ = 0.5;
  double max_future_stamp_sec_ = 0.05;
  double hard_stale_warn_sec_ = 1.0;
  // LIO PRIMARY 观测方差参数。
  double fusion_primary_xy_variance_ = 0.0005;
  double fusion_primary_vxy_variance_ = 0.005;
  double fusion_primary_yaw_std_deg_ = 0.5;
  double fusion_primary_vyaw_std_deg_ = 0.5;
  // wheel vx 融合、门控与故障回退参数。
  bool wheel_vx_enabled_ = true;
  bool wheel_vx_replace_lio_vx_ = false;
  double wheel_vx_timeout_sec_ = 0.15;
  double wheel_vx_max_abs_ = 3.5;
  double wheel_lio_vx_blend_ = 0.25;
  double wheel_lio_sync_tolerance_sec_ = 0.03;
  double wheel_lio_vx_gate_ = 0.03;
  double wheel_lio_vx_gate_max_dt_ = 0.10;
  double wheel_primary_vx_variance_ = 0.05;
  // 各观测的 Mahalanobis rejection threshold。
  double ins_pose_rejection_threshold_ = 5.0;
  double ins_twist_rejection_threshold_ = 3.0;
  double fusion_pose_rejection_threshold_ = 3.0;
  double fusion_twist_rejection_threshold_ = 3.0;
  double wheel_vx_rejection_threshold_ = 5.0;

  // 各数据源 CallbackData。
  RobotLocalization::CallbackData ins_pose_cb_data_;
  RobotLocalization::CallbackData ins_twist_cb_data_;
  RobotLocalization::CallbackData fusion_pose_cb_data_;
  RobotLocalization::CallbackData fusion_twist_cb_data_;
  RobotLocalization::CallbackData fusion_twist_without_vx_cb_data_;
  RobotLocalization::CallbackData wheel_vx_cb_data_;

  // 运行统计计数器。
  std::atomic<uint64_t> ins_count_{0};
  std::atomic<uint64_t> ins_drop_count_{0};
  std::atomic<uint64_t> fusion_count_{0};
  std::atomic<uint64_t> fusion_drop_count_{0};
  std::atomic<uint64_t> ins_invalid_count_{0};
  std::atomic<uint64_t> ins_stale_count_{0};
  std::atomic<uint64_t> fusion_invalid_count_{0};
  std::atomic<uint64_t> fusion_stale_count_{0};
  std::atomic<uint64_t> wheel_count_{0};
  std::atomic<uint64_t> wheel_accepted_count_{0};
  std::atomic<uint64_t> wheel_blended_count_{0};
  std::atomic<uint64_t> wheel_drop_count_{0};
  std::atomic<uint64_t> wheel_preinit_drop_count_{0};
  std::atomic<uint64_t> wheel_invalid_count_{0};
  std::atomic<uint64_t> wheel_stale_count_{0};
  std::atomic<uint64_t> wheel_outlier_count_{0};
  std::atomic<uint64_t> output_count_{0};
  std::atomic<uint64_t> loop_overrun_count_{0};
  std::atomic<uint64_t> output_regression_count_{0};
  std::atomic<double> last_ins_age_sec_{-1.0};
  std::atomic<double> last_fusion_age_sec_{-1.0};
  std::atomic<double> last_wheel_age_sec_{-1.0};
  std::atomic<double> last_ins_accepted_stamp_sec_{0.0};
  std::atomic<double> last_fusion_accepted_stamp_sec_{0.0};
  std::atomic<double> last_wheel_accepted_stamp_sec_{0.0};
  std::atomic<double> last_lio_raw_stamp_sec_{0.0};
  std::atomic<double> last_lio_raw_vx_{0.0};
  std::atomic<double> last_execution_ms_{0.0};
  mutable std::mutex wheel_history_mtx_;
  std::deque<WheelSample> wheel_history_;
  // 输出时间单调性保护。
  ros::Time last_output_stamp_;
  ros::Time last_cycle_time_;
  uint32_t localization_estimate_sequence_ = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_ekf_node");
  LocEkfNode node;
  // 使用两个 callback 线程，使传感器输入和定时输出可以并发调度。
  // RosEkf 内部通过递归互斥锁保护 measurement queue 和滤波状态；
  // 本节点的 status/policy 数据另外由 state_mtx_ 保护。
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return EXIT_SUCCESS;
}
