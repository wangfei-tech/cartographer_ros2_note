/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
// 保持姿势一段时间以估计线速度和角速度。
// 使用速度推断运动。使用 IMU 和/或里程计数据（如果可用）来改进推断。
class PoseExtrapolator : public PoseExtrapolatorInterface {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  common::Time GetLastPoseTime() const override;
  common::Time GetLastExtrapolatedTime() const override;
// 前端计算的位姿后传到pose的接口，更新位姿推测器的变量
  void AddPose(common::Time time, const transform::Rigid3d& pose) override;
  //观测量的更新
  // imu 的数据接口
  void AddImuData(const sensor::ImuData& imu_data) override;
  // 轮速计的数据接口
  void AddOdometryData(const sensor::OdometryData& odometry_data) override;

  // 位姿预测的调用接口
  transform::Rigid3d ExtrapolatePose(common::Time time) override;

  ExtrapolationResult ExtrapolatePosesWithGravity(
      const std::vector<common::Time>& times) override;

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  // 估计重力方向的接口，对应IMU，如果没有IMU，则返回单位四元数
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time) override;

  // override 关键字告诉编译器，该函数应该覆盖基类中的函数
  // 如果该函数没有覆盖任何函数，则会导致编译器报错
  // 没有加这个关键字也没有啥影响，只是少了安全检查

 private:
 // 根据前端解算的位姿，更新速度
  void UpdateVelocitiesFromPoses();
  // 删除imu数据
  void TrimImuData();
  // 删除轮速计的数据
  void TrimOdometryData();
  // 更新外推器
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  //推算旋转矩阵
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  // 推算平移变换矩阵
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

 //存放时间差
  const common::Duration pose_queue_duration_;
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  // 只保留最新的两帧pose
  std::deque<TimedPose> timed_pose_queue_;
  // // 由位姿推算的线速度和角速度
  // 在添加位姿时进行更新，用于位姿预测时不使用里程计数据时 平移量的预测
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  // 在添加位姿时进行更新，用于在不使用IMU数据时候角速度的更新
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();
  // 重力加速度的常量
  const double gravity_time_constant_;
  std::deque<sensor::ImuData> imu_data_; //队列数据的个数，最少为1
  std::unique_ptr<ImuTracker> imu_tracker_;//全局的imu外推器 只在添加位姿时候进行更新，用于保存添加位姿
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;//轮速计外推器 只在添加位姿时进行更新，用于根据里程计数据计算线速度时姿态的预测
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;// 局部的imu外推器 只在添加位姿时候进行更新，用于为预测时的姿态预测
  TimedPose cached_extrapolated_pose_; //缓存最新的一次位姿推测的结果

  std::deque<sensor::OdometryData> odometry_data_;//里程计的数据队列，里程计数据最少为2

  //  // 轮速计数据推算出来的线速度和角速度
  // 只在添加里程计数据的时候进行更新，用于位姿预测时的平移量预测
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  // 只在添加里程计数据的时候进行更新，用于不使用IMU数据时的角速度更新
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
