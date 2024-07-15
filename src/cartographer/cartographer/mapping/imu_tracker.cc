/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()), //初始方向角
      gravity_vector_(Eigen::Vector3d::UnitZ()),  //重力方向初始化为[0,0,1] imu加速度向上为正，向下为负
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}
// 
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  // 上一时刻的角速度乘以时间得到当前时刻相对于上一时刻的预测的姿态变换量，再转换成四元数
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  //使用上一时刻的姿态 乘以姿态变换量得到当前时刻预测出的姿态
  orientation_ = (orientation_ * rotation).normalized();

  // 根据预测出的姿态变换量 ，预测旋转之后的线性加速度的值
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  time_ = time;
}

void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.

  // 1:求delta_t ,delta_t初始时刻为infinity，之后time-last_linear_accelerate
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  
  // 2: 求alpha ，alpha = 1-e^(-delta_t/10)
  // delta_t 越大，alpha 越大
  last_linear_acceleration_time_ = time_;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);

  // 3: 将之前的线加速度与当前传入的线加速度进行融合，这里采用指数滑动平均法
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'. 
  // 指数用来确定权重，因为有噪声的存在，时间差越大，当前的线加速度的权重越大(相信之前的加速度)
  // 4: 求得线加速度的值 与 由上一时刻姿态求出的线性加速度 间的旋转量
  // conjugate()共轭来代替逆 

  //将两个向量生成四元数可以使用 Eigen::Quaterniond::FromTwoVectors 方法。
  //这种方法计算出一个旋转四元数，使得第一个向量旋转到第二个向量
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  
  // step5: 使用这个旋转量来校准当前的姿态
  orientation_ = (orientation_ * rotation).normalized();

  //如果线性加速度与姿态均计算完全正确，那么这二者的乘积因该是0.0.1
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}
// 更新角速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
