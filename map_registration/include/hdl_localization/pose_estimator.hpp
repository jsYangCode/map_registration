#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <hdl_localization/pose_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief constructor
   * @param registration        registration method
   * @param stamp               timestamp
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  //todo 16个状态，7个观测  state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const ros::Time& stamp, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration = 1.0)
    : init_stamp(stamp),
      registration(registration),//配准方法
      cool_time_duration(cool_time_duration)//  todo 什么是 cool_time
  {
    process_noise = Eigen::MatrixXf::Identity(16, 16);//过程噪声
    process_noise.middleRows(0, 3) *= 1.0;
    process_noise.middleRows(3, 3) *= 1.0;//middleRows()方法 返回4,5,6行
    process_noise.middleRows(6, 4) *= 0.5;
    process_noise.middleRows(10, 3) *= 1e-6;
    process_noise.middleRows(13, 3) *= 1e-6;

    Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);//量测噪声
    measurement_noise.middleRows(0, 3) *= 0.01;  //量测噪声的值很小
    measurement_noise.middleRows(3, 4) *= 0.001;

    Eigen::VectorXf mean(16);//todo 状态初始值，对应与KF里的X0
    mean.middleRows(0, 3) = pos;
    mean.middleRows(3, 3).setZero();
    mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
    mean.middleRows(10, 3).setZero();
    mean.middleRows(13, 3).setZero();

    Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;//todo 状态初始方差 对应于KF里的P0

    PoseSystem system;
    ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
  }

  /**
   * @brief predict
   * @param stamp    timestamp
   * @param acc      acceleration
   * @param gyro     angular velocity
   */
  void predict(const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro, const Eigen::Vector3f odom_translation = Eigen::Vector3f::Zero())
  {//todo gc odom_translation
    if((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp)
    {//初始化
      prev_stamp = stamp;
      return;
    }

    double dt = (stamp - prev_stamp).toSec();
    //std::cout << "in pose_estimator dt = " << dt << std::endl;
    prev_stamp = stamp;

    ukf->setProcessNoiseCov(process_noise * dt);//过程噪声的协方差设置为过程噪声与时间t的乘积  todo 噪声为何这么设置
    ukf->system.dt = dt;

    Eigen::VectorXf control(6);
    control.head<3>() = acc;
    control.tail<3>() = gyro;

      Eigen::Vector3f odom_translation_relative = odom_translation;

    ukf->predict(control,odom_translation_relative);//todo gc odom_translation_relative
  }

  /**
   * @brief correct
   * @param cloud   input cloud
   * @return cloud aligned to the globalmap
   */
  pcl::PointCloud<PointT>::Ptr correct(const pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess.block<3, 3>(0, 0) = quat().toRotationMatrix();//quat()返回对应的四元数
    init_guess.block<3, 1>(0, 3) = pos();//pos()返回对应的位置

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->setInputSource(cloud);//用于陪准的  源点云
    registration->align(*aligned, init_guess);//aligned 陪准后的点云  init_guess是利用control计算得到的初值
      //todo gc 此处可以加一个鲁棒性处理
      //if(registration->getFitnessScore() )
    //std::cout << "in pose_estimator " << registration->getFitnessScore() << std::endl;

    Eigen::Matrix4f trans = registration->getFinalTransformation();//计算得到的位姿变换
    Eigen::Vector3f p = trans.block<3, 1>(0, 3);
    Eigen::Quaternionf q(trans.block<3, 3>(0, 0));

    if(quat().vec().dot(q.vec()) < 0.0f)
    {
      q.coeffs() *= -1.0f;
    }

    Eigen::VectorXf observation(7);
    observation.middleRows(0, 3) = p;
    observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());

    ukf->correct(observation);//将点云陪准的结果作为correct的输入
    return aligned;
  }

  /* getters */
  Eigen::Vector3f pos() const {
    return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
  }

  Eigen::Vector3f vel() const {
    return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
  }

  Eigen::Quaternionf quat() const
  {
    return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
  }

  Eigen::Matrix4f matrix() const
  {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3, 3>(0, 0) = quat().toRotationMatrix();
    m.block<3, 1>(0, 3) = pos();
    return m;
  }

private:
  ros::Time init_stamp;         // when the estimator was initialized
  ros::Time prev_stamp;         // when the estimator was updated last time
  double cool_time_duration;    //

  Eigen::MatrixXf process_noise;
public:
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;

  pcl::Registration<PointT, PointT>::Ptr registration;
};

}

#endif // POSE_ESTIMATOR_HPP
