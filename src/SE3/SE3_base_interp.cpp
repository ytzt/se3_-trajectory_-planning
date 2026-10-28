
#include "SE3Interpolators.hpp"

Sophus::SE3d SE3BaseInterp::interpolate(const Sophus::SE3d& T0,
                                       const Sophus::SE3d& T1,
                                       double beta) {
    return T0 * Sophus::SE3d::exp(beta * (T0.inverse() * T1).log());
}

std::vector<Sophus::SE3d>
SE3BaseInterp::interpolate(const Sophus::SE3d& T0,
                           const Sophus::SE3d& T1,
                           std::size_t num_steps) const {
    std::vector<Sophus::SE3d> traj;
    traj.reserve(num_steps + 1);
    for (std::size_t i = 0; i <= num_steps; ++i)
        traj.push_back(interpolate(T0, T1, static_cast<double>(i) / num_steps));
    return traj;
}

Eigen::Matrix4d QuatAndTInterpolator::Interpolate(const Eigen::Matrix4d& pose_src, const Eigen::Matrix4d& pose_dst, double beta) {
  Eigen::Matrix4d v_interpolated_poses = Eigen::Matrix4d::Identity();

  // Translation Linear Interpolation
  {
    const Eigen::Vector3d t_src = pose_src.block(0,3,3,1);
    const Eigen::Vector3d t_dst = pose_dst.block(0,3,3,1);
    Eigen::Vector3d t = (t_dst - t_src)*beta + t_src;
    v_interpolated_poses.block(0,3,3,1) = t;

  }

  // Rotation Spherical Interpolation(Quarternion)
  // Algorithm -> https://www.mathworks.com/help/fusion/ref/quaternion.slerp.html
  {
    Eigen::Quaterniond quat_src(Eigen::Matrix3d(pose_src.block(0,0,3,3))); quat_src.normalize();
    Eigen::Quaterniond quat_dst(Eigen::Matrix3d(pose_dst.block(0,0,3,3))); quat_dst.normalize();
    // quat_src.normalize();
    // quat_dst.normalize();
    Eigen::Quaterniond q = quat_src.slerp(beta, quat_dst);   // ← 一行搞定
    v_interpolated_poses.block(0,0,3,3) = q.toRotationMatrix(); 

   
  }

  return v_interpolated_poses;
}