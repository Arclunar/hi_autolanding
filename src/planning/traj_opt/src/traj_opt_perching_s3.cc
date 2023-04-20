#include <traj_opt/traj_opt.h>
// 使用S3的轨迹

#include <random>
#include <chrono>

#include <traj_opt/geoutils.hpp>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt_perching_s3 {

static Eigen::Vector3d car_p_,car_v_;
static Eigen::Vector3d tail_q_v_;
static Eigen::Vector3d g_(0,0,-9.8);
static Eigen::Vector3d land_v_;
static Eigen::Vector3d v_t_x_, v_t_y_;

static Trajectory init_traj;
static double init_tail_f_;
static Eigen::Vector2d init_vt_;
static bool initial_guess_ = false; 



//***********************
// 工具函数
//***********************
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}

//* 消除时间约束的，T=exp（t），T对t的导数
static inline double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}


}