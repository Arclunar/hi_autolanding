#pragma once
#include <ros/ros.h>

#include <chrono>
#include <thread>
#include <vis_utils/vis_utils.hpp>

#include "minco.hpp"

namespace traj_opt_perching {

class TrajOpt_perching {
 public:
  ros::NodeHandle nh_;
  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  bool pause_debug_ = false;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // weight for time regularization term
  double rhoT_, rhoVt_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double rhoP_, rhoV_, rhoA_;
  double rhoThrust_, rhoOmega_;
  double rhoPerchingCollision_;

// 远离多面体
  double clearance_d_=0.2;

  // 降落添加的
  double rhoLanding = 1000000;
  double theta_cone_;
  double landing_circle_r;
  double landing_ld;
  bool using_cone=false;

  // landing parameters
  // robot_l_ 是机器人的中心到底盘的距离，r是机器人的半径
  double v_plus_, robot_l_, robot_r_, platform_r_;
  // SE3 dynamic limitation parameters
  double thrust_max_, thrust_min_;
  double omega_max_, omega_yaw_max_;
  // MINCO Optimizer 
  minco::MINCO_S4_Uniform mincoOpt_; //TrajOpt对象里有一个minco优化器
  Eigen::MatrixXd initS_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double* x_;


// 应该是从elastic tracker那里改过来的
  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<Eigen::Vector3d> tracking_visible_ps_;
  std::vector<double> tracking_thetas_;

  // corridor
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  // 角顶
  Eigen::VectorXd p_;
  // polyH utils
  bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                 std::vector<Eigen::MatrixXd>& vPs) const;

 public:
  TrajOpt_perching(ros::NodeHandle& nh);
  ~TrajOpt_perching() {}


  // 优化函数
  int optimize(const double& delta = 1e-4);

  // 轨迹生成函数
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::Vector3d& car_p,
                     const Eigen::Vector3d& car_v,
                     const Eigen::Quaterniond& land_q,
                     const int& N,
                     Trajectory_S4& traj,bool using_cone, 
                     const double& t_replan = -1.0);
  
  bool generate_traj_corridor(const Eigen::MatrixXd& iniState,
                            const Eigen::Vector3d& car_p,
                            const Eigen::Vector3d& car_v,
                            const Eigen::Quaterniond& land_q,
                            const std::vector<Eigen::Vector3d>& target_predcit,
                            const std::vector<Eigen::MatrixXd>& hPolys,
                            Trajectory_S4& traj,
                            bool using_cone_,
                            const double& t_replan = -1.0) ;

// 可行性检查
  bool feasibleCheck(Trajectory_S4& traj);

// 时间积分惩罚
  void addTimeIntPenalty(double& cost);

// cost函数
//  * @brief 得到单位向量
//  * 
//  * @param x 
  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_thrust(const Eigen::Vector3d& a,
                        Eigen::Vector3d& grada,
                        double& costa);
  bool grad_cost_omega(const Eigen::Vector3d& a,
                       const Eigen::Vector3d& j,
                       Eigen::Vector3d& grada,
                       Eigen::Vector3d& gradj,
                       double& cost);
  bool grad_cost_omega_yaw(const Eigen::Vector3d& a,
                           const Eigen::Vector3d& j,
                           Eigen::Vector3d& grada,
                           Eigen::Vector3d& gradj,
                           double& cost);
  bool grad_cost_floor(const Eigen::Vector3d& p,
                       Eigen::Vector3d& gradp,
                       double& costp);
            
  bool grad_cost_p_corridor(const Eigen::Vector3d& p,
                                   const Eigen::MatrixXd& hPoly,
                                   Eigen::Vector3d& gradp,
                                   double& costp) ;

  
  bool grad_cost_perching_collision(const Eigen::Vector3d& pos,
                                    const Eigen::Vector3d& acc,
                                    const Eigen::Vector3d& car_p,
                                    Eigen::Vector3d& gradp,
                                    Eigen::Vector3d& grada,
                                    Eigen::Vector3d& grad_car_p,
                                    double& cost);

  bool grad_cost_landing_upside(const Eigen::Vector3d& pos,
                                const Eigen::Vector3d& car_p,
                                Eigen::Vector3d& gradp,
                                Eigen::Vector3d& grad_car_p,
                                double& cost);

  bool check_collilsion(const Eigen::Vector3d& pos,
                        const Eigen::Vector3d& acc,
                        const Eigen::Vector3d& car_p);
};

}  // namespace traj_opt