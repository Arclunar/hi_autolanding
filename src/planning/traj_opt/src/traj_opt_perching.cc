#include <traj_opt/traj_opt_perching.h>

#include <traj_opt/geoutils.hpp>
#include <traj_opt/lbfgs_perching.hpp>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt_perching {

static Eigen::Vector3d car_p_, car_v_;
static Eigen::Vector3d tail_q_v_;
static Eigen::Vector3d g_(0, 0, -9.8);
static Eigen::Vector3d land_v_;

// 降落时的切向分量基底
static Eigen::Vector3d v_t_x_, v_t_y_;

static Trajectory_S4 init_traj_;
static double init_tail_f_;
static Eigen::Vector2d init_vt_;
static bool initial_guess_ = false;

static double thrust_middle_, thrust_half_;

static double tictoc_innerloop_;
static double tictoc_integral_;

// 迭代次数
static int iter_times_;


//* 这个用来获得zb方向矢量，从姿态得到一个z轴的方向矢量
static bool q2v(const Eigen::Quaterniond& q,
                Eigen::Vector3d& v) {
  Eigen::MatrixXd R = q.toRotationMatrix();
  v = R.col(2); //取第三列,也就是z轴在坐标系下的三个分量
  return true;
}


//* brief 得到单位向量
static Eigen::Vector3d f_N(const Eigen::Vector3d& x) {
  return x.normalized();
}

//* 求f_DN函数
static Eigen::MatrixXd f_DN(const Eigen::Vector3d& x) {
  double x_norm_2 = x.squaredNorm();
  return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
}


static Eigen::MatrixXd f_D2N(const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
  double x_norm_2 = x.squaredNorm();
  double x_norm_3 = x_norm_2 * x.norm();
  Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
  return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
}

// SECTION  variables transformation and gradient transmission

// L1平滑项
static double smoothedL1(const double& x,
                         double& grad) {
  static double mu = 0.01;
  if (x < 0.0) {
    return 0.0;
  } else if (x > mu) {
    grad = 1.0;
    return x - 0.5 * mu;
  } else {
    const double xdmu = x / mu;
    const double sqrxdmu = xdmu * xdmu;
    const double mumxd2 = mu - 0.5 * x;
    grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu); // 对x求导
    return mumxd2 * sqrxdmu * xdmu; //(mu - x/2)(x/mu)^3;
  }
}

// 开关平滑项
static double smoothed01(const double& x,
                         double& grad) {
  static double mu = 0.01;
  static double mu4 = mu * mu * mu * mu;
  static double mu4_1 = 1.0 / mu4;
  if (x < -mu) {
    grad = 0;
    return 0;
  } else if (x < 0) {
    double y = x + mu;
    double y2 = y * y;
    grad = y2 * (mu - 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu - x) * mu4_1;
  } else if (x < mu) {
    double y = x - mu;
    double y2 = y * y;
    grad = y2 * (mu + 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu + x) * mu4_1 + 1;
  } else {
    grad = 0;
    return 1;
  }
}

// 求指数函数的C2近似
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0) 
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}

//* 对数函数的C2近似
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

//* 实际推力，往前计算
static double forward_thrust(const double& f) {
  return thrust_half_ * sin(f) + thrust_middle_;
  // return f;
}

// 
static void addLayerThrust(const double& f,
                           const double& grad_thrust,
                           double& grad_f) {
  grad_f = thrust_half_ * cos(f) * grad_thrust;
  // grad_f = grad_thrust;
}

//* 合成降落速度，加上了切向的速度
static void forwardTailV(const Eigen::Ref<const Eigen::Vector2d>& xy,
                         Eigen::Ref<Eigen::Vector3d> tailV) {
  tailV = land_v_ + xy.x() * v_t_x_ + xy.y() * v_t_y_; // land_v_是车的速度加上降落平面的法向速度
}

//* 位置代理变量
// 从p到P
static void forwardP(const Eigen::Ref<const Eigen::VectorXd>& p,
                     const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                     Eigen::MatrixXd& inP) {
  int M = cfgPolyVs.size();
  Eigen::VectorXd q;
  int j = 0, k;
  for (int i = 0; i < M; ++i) {
    k = cfgPolyVs[i].cols() - 1;
    q = 2.0 / (1.0 + p.segment(j, k).squaredNorm()) * p.segment(j, k);
    inP.col(i) = cfgPolyVs[i].rightCols(k) * q.cwiseProduct(q) +
                 cfgPolyVs[i].col(0);
    j += k;
  }
  return;
}

static double objectiveNLS(void* ptrPOBs,
                           const double* x,
                           double* grad,
                           const int n) {
  const Eigen::MatrixXd& pobs = *(Eigen::MatrixXd*)ptrPOBs;
  Eigen::Map<const Eigen::VectorXd> p(x, n);
  Eigen::Map<Eigen::VectorXd> gradp(grad, n);

  double qnsqr = p.squaredNorm();
  double qnsqrp1 = qnsqr + 1.0;
  double qnsqrp1sqr = qnsqrp1 * qnsqrp1;
  Eigen::VectorXd r = 2.0 / qnsqrp1 * p;

  Eigen::Vector3d delta = pobs.rightCols(n) * r.cwiseProduct(r) +
                          pobs.col(1) - pobs.col(0);
  double cost = delta.squaredNorm();
  Eigen::Vector3d gradR3 = 2 * delta;

  Eigen::VectorXd gdr = pobs.rightCols(n).transpose() * gradR3;
  gdr = gdr.array() * r.array() * 2.0;
  gradp = gdr * 2.0 / qnsqrp1 -
          p * 4.0 * gdr.dot(p) / qnsqrp1sqr;

  return cost;
}

// 从P到p
static void backwardP(const Eigen::Ref<const Eigen::MatrixXd>& inP,
                      const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                      Eigen::VectorXd& p) {
  int M = inP.cols();
  int j = 0, k;

  // Parameters for tiny nonlinear least squares
  double minSqrD;
  lbfgs::lbfgs_parameter_t nls_params;
  lbfgs::lbfgs_load_default_parameters(&nls_params);
  nls_params.g_epsilon = FLT_EPSILON;
  nls_params.max_iterations = 128;

  Eigen::MatrixXd pobs;
  for (int i = 0; i < M; i++) {
    k = cfgPolyVs[i].cols() - 1;
    p.segment(j, k).setConstant(1.0 / (sqrt(k + 1.0) + 1.0));
    pobs.resize(3, k + 2);
    pobs << inP.col(i), cfgPolyVs[i];
    lbfgs::lbfgs_optimize(k,
                          p.data() + j,
                          &minSqrD,
                          &objectiveNLS,
                          nullptr,
                          nullptr,
                          &pobs,
                          &nls_params);
    j += k;
  }
  return;
}

// 算梯度
static void addLayerPGrad(const Eigen::Ref<const Eigen::VectorXd>& p,
                          const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                          const Eigen::Ref<const Eigen::MatrixXd>& gradInPs,
                          Eigen::Ref<Eigen::VectorXd> grad) {
  int M = gradInPs.cols();

  int j = 0, k;
  double qnsqr, qnsqrp1, qnsqrp1sqr;
  Eigen::VectorXd q, r, gdr;
  for (int i = 0; i < M; i++) {
    k = cfgPolyVs[i].cols() - 1;
    q = p.segment(j, k);
    qnsqr = q.squaredNorm();
    qnsqrp1 = qnsqr + 1.0;
    qnsqrp1sqr = qnsqrp1 * qnsqrp1;
    r = 2.0 / qnsqrp1 * q;
    gdr = cfgPolyVs[i].rightCols(k).transpose() * gradInPs.col(i);
    gdr = gdr.array() * r.array() * 2.0;

    grad.segment(j, k) = gdr * 2.0 / qnsqrp1 -
                         q * 4.0 * gdr.dot(q) / qnsqrp1sqr;
    j += k;
  }
  return;
}



// !SECTION variables transformation and gradient transmission

// h表示变成v表示
bool TrajOpt_perching::extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                        std::vector<Eigen::MatrixXd>& vPs) const {
  const int M = hPs.size() - 1;

  vPs.clear();
  vPs.reserve(2 * M + 1);

  int nv;
  Eigen::MatrixXd curIH, curIV, curIOB;
  for (int i = 0; i < M; i++) {
    if (!geoutils::enumerateVs(hPs[i], curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
    curIH << hPs[i], hPs[i + 1];
    if (!geoutils::enumerateVs(curIH, curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);
  }

  if (!geoutils::enumerateVs(hPs.back(), curIV)) {
    return false;
  }
  nv = curIV.cols();
  curIOB.resize(3, nv);
  curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
  vPs.push_back(curIOB);

  return true;
}





// SECTION object function、
// 目标函数
static inline double objectiveFunc(void* ptrObj, // 优化对象
                                   const double* x, // 优化变量
                                   double* grad, // 梯度
                                   const int n) { // 迭代次数
  // std::cout << "damn" << std::endl;
  iter_times_++;
  TrajOpt_perching& obj = *(TrajOpt_perching*)ptrObj; // 取值，得到轨迹对象的引用
  const double& t = x[0]; // 取出时间
  double& gradt = grad[0]; // 得到时间的梯度

  // 取出优化变量及其导数映射成矩阵
  Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_); // 把位置数据变成3乘N的矩阵
  Eigen::Map<Eigen::MatrixXd> gradP(grad + obj.dim_t_, 3, obj.dim_p_); // 位置的梯度
  const double& tail_f = x[obj.dim_t_ + obj.dim_p_ * 3]; // 末端推力
  double& grad_f = grad[obj.dim_t_ + obj.dim_p_ * 3]; // 末端推力的梯度
  Eigen::Map<const Eigen::Vector2d> vt(x + obj.dim_t_ + 3 * obj.dim_p_ + 1); // 末端切线速度
  Eigen::Map<Eigen::Vector2d> grad_vt(grad + obj.dim_t_ + 3 * obj.dim_p_ + 1); // 末端切向速度梯度

  // 获得时间分配和终端速度
  double dT = expC2(t); // dT是实际的一段内的时间
  Eigen::Vector3d tailV, grad_tailV;
  forwardTailV(vt, tailV); // 根据切向速度vt得到最终的整体降落速度tailV，vt是切向速度

  // 末端状态矩阵
  Eigen::MatrixXd tailS(3, 4);
  tailS.col(0) = car_p_ + car_v_ * obj.N_ * dT + tail_q_v_ * obj.robot_l_; // 假设车匀速直线运动，这里的车的速度
  tailS.col(1) = tailV;
  tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_; // 末端加速度，推力方向要求和降落平台垂直
  tailS.col(3).setZero(); // jerk

  // not suffer from problem , solve it .
  // auto tic = std::chrono::steady_clock::now();
  obj.mincoOpt_.generate(obj.initS_, tailS, P, dT); // 生成系数,为了获得cost

  double cost = obj.mincoOpt_.getTrajSnapCost(); // 获得snap的cost，套公式
  obj.mincoOpt_.calGrads_CT(); // 计算对CT的导数，没改

  // auto toc = std::chrono::steady_clock::now();
  // tictoc_innerloop_ += (toc - tic).count();
  // double cost_with_only_energy = cost;
  // std::cout << "cost of energy: " << cost_with_only_energy << std::endl;

  // tic = std::chrono::steady_clock::now();
  obj.addTimeIntPenalty(cost); // 计算时间积分，这里改了
  // toc = std::chrono::steady_clock::now();
  // tictoc_integral_ += (toc - tic).count();

  // tic = std::chrono::steady_clock::now();
  obj.mincoOpt_.calGrads_PT(); // 没改
  // toc = std::chrono::steady_clock::now();
  // tictoc_innerloop_ += (toc - tic).count();
  // std::cout << "cost of penalty: " << cost - cost_with_only_energy << std::endl;

  obj.mincoOpt_.gdT += obj.mincoOpt_.gdTail.col(0).dot(obj.N_ * car_v_); // 这里对时间的梯度加了一项

  // 得到末端速度的梯度
  grad_tailV = obj.mincoOpt_.gdTail.col(1); 
  // 末端推力
  double grad_thrust = obj.mincoOpt_.gdTail.col(2).dot(tail_q_v_); //获得目标函数对末段推力的梯度，为什么这么算呢，tail_q_v_是平台垂直法向量

  addLayerThrust(tail_f, grad_thrust, grad_f); //计算关于f的梯度， f是优化的f，把结果计算在grad_f中
  // grad_f = thrust_half_ * cos(f) * grad_thrust;

  // 切向速度的正则项系数，正则项直接就是vt的范数，很简单
  if (obj.rhoVt_ > -1) {
    grad_vt.x() = grad_tailV.dot(v_t_x_);
    grad_vt.y() = grad_tailV.dot(v_t_y_);
    double vt_sqr = vt.squaredNorm();
    cost += obj.rhoVt_ * vt_sqr; 
    grad_vt += obj.rhoVt_ * 2 * vt; 
  }

  obj.mincoOpt_.gdT += obj.rhoT_; 
  cost += obj.rhoT_ * dT; // 时间正则项，只有一个时间，所以是时间均匀的
  gradt = obj.mincoOpt_.gdT * gdT2t(t); //转化成代理变量

  gradP = obj.mincoOpt_.gdP;

  return cost;
}

// !SECTION object function

static inline double objectiveFunc_corridor(void* ptrObj,
                                          const double* x,
                                          double *grad,
                                          const int n){

      iter_times_++;
      TrajOpt_perching& obj = *(TrajOpt_perching*)ptrObj; 

      const double& t = x[0]; // 取出时间
      double& gradt = grad[0]; // 得到时间的梯度
      Eigen::Map<const Eigen::VectorXd> p(x + obj.dim_t_, obj.dim_p_);
      Eigen::Map<Eigen::VectorXd> gradp(grad + obj.dim_t_, obj.dim_p_);
      
      const double& tail_f = x[obj.dim_t_ + obj.dim_p_]; // 末端推力
      double& grad_f = grad[obj.dim_t_ + obj.dim_p_]; // 末端推力的梯度
      Eigen::Map<const Eigen::Vector2d> vt(x + obj.dim_t_ + obj.dim_p_ + 1); // 末端切线速度
      Eigen::Map<Eigen::Vector2d> grad_vt(grad + obj.dim_t_ + obj.dim_p_ + 1); // 末端切向速度梯度

      double dT = expC2(t); // dT是实际的一段内的时间

      Eigen::Vector3d tailV, grad_tailV;
      forwardTailV(vt, tailV); // 根据切向速度vt得到最终的整体降落速度tailV，vt是切向速度

      Eigen::VectorXd T(obj.N_);
      Eigen::MatrixXd P(3, obj.N_ - 1);
      forwardP(p, obj.cfgVs_, P);

      // 末端状态矩阵
      Eigen::MatrixXd tailS(3, 4);
      tailS.col(0) = car_p_ + car_v_ * obj.N_ * dT + tail_q_v_ * obj.robot_l_; // 假设车匀速直线运动，这里的车的速度
      tailS.col(1) = tailV;
      tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_; // 末端加速度，推力方向要求和降落平台垂直
      tailS.col(3).setZero(); // jerk

      obj.mincoOpt_.generate(obj.initS_, tailS, P, dT); // 生成系数,为了获得cost

      double cost = obj.mincoOpt_.getTrajSnapCost(); // 获得snap的cost，套公式

      obj.mincoOpt_.calGrads_CT(); // 计算对CT的导数，没改

      obj.addTimeIntPenalty(cost); // 计算时间积分，这里改了
      obj.mincoOpt_.calGrads_PT(); // 没改
      obj.mincoOpt_.gdT += obj.mincoOpt_.gdTail.col(0).dot(obj.N_ * car_v_); // 这里对时间的梯度加了一项

      grad_tailV = obj.mincoOpt_.gdTail.col(1); 
      double grad_thrust = obj.mincoOpt_.gdTail.col(2).dot(tail_q_v_); //获得目标函数对末段推力的梯度，为什么这么算呢，tail_q_v_是平台垂直法向量
      addLayerThrust(tail_f, grad_thrust, grad_f); //计算关于f的梯度， f是优化的f，把结果计算在grad_f中

      if (obj.rhoVt_ > -1) {
      grad_vt.x() = grad_tailV.dot(v_t_x_);
      grad_vt.y() = grad_tailV.dot(v_t_y_);
      double vt_sqr = vt.squaredNorm();
      cost += obj.rhoVt_ * vt_sqr; 
      grad_vt += obj.rhoVt_ * 2 * vt; 
    }

    obj.mincoOpt_.gdT += obj.rhoT_; 
    cost += obj.rhoT_ * dT; // 时间正则项，只有一个时间，所以是时间均匀的

    gradt = obj.mincoOpt_.gdT * gdT2t(t); //转化成时间代理变量

    addLayerPGrad(p, obj.cfgVs_, obj.mincoOpt_.gdP, gradp); //转化成位置代理变量

  return cost;
}

// 退出时进行可视化
// 是否提前退出
static inline int earlyExit(void* ptrObj,
                            const double* x,
                            const double* grad,
                            const double fx,
                            const double xnorm,
                            const double gnorm,
                            const double step,
                            int n,
                            int k,
                            int ls) {
  TrajOpt_perching& obj = *(TrajOpt_perching*)ptrObj;
  if (obj.pause_debug_) {     // 暂停debug
    const double& t = x[0];   // 获取时间
    Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_); // 获取中间点
    const double& tail_f = x[obj.dim_t_ + obj.dim_p_ * 3]; //获取终端推力
    Eigen::Map<const Eigen::Vector2d> vt(x + obj.dim_t_ + 3 * obj.dim_p_ + 1); // 获取终端切向速度

    double dT = expC2(t);     // 实际时间
    double T = obj.N_ * dT;   //总时间
    Eigen::Vector3d tailV;    // 终端速度
    forwardTailV(vt, tailV);

    Eigen::MatrixXd tailS(3, 4);
    tailS.col(0) = car_p_ + car_v_ * T + tail_q_v_ * obj.robot_l_;
    tailS.col(1) = tailV;
    tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_;
    tailS.col(3).setZero();

    obj.mincoOpt_.generate(obj.initS_, tailS, P, dT);
    auto traj = obj.mincoOpt_.getTraj();
    obj.visPtr_->visualize_traj(traj, "debug_traj");
    std::vector<Eigen::Vector3d> int_waypts;
    for (const auto& piece : traj) {
      const auto& dur = piece.getDuration();
      for (int i = 0; i < obj.K_; ++i) {
        double t = dur * i / obj.K_;
        int_waypts.push_back(piece.getPos(t));
      }
    }
    obj.visPtr_->visualize_pointcloud(int_waypts, "int_waypts");

    // NOTE pause
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // return k > 1e3;
  return 0;
}

// 边值问题
static void bvp(const double& t,
                const Eigen::MatrixXd i_state,
                const Eigen::MatrixXd f_state,
                CoefficientMat_S4& coeffMat) {
  double t1 = t;
  double t2 = t1 * t1; //t的平方
  double t3 = t2 * t1;  //t的三次方
  double t4 = t2 * t2; 
  double t5 = t3 * t2;
  double t6 = t3 * t3;
  double t7 = t4 * t3; // 的七次方
  CoefficientMat_S4 boundCond; // 3行8列
  boundCond.leftCols(4) = i_state; // 左边四列是初始条件
  boundCond.rightCols(4) = f_state; // 右边四列是末端条件

  coeffMat.col(0) = (boundCond.col(7) / 6.0 + boundCond.col(3) / 6.0) * t3 +
                    (-2.0 * boundCond.col(6) + 2.0 * boundCond.col(2)) * t2 +
                    (10.0 * boundCond.col(5) + 10.0 * boundCond.col(1)) * t1 +
                    (-20.0 * boundCond.col(4) + 20.0 * boundCond.col(0));
  coeffMat.col(1) = (-0.5 * boundCond.col(7) - boundCond.col(3) / 1.5) * t3 +
                    (6.5 * boundCond.col(6) - 7.5 * boundCond.col(2)) * t2 +
                    (-34.0 * boundCond.col(5) - 36.0 * boundCond.col(1)) * t1 +
                    (70.0 * boundCond.col(4) - 70.0 * boundCond.col(0));
  coeffMat.col(2) = (0.5 * boundCond.col(7) + boundCond.col(3)) * t3 +
                    (-7.0 * boundCond.col(6) + 10.0 * boundCond.col(2)) * t2 +
                    (39.0 * boundCond.col(5) + 45.0 * boundCond.col(1)) * t1 +
                    (-84.0 * boundCond.col(4) + 84.0 * boundCond.col(0));
  coeffMat.col(3) = (-boundCond.col(7) / 6.0 - boundCond.col(3) / 1.5) * t3 +
                    (2.5 * boundCond.col(6) - 5.0 * boundCond.col(2)) * t2 +
                    (-15.0 * boundCond.col(5) - 20.0 * boundCond.col(1)) * t1 +
                    (35.0 * boundCond.col(4) - 35.0 * boundCond.col(0));
  coeffMat.col(4) = boundCond.col(3) / 6.0;
  coeffMat.col(5) = boundCond.col(2) / 2.0;
  coeffMat.col(6) = boundCond.col(1);
  coeffMat.col(7) = boundCond.col(0);

  coeffMat.col(0) = coeffMat.col(0) / t7;
  coeffMat.col(1) = coeffMat.col(1) / t6;
  coeffMat.col(2) = coeffMat.col(2) / t5;
  coeffMat.col(3) = coeffMat.col(3) / t4;
}

// 获取最大转速
static double getMaxOmega(Trajectory_S4& traj) {
  double dt = 0.01;
  double max_omega = 0;
  for (double t = 0; t < traj.getTotalDuration(); t += dt) { // 采样取点
    Eigen::Vector3d a = traj.getAcc(t);
    Eigen::Vector3d j = traj.getJer(t);
    Eigen::Vector3d thrust = a - g_;
    Eigen::Vector3d zb_dot = f_DN(thrust) * j;
    double omega12 = zb_dot.norm();
    if (omega12 > max_omega) {
      max_omega = omega12;
    }
  }
  return max_omega;
}



// for landing in the car of zhw
bool TrajOpt_perching::generate_traj_corridor(const Eigen::MatrixXd& iniState,
                            const Eigen::Vector3d& car_p,
                            const Eigen::Vector3d& car_v,
                            const Eigen::Quaterniond& land_q,
                            const std::vector<Eigen::Vector3d>& target_predcit,
                            const std::vector<Eigen::MatrixXd>& hPolys,
                          Trajectory_S4& traj,bool using_cone_,const double& t_replan) {

    //获得凸包
    cfgHs_ = hPolys;
    if (cfgHs_.size() == 1) {
      cfgHs_.push_back(cfgHs_[0]);//再来一个
    }

    // 变成V表示
    if (!extractVs(cfgHs_, cfgVs_)) {
      ROS_ERROR("extractVs fail!");
      return false;
    }

    N_ = 2 * cfgHs_.size(); // 一个走廊里有2个多项式

    using_cone = using_cone_;


    // 初始条件
    initS_ = iniState;
    car_p_ =car_p;
    car_v_ = car_v;


    // 末段条件
    q2v(land_q, tail_q_v_); 
    thrust_middle_ = (thrust_max_ + thrust_min_) / 2;
    thrust_half_ = (thrust_max_ - thrust_min_) / 2;
    land_v_ = car_v - tail_q_v_ * v_plus_; 
    v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 0, 1));// 其实只要和tail_q_v垂直就行了，叉乘得到结果即可，另一个矢量随意，这里刚好是如果tail_q_v垂直，就换另一个
    if (v_t_x_.squaredNorm() == 0) {
      v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 1, 0));
    }
    v_t_x_.normalize();
    v_t_y_ = tail_q_v_.cross(v_t_x_);
    v_t_y_.normalize();


    dim_t_ = 1;
    dim_p_ = 0;

    for (const auto& cfgV : cfgVs_) {
      dim_p_ += cfgV.cols() - 1; // p变量的个数等于角点数-1
    }

    p_.resize(dim_p_);

    x_ = new double[dim_t_ + dim_p_ + 1 + 2];  // 1: tail thrust; 2: tail vt

    
    double& t = x_[0];                                               // 代理变量
    Eigen::Map<Eigen::VectorXd> p(x_ + dim_t_, dim_p_);         // 其实就是一个3xN矩阵
    double& tail_f = x_[dim_t_ + dim_p_];                       // 取出最后的推力值
    Eigen::Map<Eigen::Vector2d> vt(x_ + dim_t_ + dim_p_ + 1);   // 二维向量

    Eigen::MatrixXd P(3, N_ - 1); // 实际的waypoint个数


    tracking_ps_ = target_predcit; // 只用到预测的位置

    //初始化一个时间T，然后变成t 
    double init_T = (initS_.col(0) - car_p).norm() / vmax_ ;
    t = logC2(init_T / N_ ); 

    // 初始化航点
    for(int i=0;i<N_-1;++i)
    {
      int k = cfgVs_[i].cols() - 1;
      P.col(i) = cfgVs_[i].rightCols(k).rowwise().sum() / (1.0 + k) + cfgVs_[i].col(0); // 从这里得到初始的位置
    }
    backwardP(P, cfgVs_, p_);
    p = p_;

    // 初始化末段切向速度
    vt.setConstant(0.0);

    // Optimization
    lbfgs_perching::lbfgs_parameter_t lbfgs_params;
    lbfgs_perching::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 32;
    lbfgs_params.past = 3;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.min_step = 1e-16;
    lbfgs_params.delta = 1e-4;
    lbfgs_params.line_search_type = 0;
    double minObjective;
    int opt_ret = 0;

    iter_times_ = 0;
    // 优化对象是x_
    opt_ret = lbfgs_perching::lbfgs_optimize(dim_t_ + dim_p_ + 1 + 2, x_, &minObjective,
                                  &objectiveFunc_corridor, nullptr,
                                  &earlyExit, this, &lbfgs_params);
    

    if (opt_ret < 0) {
    delete[] x_;
    ROS_WARN("[ generate_traj] optimization failed");
    return false; // 优化失败
  }

    // 获得轨迹
    double dT = expC2(t);
    double T = N_ * dT; 
    Eigen::Vector3d tailV;
    forwardTailV(vt, tailV); 
    Eigen::MatrixXd tailS(3, 4);
    tailS.col(0) = car_p_ + car_v_ * T + tail_q_v_ * robot_l_; 
    tailS.col(1) = tailV; 
    tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_; 
    tailS.col(3).setZero();

    mincoOpt_.generate(initS_, tailS, P, dT); // 根据waypoint，时间分配、初始和末端状态生成轨迹，也就是得到参数
    traj = mincoOpt_.getTraj(); 

    // for warm start
    init_traj_ = traj;
    init_tail_f_ = tail_f;
    init_vt_ = vt;
    initial_guess_ = true;
    delete[] x_;
    return true;
}



/**
 * @brief 关键函数，生成轨迹
 * 
 * @param iniState  初始状态
 * @param car_p  移动平台的初始位置
 * @param car_v  移动平台的速度
 * @param land_q  移动平台的姿态
 * @param N       轨迹被切分成多少段
 * @param traj   轨迹对象，多项式拟合
 * @param t_replan 
 * @return true 
 * @return false 
 */
bool TrajOpt_perching::generate_traj(const Eigen::MatrixXd& iniState,
                            const Eigen::Vector3d& car_p,
                            const Eigen::Vector3d& car_v,
                            const Eigen::Quaterniond& land_q,
                            const int& N,
                            Trajectory_S4& traj,bool using_cone_,
                            const double& t_replan) {
  N_ = N;

  using_cone = using_cone_;

  dim_t_ = 1; // 时间分配维度为1维
  
  dim_p_ = N_ - 1; // waypoint个数为N-1，段数少1

  // x_是全局变量
  // 一维double数组，新建一个内存，保存时间、每个时间上的位置、最后的推力和最后的切向速度这些优化变量
  x_ = new double[dim_t_ + 3 * dim_p_ + 1 + 2];  // 1: tail thrust; 2: tail vt


  double& t = x_[0]; // 这个t的优化的时间，也就是实际时间取对数，用的引用，所以t变了x_的第一个元素也会变,同理，反正两者指向同样的内存

  // 这个map在P改变后，x_对应元素会不会改变 !!!!!!!! 我是对的，P变了x_对应元素也会变
  Eigen::Map<Eigen::MatrixXd> P(x_ + dim_t_, 3, dim_p_); // 其实就是一个3xN矩阵
  double& tail_f = x_[dim_t_ + 3 * dim_p_]; // 取出最后的推力值
  Eigen::Map<Eigen::Vector2d> vt(x_ + dim_t_ + 3 * dim_p_ + 1); // 二维向量
  car_p_ = car_p; // 移动平台位置
  car_v_ = car_v; // 移动平台速度
  // std::cout << "land_q: "
  //           << land_q.w() << ","
  //           << land_q.x() << ","
  //           << land_q.y() << ","
  //           << land_q.z() << "," << std::endl;


  q2v(land_q, tail_q_v_); // 降落平台的法向量，也就是降落位姿的z轴。由平台的姿态取的
  thrust_middle_ = (thrust_max_ + thrust_min_) / 2;
  thrust_half_ = (thrust_max_ - thrust_min_) / 2;


  // 除了切向速度以外的降落速度，v_plus_是垂直与平面撞击的速度，负号是因为速度和降落平台法线方向相反
  land_v_ = car_v - tail_q_v_ * v_plus_; 
  // std::cout << "tail_q_v_: " << tail_q_v_.transpose() << std::endl;

  // 降落时切向速度的基底
  // 基底直接对降落时的法向速度求叉乘
  // tail_q_v 是降落平台姿态的z轴
  v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 0, 1));// 其实只要和tail_q_v垂直就行了，叉乘得到结果即可，另一个矢量随意，这里刚好是如果tail_q_v垂直，就换另一个
  if (v_t_x_.squaredNorm() == 0) {
    v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 1, 0));
  }
  v_t_x_.normalize();
  v_t_y_ = tail_q_v_.cross(v_t_x_);
  v_t_y_.normalize();

  // 数值全设为0
  vt.setConstant(0.0);

  // NOTE set boundary conditions
  initS_ = iniState; // 初始状态：初始的位置，速度，加速度，jerk

  //! set initial guess with obvp minimum jerk + rhoT
  mincoOpt_.reset(N_);  

  tail_f = 0;

  // 曾经优化过
  bool opt_once = initial_guess_ && t_replan > 0 && t_replan < init_traj_.getTotalDuration();

  // initial guess已经完成
  if (opt_once) { // 已经进行过initial guess了，warm start过了
    double init_T = init_traj_.getTotalDuration() - t_replan; // t_replan下一次replan的时间
    t = logC2(init_T / N_); // 从实际时间得到优化的时间 实际时间=exp(优化时间)
    for (int i = 1; i < N_; ++i) {
      double tt0 = (i * 1.0 / N_) * init_T; //N段，每段的初始时间
      P.col(i - 1) = init_traj_.getPos(tt0 + t_replan); //从初始轨迹中得到位置，根据时间得到轨迹，牛，肯定是一个多项式
    }
    tail_f = init_tail_f_;
    vt = init_vt_;
  } else { // 第一次guess
    Eigen::MatrixXd bvp_i = initS_; 
    Eigen::MatrixXd bvp_f(3, 4);

    // 终止状态，一步到位进行求解
    bvp_f.col(0) = car_p_; //车的位置
    bvp_f.col(1) = car_v_; //车的速度
    bvp_f.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_; // 飞机在倾斜平台上的加速度
    bvp_f.col(3).setZero(); //jerk
    double T_bvp = (bvp_f.col(0) - bvp_i.col(0)).norm() / vmax_; // 时间分配的初值是车位置和飞机初始位置的差值处于最大速度
    CoefficientMat_S4 coeffMat; // 3行8列
    double max_omega = 0;

    // 迭代求解边值问题，不断增加时间，直到max_omega小于1.5倍的omega_max_
    // 直接飞到车上的最佳路径
    do {
      T_bvp += 1.0; // 这是实际时间
      bvp_f.col(0) = car_p_ + car_v_ * T_bvp; // 终止状态位置，假设车匀速直线运动
      bvp(T_bvp, bvp_i, bvp_f, coeffMat); //解边值问题
      std::vector<double> durs{T_bvp};
      std::vector<CoefficientMat_S4> coeffs{coeffMat}; //一堆系数
      Trajectory_S4 traj(durs, coeffs); // 根据系数生成轨迹
      max_omega = getMaxOmega(traj);
    } while (max_omega > 1.3 * omega_max_); // 原来1.5


  // ？ 
  // 这里应该是根据边值问题的解得到每个时刻的位置
    Eigen::VectorXd tt(8); // 8✖1向量，表示每段的时间累计值，用来一步求得位置
    tt(7) = 1.0;
    for (int i = 1; i < N_; ++i) {
      double tt0 = (i * 1.0 / N_) * T_bvp; //每段的结束时间
      for (int j = 6; j >= 0; j -= 1) {
        tt(j) = tt(j + 1) * tt0; // tt0的n次方，索引值越小，次数越高，
      }
      P.col(i - 1) = coeffMat * tt; //算出每个时间点上的位置，这里实际上把结果放在了x_所指向的内存上
    }
    t = logC2(T_bvp / N_); //每段的实际时间转化为优化的时间变量，每段，优化时间可以是负的
  }
  // std::cout << "initial guess >>> t: " << t << std::endl;
  // std::cout << "initial guess >>> tail_f: " << tail_f << std::endl;
  // std::cout << "initial guess >>> vt: " << vt.transpose() << std::endl;

  //! initital guess

  // NOTE optimization
  // lbfgs优化
  lbfgs_perching::lbfgs_parameter_t lbfgs_params;
  lbfgs_perching::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 32;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.min_step = 1e-16;
  lbfgs_params.delta = 1e-4;
  lbfgs_params.line_search_type = 0;
  double minObjective;

  int opt_ret = 0;

  auto tic = std::chrono::steady_clock::now();
  tictoc_innerloop_ = 0;
  tictoc_integral_ = 0;

  iter_times_ = 0;
  // 优化对象是x_
  opt_ret = lbfgs_perching::lbfgs_optimize(dim_t_ + 3 * dim_p_ + 1 + 2, x_, &minObjective,
                                  &objectiveFunc, nullptr,
                                  &earlyExit, this, &lbfgs_params);

  // ! 从这里，时间t，每个时间的位置P，终点的推力、切向速度都已经优化出来了

  auto toc = std::chrono::steady_clock::now();

  std::cout << "\033[32m>ret: " << opt_ret << "\033[0m" << std::endl;

  // std::cout << "innerloop costs: " << tictoc_innerloop_ * 1e-6 << "ms" << std::endl;
  // std::cout << "integral costs: " << tictoc_integral_ * 1e-6 << "ms" << std::endl;
  std::cout << "optmization costs: " << (toc - tic).count() * 1e-6 << "ms" << std::endl;
  // std::cout << "\033[32m>iter times: " << iter_times_ << "\033[0m" << std::endl;
  if (pause_debug_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  if (opt_ret < 0) {
    delete[] x_;
    ROS_WARN("[ generate_traj] optimization failed");
    return false; // 优化失败
  }

  double dT = expC2(t); // dT才是实际的每段时间
  double T = N_ * dT; // 总时间
  Eigen::Vector3d tailV;
  forwardTailV(vt, tailV); // 从vt得到tailV，也就是得到降落时的速度
  Eigen::MatrixXd tailS(3, 4);
  tailS.col(0) = car_p_ + car_v_ * T + tail_q_v_ * robot_l_; //降落位置 重心距离底盘中心的距离
  tailS.col(1) = tailV; // 降落速度
  tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_; // 加速度，和推力绑定
  tailS.col(3).setZero(); // jerkz直接设为0
  // std::cout << "tail thrust: " << forward_thrust(tail_f) << std::endl;
  // std::cout << tailS << std::endl;

  
  mincoOpt_.generate(initS_, tailS, P, dT); // 根据waypoint，时间分配、初始和末端状态生成轨迹，也就是得到参数
  traj = mincoOpt_.getTraj(); 

  // std::cout << "tailV: " << tailV.transpose() << std::endl;
  // std::cout << "maxOmega: " << getMaxOmega(traj) << std::endl;
  // std::cout << "maxThrust: " << traj.getMaxThrust() << std::endl;

  // for warm start
  init_traj_ = traj;
  init_tail_f_ = tail_f;
  init_vt_ = vt;
  initial_guess_ = true;
  delete[] x_;
  return true;
}


//  时间积分惩罚项
void TrajOpt_perching::addTimeIntPenalty(double& cost) {
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_tmp2, grad_tmp3, grad_p, grad_v, grad_a, grad_j;
  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
  double s1, s2, s3, s4, s5, s6, s7;
  double step, alpha;
  Eigen::Matrix<double, 8, 3> gradViola_c;
  double gradViola_t;
  double omg;

  int innerLoop = K_ + 1;
  step = mincoOpt_.t(1) / K_;

  s1 = 0.0;

  // s=4，2s=8，8个系数，最高7次
  for (int j = 0; j < innerLoop; ++j) {
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    s6 = s4 * s2;
    s7 = s4 * s3;
    beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
    beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
    alpha = 1.0 / K_ * j; // 对时间求导会用上
    omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

    for (int i = 0; i < N_; ++i) {
      const auto& c = mincoOpt_.c.block<8, 3>(i * 8, 0);

      pos = c.transpose() * beta0; // 求出key point时刻的pvaj
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;
      snp = c.transpose() * beta4;

      grad_p.setZero();
      grad_v.setZero();
      grad_a.setZero();
      grad_j.setZero();
      grad_tmp3.setZero(); // 对car_p的导数
      cost_inner = 0.0;

      // 加入各种惩罚项和梯度
      if (grad_cost_floor(pos, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      // 加入走廊约束


      if (grad_cost_v(vel, grad_tmp, cost_tmp)) {
        grad_v += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_thrust(acc, grad_tmp, cost_tmp)) {
        grad_a += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_omega(acc, jer, grad_tmp, grad_tmp2, cost_tmp)) {
        grad_a += grad_tmp;
        grad_j += grad_tmp2;
        cost_inner += cost_tmp;
      }
                                                                                                                                          
      if (grad_cost_omega_yaw(acc, jer, grad_tmp, grad_tmp2, cost_tmp)) {
        grad_a += grad_tmp;
        grad_j += grad_tmp2;
        cost_inner += cost_tmp;
      }



      Eigen::Vector3d grad_car_p;
      grad_car_p.setZero();

      double dur2now = (i + alpha) * mincoOpt_.t(1);      // t(1)表示每段的时间，这个就是轨迹积分到的某一个时间到现在的时间
      Eigen::Vector3d car_p = car_p_ + car_v_ * dur2now;  // 求出该时间车的位置


      
      if (grad_cost_perching_collision(pos, acc, car_p,
                                       grad_tmp, grad_tmp2, grad_tmp3,
                                       cost_tmp)) { 
        // grad_tmp   gradp
        // grad_tmp2  grada
        // grad_tmp3  grad_car_p
        grad_p += grad_tmp;
        grad_a += grad_tmp2;
        cost_inner += cost_tmp;
        grad_car_p = grad_tmp3;
      }

      if(using_cone)
      {
        if(grad_cost_landing_upside(pos,car_p,grad_tmp,grad_tmp2,cost_tmp))
        {
          grad_p += grad_tmp;
          cost_inner += cost_tmp;
          grad_car_p += grad_tmp2;
        }
      }

      // 这里统一了grad_car_p代替grad_tmp3
      double grad_car_t = grad_car_p.dot(car_v_); // grad_tmp3目标函数对车位置的导数 // 这里要推一下

      gradViola_c = beta0 * grad_p.transpose(); // 对位置的梯度转到对c的梯度
      gradViola_t = grad_p.transpose() * vel;
      gradViola_c += beta1 * grad_v.transpose();
      gradViola_t += grad_v.transpose() * acc;
      gradViola_c += beta2 * grad_a.transpose();
      gradViola_t += grad_a.transpose() * jer;
      gradViola_c += beta3 * grad_j.transpose();
      gradViola_t += grad_j.transpose() * snp;
      gradViola_t += grad_car_t;

      // 最后把所有梯度传到C和T上面
      mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViola_c; //系数梯度
      mincoOpt_.gdT += omg * (cost_inner / K_ + alpha * step * gradViola_t); // 
      mincoOpt_.gdT += i * omg * step * grad_car_t; // 关于车的时间约束
      cost += omg * step * cost_inner; // 加到cost里面
    }
    s1 += step;
  }
}

TrajOpt_perching::TrajOpt_perching(ros::NodeHandle& nh) :nh_(nh){
  // nh.getParam("N", N_);
  nh.getParam("K_perching", K_);
  // load dynamic paramters
  nh.getParam("vmax_perching", vmax_);
  nh.getParam("amax_perching", amax_);
  nh.getParam("thrust_max", thrust_max_);
  nh.getParam("thrust_min", thrust_min_);
  nh.getParam("omega_max", omega_max_);
  nh.getParam("omega_yaw_max", omega_yaw_max_);
  nh.getParam("v_plus", v_plus_);
  nh.getParam("robot_l", robot_l_);
  nh.getParam("robot_r", robot_r_);
  nh.getParam("platform_r", platform_r_);
  nh.getParam("rhoT_perching", rhoT_);
  nh.getParam("rhoVt", rhoVt_);
  nh.getParam("rhoP_perching", rhoP_);
  nh.getParam("rhoV_perching", rhoV_);
  nh.getParam("rhoA_perching", rhoA_);
  nh.getParam("rhoThrust", rhoThrust_);
  nh.getParam("rhoOmega", rhoOmega_);
  nh.getParam("rhoPerchingCollision", rhoPerchingCollision_);
  nh.getParam("pause_debug", pause_debug_);
  nh.getParam("rhoLanding",rhoLanding);
  nh.getParam("theta_cone",theta_cone_);
  nh.getParam("landing_circle",landing_circle_r);
  nh.getParam("landing_ld",landing_ld);
  nh.getParam("using_cone",using_cone);


  visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
}


// 走廊约束
bool TrajOpt_perching::grad_cost_p_corridor(const Eigen::Vector3d& p,
                                   const Eigen::MatrixXd& hPoly,
                                   Eigen::Vector3d& gradp,
                                   double& costp) {
  // return false;
  bool ret = false;
  gradp.setZero();
  costp = 0;
  for (int i = 0; i < hPoly.cols(); ++i) { // hPloy就是一个多面体，不同列表示不同的面
    Eigen::Vector3d norm_vec = hPoly.col(i).head<3>(); //头3个数是法线，后3个数是原点
    double pen = norm_vec.dot(p - hPoly.col(i).tail<3>() + clearance_d_ * norm_vec); // = dp和发现做点积+ clearance*法线点积自己
    if (pen > 0) {
      double pen2 = pen * pen;
      gradp += rhoP_ * 3 * pen2 * norm_vec;
      costp += rhoP_ * pen2 * pen; // 3次方约束
      ret = true;
    }
  }
  return ret;
}

bool TrajOpt_perching::grad_cost_v(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  double vpen = v.squaredNorm() - vmax_ * vmax_;
  if (vpen > 0) {
    double grad = 0;
    costv = smoothedL1(vpen, grad); // 这里的grad是二范数之差，所以对v求梯度会有一个2
    gradv = rhoV_ * grad * 2 * v; // 不是6，所以G没有三次方
    costv *= rhoV_; // 乘上权重
    return true;
  }
  return false;
}


// 推力限制，表现为加速度的限制，加速度带来的惩罚项
bool TrajOpt_perching::grad_cost_thrust(const Eigen::Vector3d& a,
                               Eigen::Vector3d& grada,
                               double& costa) {
  bool ret = false;
  grada.setZero();
  costa = 0;
  Eigen::Vector3d thrust_f = a - g_;
  double max_pen = thrust_f.squaredNorm() - thrust_max_ * thrust_max_;
  if (max_pen > 0) {
    double grad = 0;
    costa = rhoThrust_ * smoothedL1(max_pen, grad);
    grada = rhoThrust_ * 2 * grad * thrust_f; // 转化成对加速度的梯度，链式法则
    ret = true;
  }

  double min_pen = thrust_min_ * thrust_min_ - thrust_f.squaredNorm();
  if (min_pen > 0) {
    double grad = 0;
    costa = rhoThrust_ * smoothedL1(min_pen, grad);
    grada = -rhoThrust_ * 2 * grad * thrust_f;
    ret = true;
  }

  return ret;
}

// using hopf fibration:
// [a,b,c] = thrust.normalized()
// \omega_1 = sin(\phi) \dot{a] - cos(\phi) \dot{b} - (a sin(\phi) - b cos(\phi)) (\dot{c}/(1+c))
// \omega_2 = cos(\phi) \dot{a] - sin(\phi) \dot{b} - (a cos(\phi) - b sin(\phi)) (\dot{c}/(1+c))
// \omega_3 = (b \dot{a} - a \dot(b)) / (1+c)
// || \omega_12 ||^2 = \omega_1^2 + \omega_2^2 = \dot{a}^2 + \dot{b}^2 + \dot{c}^2


// 转速的梯度
bool TrajOpt_perching::grad_cost_omega(const Eigen::Vector3d& a,
                              const Eigen::Vector3d& j,
                              Eigen::Vector3d& grada,
                              Eigen::Vector3d& gradj,
                              double& cost) {
  Eigen::Vector3d thrust_f = a - g_;
  Eigen::Vector3d zb_dot = f_DN(thrust_f) * j; // zb轴变化率 = f_DN（tau） * p^(3)(t)
  double omega_12_sq = zb_dot.squaredNorm(); // omega1^2 + omega2^2
  double pen = omega_12_sq - omega_max_ * omega_max_;
  if (pen > 0) {
    double grad = 0;
    cost = smoothedL1(pen, grad);

    Eigen::Vector3d grad_zb_dot = 2 * zb_dot; // 分成三个维度，   2 zb_dot = 2 * f_DN * j
    // std::cout << "grad_zb_dot: " << grad_zb_dot.transpose() << std::endl;
    gradj = f_DN(thrust_f).transpose() * grad_zb_dot;  //  这里就是 f_DN转置 * 2 * f_DN * j 符合二次型的求导法则
    grada = f_D2N(thrust_f, j).transpose() * grad_zb_dot;  // 这个就知就行

    cost *= rhoOmega_;
    grad *= rhoOmega_;
    grada *= grad;
    gradj *= grad;  // 总的来说 gradj = rhoOMega_ *  grad  *  f_DN(thrust_f).transpose() * grad_zb_dot

    return true;
  }
  return false;
}

// 没有对yaw轴转速进行惩罚
bool TrajOpt_perching::grad_cost_omega_yaw(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& j,
                                  Eigen::Vector3d& grada,
                                  Eigen::Vector3d& gradj,
                                  double& cost) {
  // TODO
  return false;
}


// 对位置高度惩罚
bool TrajOpt_perching::grad_cost_floor(const Eigen::Vector3d& p,
                              Eigen::Vector3d& gradp,
                              double& costp) {
  static double z_floor = 0.4;
  double pen = z_floor - p.z(); // 惩罚项
  if (pen > 0) { // 需要惩罚
    double grad = 0; 

    // smoothedL1对x的导数在grad里了
    costp = smoothedL1(pen, grad); // 该惩罚项的导数为grad // 惩罚就是p_floor -p_z
    costp *= rhoP_;
    gradp.setZero();
    gradp.z() = -rhoP_ * grad; // 往负梯度方向走 惩罚对p_z求导就是-1
    return true;
  } else {
    return false;
  }
}


// 算出来之后grad_car_p不用管
bool TrajOpt_perching::grad_cost_landing_upside(const Eigen::Vector3d& pos,
                                const Eigen::Vector3d& car_p,
                                Eigen::Vector3d& gradp,
                                Eigen::Vector3d& grad_car_p, 
                                double& cost)
{

    // 距离约束
    double ld = landing_ld; // 锥尖到平台的距离
    Eigen::Vector3d zd = tail_q_v_;
    Eigen::Vector3d cone_tip = car_p - zd * ld;

    // Note : visualization
    // visPtr_->visualize_a_ball(cone_tip,0.1,"cone_tip",vis_utils::yellow,1);
    // visPtr_->visualize_a_ball(cone_tip,landing_circle_r,"target_circle_area",vis_utils::blue,0.5);
    // Eigen::Vector3d cone_line;
    // cone_line.x() = - landing_circle_r * sin(theta_cone_); cone_line.z() = landing_circle_r * cos(theta_cone_);cone_line.y()=0;
    // visPtr_->visualize_arrow(cone_tip,cone_tip+cone_line,"cone_line",vis_utils::red);


    double dist_sqr = (pos - cone_tip).squaredNorm();
    double safe_r = landing_circle_r ; // 作用距离
    double safe_r_sqr = safe_r * safe_r;
    double pen_dist = safe_r_sqr - dist_sqr; // d^2 - 距离^2
    // pen_dist /= safe_r_sqr;  
    double grad_dist = 0;
    double cost_dist = smoothed01(pen_dist, grad_dist); // L_epsilon,L_epsion(x)对x的导数在grad上
    if (cost_dist == 0) { 
      return false; // 也就是太远了不起作用
    }
    Eigen::Vector3d gradp_dist = grad_dist * 2 * (cone_tip - pos); // 
    Eigen::Vector3d grad_carp_dist = -gradp_dist;

    double theta_cone=theta_cone_; // 锥角
    // ROS_INFO("[ landing_cost] theta_cone = %f",theta_cone);
    Eigen::Vector3d dp = pos - cone_tip;
    double inner_product = dp.dot(zd);
    double norm_dp = dp.norm();
    double norm_zd = zd.norm();

    double pen_land = cos(theta_cone) - inner_product / norm_dp / norm_zd;
    


    if(pen_land <0)
    {
      // ROS_INFO("[ landing_cost ] pan_land < 0");
      return false;  // 满足要求
    }

    // ROS_INFO("[ landing cost ] pos :");
    // std::cout<<pos<<std::endl;
    // ROS_INFO(" pen_land is %f",pen_land);


    double grad = 0;
    double cost_land;
    cost_land = smoothedL1(pen_land,grad);
    
    Eigen::Vector3d gradp_land,grad_carp_land;
    gradp_land = grad * -(norm_dp * zd - inner_product / norm_dp * dp) / norm_dp /norm_dp / norm_zd; // 主要求这个
    grad_carp_land = -gradp_land;

    // land项写完了
    gradp_land *= cost_dist; 
    grad_carp_land *= cost_dist;

    // dist项写完了
    gradp_dist *= cost_land;
    grad_carp_dist *= cost_land;

    cost = cost_land * cost_dist;
    cost *= rhoLanding;

    // ROS_INFO("[ landing_cost ]pos.x=%f cost = %f",pos.x(),cost);
    // visPtr_->visualize_a_ball(pos,0.01,"pen_keypoint",vis_utils::green,1);


    gradp = gradp_land + gradp_dist;
    grad_car_p = grad_carp_land + grad_carp_dist;

    gradp*=rhoLanding;
    grad_car_p *= rhoLanding;
    return true;
                                
}



// plate: \Epsilon = \left{ x = RBu + c | \norm(u) \leq r \right}
// x \in R_{3\times1}, u \in R_{2\times1}, B \in R_{3\times2}
// c: center of the plate; p: center of the drone bottom
//  c = p - l * z_b
// plane: a^T x \leq b
//        a^T(RBu + c) \leq b
//        a^T(RBu + p - l * z_b) \leq b
//        u^T(B^T R^T a) + a^Tp - a^T*l*z_b - b \leq 0
//        r \norm(B^T R^T a) + a^Tp - a^T*l*z_b - b \leq 0
// B^T R^T = [1-2y^2,    2xy, -2yw;
//               2xy, 1-2x^2,  2xw]
// B^T R^T = [1-a^2/(1+c),   -ab/(1+c), -a;
//              -ab/(1+c), 1-b^2/(1+c), -b]
bool TrajOpt_perching::grad_cost_perching_collision(const Eigen::Vector3d& pos,
                                           const Eigen::Vector3d& acc,
                                           const Eigen::Vector3d& car_p,
                                           Eigen::Vector3d& gradp,
                                           Eigen::Vector3d& grada,
                                           Eigen::Vector3d& grad_car_p,
                                           double& cost) {
  static double eps = 1e-6;

  double dist_sqr = (pos - car_p).squaredNorm();
  double safe_r = platform_r_ + robot_r_; // 论文里的d
  double safe_r_sqr = safe_r * safe_r;
  double pen_dist = safe_r_sqr - dist_sqr; // d^2 - 距离^2
  pen_dist /= safe_r_sqr;  
  double grad_dist = 0;
  double var01 = smoothed01(pen_dist, grad_dist); // L_epsilon,L_epsion(x)对x的导数在grad上
  if (var01 == 0) { 
    return false; // 也就是太远了不起作用
  }
  Eigen::Vector3d gradp_dist = grad_dist * 2 * (car_p - pos); // 
  Eigen::Vector3d grad_carp_dist = -gradp_dist;

  Eigen::Vector3d a_i = -tail_q_v_;
  double b_i = a_i.dot(car_p);

  Eigen::Vector3d thrust_f = acc - g_;
  Eigen::Vector3d zb = f_N(thrust_f); // 就是归一化

  Eigen::MatrixXd BTRT(2, 3);
  double a = zb.x();
  double b = zb.y();
  double c = zb.z();

  double c_1 = 1.0 / (1 + c);

  BTRT(0, 0) = 1 - a * a * c_1;
  BTRT(0, 1) = -a * b * c_1;
  BTRT(0, 2) = -a;
  BTRT(1, 0) = -a * b * c_1;
  BTRT(1, 1) = 1 - b * b * c_1;
  BTRT(1, 2) = -b;

  Eigen::Vector2d v2 = BTRT * a_i;
  double v2_norm = sqrt(v2.squaredNorm() + eps);
  double pen = a_i.dot(pos) - (robot_l_ - 0.005) * a_i.dot(zb) - b_i + robot_r_ * v2_norm;

  if (pen > 0) {
    double grad = 0;
    cost = smoothedL1(pen, grad);
    // gradients: pos, car_p, v2
    gradp = a_i; // F1对飞机位置的梯度
    grad_car_p = -a_i;  // F1对车位置的梯度，也就是b对车位置的梯度
    Eigen::Vector2d grad_v2 = robot_r_ * v2 / v2_norm;  // BTRT范数对BTRT的梯度

    // 下面求BTRT对zb的梯度
    Eigen::MatrixXd pM_pa(2, 3), pM_pb(2, 3), pM_pc(2, 3); 
    double c2_1 = c_1 * c_1;

    pM_pa(0, 0) = -2 * a * c_1;
    pM_pa(0, 1) = -b * c_1;
    pM_pa(0, 2) = -1;
    pM_pa(1, 0) = -b * c_1;
    pM_pa(1, 1) = 0;
    pM_pa(1, 2) = 0;

    pM_pb(0, 0) = 0;
    pM_pb(0, 1) = -a * c_1;
    pM_pb(0, 2) = 0;
    pM_pb(1, 0) = -a * c_1;
    pM_pb(1, 1) = -2 * b * c_1;
    pM_pb(1, 2) = -1;

    pM_pc(0, 0) = a * a * c2_1;
    pM_pc(0, 1) = a * b * c2_1;
    pM_pc(0, 2) = 0;
    pM_pc(1, 0) = a * b * c2_1;
    pM_pc(1, 1) = b * b * c2_1;
    pM_pc(1, 2) = 0;

    Eigen::MatrixXd pv2_pzb(2, 3);
    pv2_pzb.col(0) = pM_pa * a_i;
    pv2_pzb.col(1) = pM_pb * a_i;
    pv2_pzb.col(2) = pM_pc * a_i;

    Eigen::Vector3d grad_zb = pv2_pzb.transpose() * grad_v2 - robot_l_ * a_i; // 和起来得到F1对zb的梯度

    grada = f_DN(thrust_f).transpose() * grad_zb; // 从zb的梯度到加速度a的梯度

    grad *= var01; // F1梯度项乘上连时法则 
    gradp_dist *= cost; // F2的飞机位置梯度项乘上连时法则
    grad_carp_dist *= cost; // F2的车位置梯度项乘上连时法则
    cost *= var01; // 两项cost相乘 
    gradp = grad * gradp + gradp_dist; // 对位置的梯度和起来
    grada *= grad;
    grad_car_p = grad * grad_car_p + grad_carp_dist;

    cost *= rhoPerchingCollision_;
    gradp *= rhoPerchingCollision_;
    grada *= rhoPerchingCollision_;
    grad_car_p *= rhoPerchingCollision_;

    // std::cout << "var01: " << var01 << std::endl;

    return true;
  }
  return false;
}



// 检查是否发生碰撞
bool TrajOpt_perching::check_collilsion(const Eigen::Vector3d& pos,
                               const Eigen::Vector3d& acc,
                               const Eigen::Vector3d& car_p) {
  // 离太远了没有碰撞
  if ((pos - car_p).norm() > platform_r_) {
    return false;
  }
  static double eps = 1e-6;

  Eigen::Vector3d a_i = -tail_q_v_; //，指向降落平台之外的方向
  double b_i = a_i.dot(car_p);

  Eigen::Vector3d thrust_f = acc - g_; // 由加速度得到姿态
  Eigen::Vector3d zb = f_N(thrust_f);

  Eigen::MatrixXd BTRT(2, 3);
  double a = zb.x();
  double b = zb.y();
  double c = zb.z();

  double c_1 = 1.0 / (1 + c);

  BTRT(0, 0) = 1 - a * a * c_1;
  BTRT(0, 1) = -a * b * c_1;
  BTRT(0, 2) = -a;
  BTRT(1, 0) = -a * b * c_1;
  BTRT(1, 1) = 1 - b * b * c_1;
  BTRT(1, 2) = -b; // 取出旋转矩阵前两列再转至

  Eigen::Vector2d v2 = BTRT * a_i;
  double v2_norm = sqrt(v2.squaredNorm() + eps);
  double pen = a_i.dot(pos) - (robot_l_ - 0.005) * a_i.dot(zb) - b_i + robot_r_ * v2_norm;

  return pen > 0;
}

// 可行性约束
bool TrajOpt_perching::feasibleCheck(Trajectory_S4& traj) {
  double dt = 0.01;
  for (double t = 0; t < traj.getTotalDuration(); t += dt) {
    Eigen::Vector3d p = traj.getPos(t);
    Eigen::Vector3d a = traj.getAcc(t);
    Eigen::Vector3d j = traj.getJer(t);
    Eigen::Vector3d thrust = a - g_;
    Eigen::Vector3d zb_dot = f_DN(thrust) * j;
    double omega12 = zb_dot.norm();
    if (omega12 > omega_max_ + 0.2) {
      return false;
    }
    if (p.z() < 0.1) {
      return false;
    }
  }
  return true;
}

}  // namespace traj_opt_perching