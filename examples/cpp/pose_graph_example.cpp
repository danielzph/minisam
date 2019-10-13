/**
 * A simple 2D pose-graph SLAM
 * The robot moves from x1 to x5, with odometry information between each pair.
 * the robot moves 5 each step, and makes 90 deg right turns at x3 - x5
 * At x5, there is a *loop closure* between x2 is avaible
 * The graph strcuture is shown:
 *
 *  p-x1 - x2 - x3
 *         |    |
 *         x5 - x4
 */
   // 选择需要的.h头文件
#include <minisam/core/Factor.h>
#include <minisam/core/FactorGraph.h>
#include <minisam/core/LossFunction.h>
#include <minisam/core/Variables.h>
#include <minisam/geometry/Sophus.h>  // include when use Sophus types in optimization
#include <minisam/nonlinear/LevenbergMarquardtOptimizer.h>  //这里我们使用LM优化方法
#include <minisam/nonlinear/MarginalCovariance.h>
#include <minisam/slam/BetweenFactor.h>
#include <minisam/slam/PriorFactor.h>

#include <iostream>

using namespace std;
using namespace minisam;

/* ******************************* example ********************************** */

int main() {
  //  定义FactorGraph类的对象 因子图容器
  FactorGraph graph;

  // 在第一位置上加入一个先验，并将其设置为原点
  // 先验需要在世界坐标系下去修改/校正运行轨迹
  // 先验因子包括一个均值和损失函数(协方差矩阵)

  const std::shared_ptr<LossFunction> priorLoss =
      DiagonalLoss::Sigmas(Eigen::Vector3d(1.0, 1.0, 0.1));   //先验因子的损失函数
  graph.add(PriorFactor<Sophus::SE2d>(
      key('x', 1), Sophus::SE2d(0, Eigen::Vector2d(0, 0)), priorLoss));   //添加先验因子

  // 里程计测量的误差函数
  const std::shared_ptr<LossFunction> odomLoss =
      DiagonalLoss::Sigmas(Eigen::Vector3d(0.5, 0.5, 0.1));

  // 在连续位姿之间添加Between Factor
  // 每次前进5个单位,运动到x3-x5时会有向右90°的转动
  graph.add(BetweenFactor<Sophus::SE2d>(
      key('x', 1), key('x', 2), Sophus::SE2d(0.0, Eigen::Vector2d(5, 0)),
      odomLoss));
  graph.add(BetweenFactor<Sophus::SE2d>(
      key('x', 2), key('x', 3), Sophus::SE2d(-1.57, Eigen::Vector2d(5, 0)),
      odomLoss));
  graph.add(BetweenFactor<Sophus::SE2d>(
      key('x', 3), key('x', 4), Sophus::SE2d(-1.57, Eigen::Vector2d(5, 0)),
      odomLoss));
  graph.add(BetweenFactor<Sophus::SE2d>(
      key('x', 4), key('x', 5), Sophus::SE2d(-1.57, Eigen::Vector2d(5, 0)),
      odomLoss));

  // 闭环检测的误差函数
  const std::shared_ptr<LossFunction> loopLoss =
      DiagonalLoss::Sigmas(Eigen::Vector3d(0.5, 0.5, 0.1));

  // 加入闭环约束
  graph.add(BetweenFactor<Sophus::SE2d>(
      key('x', 5), key('x', 2), Sophus::SE2d(-1.57, Eigen::Vector2d(5, 0)),
      loopLoss));

  graph.print();
  cout << endl;

  // 加入变量初始值
  // 加入随机噪声(自定义？)  from ground truth values
  Variables initials;

  initials.add(key('x', 1), Sophus::SE2d(0.2, Eigen::Vector2d(0.2, -0.3)));
  initials.add(key('x', 2), Sophus::SE2d(-0.1, Eigen::Vector2d(5.1, 0.3)));
  initials.add(key('x', 3),
               Sophus::SE2d(-1.57 - 0.2, Eigen::Vector2d(9.9, -0.1)));
  initials.add(key('x', 4),
               Sophus::SE2d(-3.14 + 0.1, Eigen::Vector2d(10.2, -5.0)));
  initials.add(key('x', 5),
               Sophus::SE2d(1.57 - 0.1, Eigen::Vector2d(5.1, -5.1)));

  initials.print();
  cout << endl;

  // 调用 LM 方法对初值进行优化
  LevenbergMarquardtOptimizerParams opt_param;
  opt_param.verbosity_level = NonlinearOptimizerVerbosityLevel::ITERATION;
  LevenbergMarquardtOptimizer opt(opt_param);

  Variables results;

  auto status = opt.optimize(graph, initials, results);
  if (status != NonlinearOptimizationStatus::SUCCESS) {
    cout << "optimization error" << endl;
  }

  results.print();
  cout << endl;

  // 计算所有位姿的边际协方差(误差)
  MarginalCovarianceSolver mcov_solver;

  auto cstatus = mcov_solver.initialize(graph, results);
  if (cstatus != MarginalCovarianceSolverStatus::SUCCESS) {
    cout << "maginal covariance error" << endl;
  }

  Eigen::Matrix3d cov1 = mcov_solver.marginalCovariance(key('x', 1));
  Eigen::Matrix3d cov2 = mcov_solver.marginalCovariance(key('x', 2));
  Eigen::Matrix3d cov3 = mcov_solver.marginalCovariance(key('x', 3));
  Eigen::Matrix3d cov4 = mcov_solver.marginalCovariance(key('x', 4));
  Eigen::Matrix3d cov5 = mcov_solver.marginalCovariance(key('x', 5));

  cout << "cov pose 1:" << cov1 << endl;
  cout << "cov pose 2:" << cov2 << endl;
  cout << "cov pose 3:" << cov3 << endl;
  cout << "cov pose 4:" << cov4 << endl;
  cout << "cov pose 5:" << cov5 << endl;

  return 0;
}
