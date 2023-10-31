#ifndef MPC_H_
#define MPC_H_

#include <qpOASES.hpp>
#include <Eigen/Eigen>
#include <vector>
#include "ros/ros.h"

class MPC_controller
{
public:
    /****
     * 使用线性误差模型，用qpOASES求解
     * @param X_c 当前状态
     * @param X_r 参考状态
     * @param U_r 参考收入
     * @param N   预测步数
     * @return    预测输出序列
     */
    Eigen::MatrixXd MPC_Solve_qp(Eigen::Vector3d X_k,std::vector<Eigen::Vector3d >X_r,std::vector<Eigen::Vector2d >U_r,const int N);
    void MPC_init(ros::NodeHandle &nh);
private:
    double v_max;
    double v_min;
    double w_max;
    double w_min;
};

#endif