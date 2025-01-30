#ifndef KINEMATIC_MODEL_H
#define KINEMATIC_MODEL_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class KinematicModel
{
public:
    // 构造函数
    KinematicModel(double x, double y, double psi, double v, double L, double dt);

    // 更新运动学状态
    void updateState(double a, double delta);

    // 获取当前状态
    vector<double> getState();

    // 获取状态空间模型的离散化矩阵 A 和 B
    vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw);

//private:
    double x, y, psi, v; // 位置 (x, y)，航向角 psi，速度 v
    double L;            // 车辆轴距
    double dt;           // 时间步长
};

#endif // KINEMATIC_MODEL_H

