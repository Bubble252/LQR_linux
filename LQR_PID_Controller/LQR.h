#ifndef LQR_H
#define LQR_H

#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class LQR {
public:
    explicit LQR(int n);  // 构造函数，显式声明防止隐式转换

    // 计算 Riccati 方程，返回矩阵 P
    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);

    // LQR 控制方法，返回计算出的控制量（如方向角 delta）
    double LQRControl(vector<double> robot_state, vector<vector<double>> ref_path, 
                      double s0, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);

private:
    int N;  // 迭代步数
    static constexpr double EPS = 1e-6;  // 迭代终止条件
};

#endif // LQR_H

