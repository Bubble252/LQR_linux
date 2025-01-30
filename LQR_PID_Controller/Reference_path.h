#ifndef REFERENCE_PATH_H
#define REFERENCE_PATH_H

#include <vector>
#include <cmath>
#include <algorithm>

#define PI 3.14159265358979323846

using namespace std;

class ReferencePath
{
public:
    ReferencePath(); // 构造函数

    // 计算跟踪误差
    vector<double> calcTrackError(vector<double> robot_state);

//private:
    vector<vector<double>> ref_path; // 参考轨迹 [x, y, yaw, k]
    vector<double> ref_x; // 参考轨迹 x 坐标
    vector<double> ref_y; // 参考轨迹 y 坐标

    // 角度归一化到 [-PI, PI]
    double normalizeAngle(double angle);
};

#endif // REFERENCE_PATH_H

