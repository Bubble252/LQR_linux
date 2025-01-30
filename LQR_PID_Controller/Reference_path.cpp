#include "Reference_path.h"

ReferencePath::ReferencePath()
{
    ref_path = vector<vector<double>>(1000, vector<double>(4));
    // 生成参考轨迹
    for (int i = 0; i < 1000; i++)
    {
        double time_factor =  0.002 * i; // 时间因子逐渐增大，幅度随时间增加
        ref_path[i][0] = 0.1 * i;
        ref_path[i][1] = (2+time_factor) * sin(ref_path[i][0] / 3.0);

        ref_x.push_back(ref_path[i][0]);
        ref_y.push_back(ref_path[i][1]);
    }
    double dx, dy, ddx, ddy;
    for (int i = 0; i < ref_path.size(); i++)
    {
        if (i == 0) {
            dx = ref_path[i + 1][0] - ref_path[i][0];
            dy = ref_path[i + 1][1] - ref_path[i][1];
            ddx = ref_path[2][0] + ref_path[0][0] - 2 * ref_path[1][0];
            ddy = ref_path[2][1] + ref_path[0][1] - 2 * ref_path[1][1];
        } else if (i == ref_path.size() - 1) {
            dx = ref_path[i][0] - ref_path[i- 1][0];
            dy = ref_path[i][1] - ref_path[i- 1][1];
            ddx = ref_path[i][0] + ref_path[i- 2][0] - 2 * ref_path[i - 1][0];
            ddy = ref_path[i][1] + ref_path[i - 2][1] - 2 * ref_path[i - 1][1];
        } else {
            dx = ref_path[i + 1][0] - ref_path[i][0];
            dy = ref_path[i + 1][1] - ref_path[i][1];
            ddx = ref_path[i + 1][0] + ref_path[i - 1][0] - 2 * ref_path[i][0];
            ddy = ref_path[i + 1][1] + ref_path[i - 1][1] - 2 * ref_path[i][1];
        }
        ref_path[i][2] = atan2(dy, dx);
        //计算曲率:设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
        ref_path[i][3] = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), 3.0 / 2); // k计算
    }
}
// 计算跟踪误差
vector<double> ReferencePath::calcTrackError(vector<double> robot_state)
{
    double x = robot_state[0], y = robot_state[1];
    vector<double> d_x(ref_path.size()), d_y(ref_path.size()), d(ref_path.size());
    for (int i = 0; i < ref_path.size(); i++)
    {
        d_x[i]=ref_path[i][0]-x;
        d_y[i]=ref_path[i][1]-y;
        d[i] = sqrt(d_x[i]*d_x[i]+d_y[i]*d_y[i]);
    }
    double min_index = min_element(d.begin(), d.end()) - d.begin();
    double yaw = ref_path[min_index][2];
    double k = ref_path[min_index][3];
    double angle = normalizeAngle(yaw - atan2(d_y[min_index], d_x[min_index]));
    double error = d[min_index];
    if (angle < 0) error *= -1;
    return {error, k, yaw, min_index};
}

// 角度归一化到 -PI到PI
double ReferencePath::normalizeAngle(double angle)
{
    while (angle > PI)
    {
        angle -= 2 * PI;
    }
    while (angle < -PI)
    {
        angle += 2 * PI;
    }
    return angle;
}

