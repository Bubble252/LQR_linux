#ifndef PID_CONTROL_H
#define PID_CONTROL_H

class PID_controller {
public:
    // 构造函数
    PID_controller(double Kp, double Ki, double Kd, double target,
                   double upper, double lower);

    // 设置目标值
    void setTarget(double target);

    // 设置PID参数
    void setK(double Kp, double Ki, double Kd);

    // 设置上下边界
    void setBound(double upper, double lower);

    // 计算输出（加速度）
    double calOutput(double state);

    // 重置PID控制器
    void reset();

    // 设置累计误差
    void setSumError(double sum_error);

private:
    double Kp, Ki, Kd;  // PID参数
    double target;       // 目标值
    double upper, lower; // 输出边界
    double error;        // 当前误差
    double pre_error;    // 上次误差
    double sum_error;    // 累计误差
};

#endif // PID_CONTROL_H

