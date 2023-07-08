#ifndef KALAMAN
#define KALAMAN
#include "../common.h"

using namespace Eigen;
using namespace cv;
#define MAX_TIME_BIAS 1000
class Kalman_Filter
{
public:
    Kalman_Filter()
    {
        Eigen::MatrixXd P_in = Eigen::MatrixXd(6, 6);
        P_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        P_ = P_in;

        // 过程噪声矩阵附初值
        Eigen::MatrixXd Q_in(6, 6);
        Q_in << 1., 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        Q_ = Q_in;

        // 测量矩阵附初值
        Eigen::MatrixXd H_in(3, 6);
        H_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
        H_ = H_in;

        // 测量噪声矩阵附初值
        Eigen::MatrixXd R_in(3, 3);
        R_in << 10.0, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 15.0;
        R_ = R_in;

        VectorXd last_of_time(6, 1);
        last_of_time << 0, 0, 0, 0, 0, 0;
        last_x_ = last_of_time;

        last_ = Point3f(0, 0, 0);
    }

    /**
     * @brief 卡尔曼滤波的预测过程
     *
     * @return
     */
    void predict()
    {
        x_ = F_ * last_x_;

        P_ = F_ * P_ * F_.transpose() + Q_;
    };

    /**
     * @brief 卡尔曼滤波的测量过程
     *
     */
    void measure(VectorXd z)
    {
        y = z - H_ * x_;                                   // 获取测量值与 预测测量值之间的差值
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; // 临时变量
        K_ = P_ * H_.transpose() * S.inverse();            // 获取卡尔曼增益
        x_ = x_ + (K_ * y);                                // 获得状态最优估计

        int size = x_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
        P_ = (I - K_ * H_) * P_; // 更新协方差矩阵，完成闭环控制
    };

    /**
     * @brief 卡尔曼主程序
     *
     */
    Vector3d KalmanMainRun(Vector3d now_x, Vector3d now_rad_v)
    {
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - ta);
        ta = t2;

        if (!flag_)
        {
            VectorXd x(6, 1);
            x << now_x[0], now_x[1], now_x[2], 0, 0, 0;
            last_x_ = x;
            cout << "kalaman init" << endl;
            flag_ = true;
            return now_x;
        }

        VectorXd x(6, 1);
        x << now_x[0], now_x[1], now_x[2], now_rad_v[0], now_rad_v[1], now_rad_v[2];
        x_ = x;

        if (is_change_direction_ = true)
        {
            Eigen::MatrixXd P_in = Eigen::MatrixXd(6, 6);
            P_in << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            P_ = P_in;
            flag_ = false;
        }

        Eigen::MatrixXd F(6, 6);
        F << 1.0, 0.0, 0.0, time_used.count(), 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, time_used.count(), 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, time_used.count(),
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        F_ = F;
        predict();

        // cout << "   " << x_[0] << "   " << x_[1] << "   " << x_[2] << endl;

        VectorXd z_(3, 1);
        z_ << now_x[0], now_x[1], now_x[2];

        measure(z_);

        cout << "1   " << x_[0] << " 2  " << x_[1] << "  3 " << x_[2] << endl;
        if (std::abs(x_[2] - last_x_[2]) > CV_PI / 24)
        {
            Vector3d last;
            last << last_x_[0], last_x_[1], last_x_[2];
            return last;
        }
        last_x_ = x_;
        cout << "kalaman filter finished" << endl;
        Vector3d x_now;
        x_now << x_[0], x_[1], x_[2];
        return x_now;
    };

    /**
     * @brief 获取当前时间的函数,单位是毫秒
     *
     */

private:
    VectorXd x_;       // 当前时刻的状态向量
    VectorXd last_x_;  // 上一次的状态向量
    MatrixXd F_;       // 状态转移矩阵
    MatrixXd B_;       // 输入矩阵
    MatrixXd P_;       // 协方差矩阵，主要数据是对角线上的数据，表示相关性
    MatrixXd K_;       // 卡尔慢增益，过程噪声与测量噪声的比值
    MatrixXd Q_;       // 过程噪声矩阵
    MatrixXd R_;       // 测量噪声矩阵
    MatrixXd H_;       // 测量矩阵
    Eigen::VectorXd y; // 残差
public:
    bool is_change_direction_ = false; // 是否变向

    Point3f last_;

    bool flag_ = false;
    std::chrono::steady_clock::time_point ta;
};

#endif
