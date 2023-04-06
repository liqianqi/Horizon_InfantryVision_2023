#include "../../include/buff/predictor.hpp"
#define DRAW_PREDICT
bool Predictor::predictLocation(BUFF &buff, int timestamp,GimbalPose &pose,int mode)
{

    /*
     * 1. 得到buff先转化为三维信息
     * 2. 手眼标定得到云台系位置
     * 3. 同时获得目标的时间，应该为每一个图像赋予一个时间辍，作为整体封装
     */
    is_params_confirmed_ = true;
    if(!is_params_confirmed_)
    {
        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary; // 优化信息
        double params_fitting[4] = {1.000, 1.900, CV_PI/3, 1.090};

        for (auto target_info : history_info)
        {
            problem.AddResidualBlock(   // 向问题中添加误差项
                                        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(
                    new CURVE_FITTING_COST((float)(target_info.timestamp) / 1e3,
                                            target_info.speed)),
                new ceres::CauchyLoss(0.5),
                params_fitting          // 待估计参数
            );
        }
        // 设置上下限
        // FIXME: 参数需根据场上大符实际调整
        problem.SetParameterLowerBound(params_fitting, 0, 0.7);
        problem.SetParameterUpperBound(params_fitting, 0, 1.2);
        problem.SetParameterLowerBound(params_fitting, 1, 1.6);
        problem.SetParameterUpperBound(params_fitting, 1, 2.2);
        problem.SetParameterLowerBound(params_fitting, 2, -CV_PI);
        problem.SetParameterUpperBound(params_fitting, 2, CV_PI);
        problem.SetParameterLowerBound(params_fitting, 3, 0.5);
        problem.SetParameterUpperBound(params_fitting, 3, 2.5);

        ceres::Solve(options, &problem, &summary);

        // 对参数进行评估，采用RMSE
        double cls = evalRMSE(params_fitting);
        if(cls < max_rmse)
        {
            params_[0] = params_fitting[0];
            params_[1] = params_fitting[1];
            params_[2] = params_fitting[2];
            params_[3] = params_fitting[3];

            is_params_confirmed_ = true;
        }

    }
    else    // 拟合成功后，直接积分得到位置
    {
        
    }
#ifdef DRAW_PREDICT
    if (target_.timestamp % 10 == 0)
    {
        std::vector<double> plt_time;
        std::vector<double> plt_speed;
        std::vector<double> plt_fitted;
        for (auto target_info : history_info)
        {
            auto t = (float)(target_info.timestamp) / 1e3;
            plt_time.push_back(t);
            plt_speed.push_back(target_info.speed);
            plt_fitted.push_back(params_[0] * sin (params_[1] * t + params_[2]) + params_[3]);
        }
        plt::clf();
        plt::plot(plt_time, plt_speed,"bx");
        plt::plot(plt_time, plt_fitted,"r-");
        plt::pause(0.001);

    }

#endif
    

}

double Predictor::evalRMSE(double params[4])
{
    double rmse_sum = 0;
    double rmse = 0;
    for (auto target_info : history_info)
    {
        auto t = (float)(target_info.timestamp) / 1e3;
        auto pred = params[0] * sin (params[1] * t + params[2]) + params[3];
        auto measure = target_info.speed;
        rmse_sum+=pow((pred - measure),2);
    }
    rmse = sqrt(rmse_sum / history_info.size());
    return rmse;
}
