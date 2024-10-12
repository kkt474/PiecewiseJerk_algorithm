#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include "PiecewiseJerkSpeed_nonlinear_Optimizer.hpp"
#include "PiecewiseJerkPathProblem.hpp"

int main()
{
    // piecewise_path_optimizer
    // 路径规划  // 处理静态障碍物

    int num_of_knots = 200;
    std::array<double, 3> x_init{0, 0, 0};
    std::array<double, 3> end_state{0, 0, 0};
    float delta_s = 0.5;
    // 实例化路径规划对象，同时初始化父类
    // 子类只是重写了CalculateKernel和CalculateOffset函数
    PiecewiseJerkPathProblem path_optimizer(num_of_knots, delta_s, x_init);
    path_optimizer.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);
    // PiecewiseJerkProblem.hpp
    path_optimizer.set_weight_x(100); // weight_x_ = 100
    path_optimizer.set_weight_dx(1);  // weight_ddx_ = 1
    path_optimizer.set_weight_ddx(10);  // weight_ddx_ = 10
    path_optimizer.set_weight_dddx(10);  // weight_dddx_ = 10

    std::vector<std::pair<double, double>> lat_boundaries;
    std::vector<std::pair<double, double>> ddl_bounds;
    for (int i = 0; i < num_of_knots; i++)
    {
        std::pair<double, double> tmp_s = std::make_pair(-2, 2);
        std::pair<double, double> tmp_ds = std::make_pair(-10, 10);
        lat_boundaries.push_back(tmp_s); // 横向，L方向的边界
        ddl_bounds.push_back(tmp_ds);    // 横向, ddl的边界
    }

    // for (int i = 0; i < num_of_knots; i++){
    //     std::cout << "lat_size: " << lat_boundaries.size() << std::endl;
    //     std::cout << "lat_boundaries每个点对应的边界: " << lat_boundaries[i].first << " "
    //               << lat_boundaries[i].second << std::endl;
    // }
// 制造一些边界
    // for (int k = 0; k < 20; k++)
    // {
    //     lat_boundaries.at(num_of_knots / 1.5 + k).second = -1;
    // }
    // for (int k = 0; k < 20; k++)
    // {
    //     lat_boundaries.at(num_of_knots / 5 + k).first = 1;
    // }
    // std::cout << __LINE__ << ":" << num_of_knots / 1.5 << std::endl;
    for (int k = 0; k < 20; k++)
    {
        lat_boundaries.at(num_of_knots / 1.5 + k).second = -1;
    }
    for (int k = 0; k < 20; k++)
    {
        lat_boundaries.at(num_of_knots / 5 + k).first = 1;
    }

    path_optimizer.set_x_bounds(lat_boundaries); // 横向l
    // 暂时设置的都是[-10,10]
    path_optimizer.set_dx_bounds(-10, 10);       // 横向dl的边界值
    path_optimizer.set_ddx_bounds(ddl_bounds);   //横向ddl

    // 求解
    int max_iter = 1000;
    // // PiecewiseJerkProblem.cpp
    bool success = path_optimizer.Optimize(max_iter);
    if (success == 1)
    {
        std::cout << "path_optimizer success " << std::endl;
    }
    auto x = path_optimizer.opt_x();
    auto dx = path_optimizer.opt_dx();
    auto ddx = path_optimizer.opt_ddx();
    std::cout << "x.size()   " << x.size() << std::endl;

    // 优化后的路径点
    // 横坐标t, 纵坐标s
    std::ofstream out;
    out.open("../plot/path_optimizer.csv");
    out << "s,l,l_boundaries_lower,l_boundaries_upper" << std::endl;
    for (int i = 0; i != x.size(); ++i)
    {
        out << i * delta_s << "," << x[i] << "," << lat_boundaries[i].first
            << "," << lat_boundaries[i].second << std::endl;
    }
    out.close();
}