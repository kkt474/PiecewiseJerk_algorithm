#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include "PiecewiseJerkSpeed_nonlinear_Optimizer.hpp"
#include "PiecewiseJerkPathProblem.hpp"

int main()
{
    DiscretizedPath path;
    std::vector<double> cur_list{4.83718e-07, -0.00370725, -0.00741507, -0.0111229, -0.0139364, -0.0167499, -0.0195634, -0.0214857, -0.0234081,
                                 -0.0253305, -0.0265169, -0.0277034, -0.0288899, -0.0294724, -0.030055, -0.0306376, -0.0307238, -0.03081,
                                 -0.0308962, -0.0305733, -0.0302505, -0.0299276, -0.0292663, -0.0286051, -0.0279439, -0.0270011, -0.0260584,
                                 -0.0251157, -0.023937, -0.0227583, -0.0215796, -0.020201, -0.0188223, -0.0174437, -0.015893, -0.0143424,
                                 -0.0127918, -0.0110905, -0.00938913, -0.00768779, -0.00581911, -0.00395044, -0.00208178, -3.97346e-05,
                                 0.0020023, 0.00404434, 0.00621845, 0.00839256, 0.0105667, 0.0128369, 0.0151072, 0.0173775, 0.019712, 0.0220464,
                                 0.0243809, 0.0267502, 0.0291195, 0.0314888, 0.0338651, 0.0362413, 0.0386176, 0.0409732, 0.0433288, 0.0456844,
                                 0.047991, 0.0502975, 0.0526041, 0.0548316, 0.0570591, 0.0592865, 0.0614022, 0.0635178, 0.0656334, 0.0676012,
                                 0.069569, 0.0715368, 0.0733172, 0.0750977, 0.0768781, 0.0784278, 0.0799775, 0.0815272, 0.0827969, 0.0840667,
                                 0.0853268, 0.0862503, 0.0871608, 0.0880583, 0.0886594, 0.0892481, 0.0898241, 0.0901828, 0.0905379, 0.0908893,
                                 0.0910726, 0.0912538, 0.0914328, 0.0914769, 0.0915203, 0.0915632, 0.0915133, 0.0914643, 0.0914161, 0.0913148,
                                 0.0912133, 0.0911112, 0.0909452, 0.0907783, 0.0906079, 0.0901936, 0.0897459, 0.0892622, 0.0882592, 0.0872562,
                                 0.0862736, 0.0849132, 0.0835357, 0.0821401, 0.0802109, 0.0782751, 0.0763393, 0.0735463, 0.0707534, 0.0679604,
                                 0.0638479, 0.0597353, 0.0556227, 0.0510547, 0.0464867, 0.0419187, 0.0376932, 0.0334678, 0.0292424, 0.0255711,
                                 0.0218998, 0.0182285, 0.0150408, 0.0118531, 0.00866547, 0.0059406, 0.00321577, 0.000491031, -0.00172144,
                                 -0.00393384, -0.00614618, -0.00792725, -0.00970828, -0.0114893, -0.0129159, -0.0143426, -0.0157692, -0.0169041,
                                 -0.0180391, -0.019174, -0.0200682, -0.0209625, -0.0218567, -0.0225519, -0.023247, -0.0239422, -0.0244723,
                                 -0.0250025, -0.0255326, -0.0259257, -0.0263189, -0.026712, -0.0269913, -0.0272707, -0.0275501, -0.0277351,
                                 -0.0279202, -0.0281053, -0.0282126, -0.02832, -0.0284274, -0.028471, -0.0285148, -0.0285585, -0.0285506,
                                 -0.0285427, -0.0285349, -0.0284858, -0.0284367, -0.0283877, -0.0283065, -0.0282254, -0.0281443, -0.0280394,
                                 -0.0279344, -0.0278294, -0.0277082, -0.027587, -0.0274658, -0.0273357, -0.0272056, -0.0270755, -0.0269438,
                                 -0.0268121, -0.0266803, -0.0265544, -0.0264284, -0.0263024, -0.0262629};
    std::vector<double> s_list{
        0, 0.3051, 0.609883, 0.914307, 1.21821, 1.52155, 1.82438, 2.12681, 2.42896, 2.73086, 3.03252, 3.33397, 3.63524,
        3.93639, 4.23735, 4.53815, 4.83882, 5.13943, 5.44001, 5.74058, 6.04113, 6.34158, 6.6419, 6.94204, 7.24214,
        7.54223, 7.84229, 8.14221, 8.44184, 8.74113, 9.03998, 9.33825, 9.63586, 9.93271, 10.2288, 10.524, 10.8183,
        11.1119, 11.4051, 11.6981, 11.991, 12.2857, 12.5825, 12.8814, 13.1811, 13.4806, 13.78, 14.0793, 14.3789,
        14.6788, 14.9789, 15.2801, 15.5823, 15.8855, 16.1885, 16.4907, 16.792, 17.0929, 17.3941, 17.6957, 17.9978,
        18.3011, 18.6057, 18.9116, 19.2184, 19.526, 19.8344, 20.1438, 20.4546, 20.7667, 21.08, 21.3947, 21.7109,
        22.0282, 22.346, 22.664, 22.9821, 23.3003, 23.6188, 23.9374, 24.256, 24.5744, 24.8923, 25.2096, 25.524, 25.8335,
        26.1378, 26.4371, 26.7299, 27.0157, 27.2945, 27.5665, 27.8345, 28.0986, 28.3586, 28.6144, 28.866, 29.1133,
        29.3564, 29.5951, 29.829, 30.058, 30.2823, 30.5017, 30.7209, 30.9411, 31.1623, 31.3843, 31.6114, 31.8575,
        32.1244, 32.4137, 32.71, 33.006, 33.2958, 33.5796, 33.8673, 34.1589, 34.4545, 34.7513, 35.0482, 35.3452,
        35.6425, 35.9399, 36.2376, 36.5354, 36.8332, 37.1311, 37.4292, 37.7274, 38.026, 38.3249, 38.624, 38.9234,
        39.223, 39.5227, 39.8227, 40.1229, 40.4232, 40.7237, 41.0242, 41.3248, 41.6254, 41.926, 42.2265, 42.5269,
        42.8272, 43.1273, 43.4271, 43.7267, 44.026, 44.325, 44.6237, 44.922, 45.2199, 45.5174, 45.8146, 46.1114,
        46.4079, 46.704, 46.9998, 47.2952, 47.5904, 47.8854, 48.1802, 48.4749, 48.7694, 49.0638, 49.3582, 49.6528,
        49.9478, 50.2431, 50.5389, 50.8351, 51.1319, 51.4291, 51.7266, 52.0244, 52.3225, 52.6208, 52.9196, 53.2186,
        53.518, 53.8177, 54.1179, 54.4182, 54.7188, 55.0196, 55.3207, 55.6222, 55.924, 56.2263, 56.5289, 56.8317,
        57.1347, 57.4379, 57.7412, 58.0446, 58.3481, 58.6516, 58.9549, 59.2581, 59.5614};
    for (int i = 0; i != cur_list.size(); ++i)
    {
        PathPoint p;
        p.kappa_ = cur_list[i];
        p.s_ = s_list[i];
        path.push_back(p);
    }

    double dt = 0.2;
    double reference_crusie_velocity = 5;
    // 上下边界
    // 这里优化速度 s_t, 需要先使用DP_speed_planner,计算在有动态障碍物的情况下的t对应的s_bounds
    // index这里需要乘以dt,表示时间坐标  // double t = i * dt;
    std::vector<std::pair<double, double>> s_bounds;
    for (int i = 0; i != 41; ++i)
    {
        s_bounds.emplace_back(std::make_pair<double, double>(0, 60));
    }
    for (int i = 0; i != 4; ++i)
    {
        s_bounds[7 + i].first = 10;
    }
    for (int i = 0; i != 5; ++i)
    {
        s_bounds[36 + i].second = 50;
    }

    // 参考路径点 // lane_center_line  // 道路的中心线
    std::vector<double> ref_s_list;
    for (int i = 0; i != 41; ++i)
    {
        ref_s_list.emplace_back(reference_crusie_velocity * i * dt);
    }

    // 速度的上边界
    SpeedLimit speed_limit;
    double ds = 0.5, s = 0;
    while (s < 80)
    {
        if (s >= 30)
            speed_limit.AppendSpeedLimit(s, 9);
        else
            speed_limit.AppendSpeedLimit(s, 15);
        s += ds;
    }

    // 速度规划  // 轨迹规划  // 处理动态障碍物
    auto t1 = std::clock();
    PiecewiseJerkSpeedNonlinearOptimizer piecewise_jerk_speed_optimizer;
    SpeedData optimized_speed_data;
    bool speed_planning_status =
        piecewise_jerk_speed_optimizer.Process(s_bounds, s_bounds, ref_s_list, speed_limit, dt, path, 7, 0, &optimized_speed_data, reference_crusie_velocity);
    auto t2 = std::clock();
    std::cout << "piecewise_jerk_speed_nonlinear_optimizer precess time: " << (double)(t2 - t1) / CLOCKS_PER_SEC << std::endl;

    double t = 0;
    while (t < 7)
    {
        SpeedPoint speed;
        optimized_speed_data.EvaluateByTime(t, &speed);
        // std::cout << "t: " << t << ", s: " << speed.s_ << ", v: " << speed.v_ << std::endl;
        t += 0.2;
    }

    // 横坐标t, 纵坐标s
    std::ofstream out;
    out.open("../plot/st_test.csv");
    out << "t,l,u,s,reference" << std::endl;
    for (int i = 0; i != s_bounds.size(); ++i)
    {
        double t = i * dt;
        SpeedPoint speed;
        optimized_speed_data.EvaluateByTime(t, &speed);
        // 时间，上边界，下边界，s
        out << t << "," << s_bounds[i].first << "," << s_bounds[i].second << "," << speed.s_ << "," << ref_s_list[i] << std::endl;
    }
    out.close();

    // 横坐标s, 纵坐标v
    out.open("../plot/v_test.csv");
    out << "s,b,v" << std::endl;
    ds = 0.3;
    double tmp_s = 0;
    while (tmp_s < optimized_speed_data.TotalLength())
    {
        double limit = speed_limit.GetSpeedLimitByS(tmp_s);
        SpeedPoint speed;
        optimized_speed_data.EvaluateByS(tmp_s, &speed);
        // s, speed_limit, v
        out << tmp_s << "," << limit << "," << speed.v_ << std::endl;
        tmp_s += ds;
    }
    out.close();
}