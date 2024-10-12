#ifndef APOLLO_SPEED_INCLUDE_piecewise_jerk_speed_nonlinear_optimizer_HPP_
#define APOLLO_SPEED_INCLUDE_piecewise_jerk_speed_nonlinear_optimizer_HPP_
#include <vector>
#include "DiscretizedPath.hpp"
#include "PiecewiseJerkTrajectory1d.hpp"
#include "SpeedLimit.hpp"
#include "SpeedData.hpp"
#include "DiscretizedPath.hpp"

class PiecewiseJerkSpeedNonlinearOptimizer
{
public:
    PiecewiseJerkSpeedNonlinearOptimizer();
    bool Process(std::vector<std::pair<double, double>> &s_bounds,
                 const std::vector<std::pair<double, double>> &soft_s_bounds,
                 const std::vector<double> &ref_s_list,
                 const SpeedLimit &speed_limit,
                 double dt,
                 const DiscretizedPath &path,
                 double init_v,
                 double init_a,
                 SpeedData *speed_data,
                 double reference_cruise_velocity);
    bool OptimizeByQP(const std::vector<std::pair<double, double>> &s_bounds,
                      const std::vector<double> &ref_s_list,
                      std::vector<double> *distance,
                      std::vector<double> *velocity,
                      std::vector<double> *acceleration);
    bool SmoothPathCurvature(const DiscretizedPath &cartesian_path);
    bool SmoothSpeedLimit(const SpeedLimit &speed_limit);
    bool speed_non_linear_optimizer(const std::vector<std::pair<double, double>> &s_bounds,
                                    const std::vector<std::pair<double, double>> &soft_s_bounds,
                                    std::vector<double> *distance,
                                    std::vector<double> *velocity,
                                    std::vector<double> *acceleration,
                                    double reference_cruise_velocity);

private:
    double delta_t_;
    double total_length_;
    double total_time_;
    int num_of_knots_;
    double s_init_;
    double s_dot_init_;
    double s_ddot_init_;
    double s_dot_max_, s_ddot_max_, s_ddot_min_, s_dddot_max_, s_dddot_min_;
    //    std::vector<std::pair<double, double>> s_bounds_;
    //    std::vector<std::pair<double, double>> s_soft_bounds_;
    // smoothed path curvature profile as a function of traversal distance
    PiecewiseJerkTrajectory1d smoothed_path_curvature_;
    PiecewiseJerkTrajectory1d smoothed_speed_limit_;
};

#endif //APOLLO_SPEED_INCLUDE_piecewise_jerk_speed_nonlinear_optimizer_HPP_
