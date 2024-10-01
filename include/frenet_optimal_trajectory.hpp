#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include "cubic_spline_planner.hpp"

const double MAX_SPEED = 50.0 / 3.6;
const double MAX_ACCEL = 2.0;
const double MAX_CURVATURE = 2.0;
const double MAX_ROAD_WIDTH = 15.0;
const double D_ROAD_W = 1.0;
const double DT = 0.2;
const double MAX_T = 5.0;
const double MIN_T = 4.0;
const double TARGET_SPEED = 30.0 / 3.6;
const double D_T_S = 5.0 / 3.6;
const int N_S_SAMPLE = 1;
const double ROBOT_RADIUS = 2.0;

const double K_J = 0.1;
const double K_T = 0.1;
const double K_D = 1.0;
const double K_LAT = 1.0;
const double K_LON = 1.0;

class QuarticPolynomial {
public:
    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double time) {
        a0 = xs;
        a1 = vxs;
        a2 = axs / 2.0;

        Eigen::Matrix2d A;
        A << 3 * time * time, 4 * time * time * time,
             6 * time, 12 * time * time;
        Eigen::Vector2d b;
        b << vxe - a1 - 2 * a2 * time,
             axe - 2 * a2;
        Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

        a3 = x[0];
        a4 = x[1];
    }

    double calc_point(double t) const {
        return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t;
    }

    double calc_first_derivative(double t) const {
        return a1 + 2 * a2 * t + 3 * a3 * t * t + 4 * a4 * t * t * t;
    }

    double calc_second_derivative(double t) const {
        return 2 * a2 + 6 * a3 * t + 12 * a4 * t * t;
    }

    double calc_third_derivative(double t) const {
        return 6 * a3 + 24 * a4 * t;
    }

private:
    double a0, a1, a2, a3, a4;
};

class FrenetPath {
public:
    std::vector<double> t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd;
    double cd = 0.0, cv = 0.0, cf = 0.0;
    std::vector<double> x, y, yaw, ds, c;
};

std::vector<FrenetPath> calc_frenet_paths(double c_speed, double c_accel, double c_d, double c_d_d, double c_d_dd, double s0) {
    std::vector<FrenetPath> frenet_paths;

    for (double di = -MAX_ROAD_WIDTH; di <= MAX_ROAD_WIDTH; di += D_ROAD_W) {
        for (double Ti = MIN_T; Ti <= MAX_T; Ti += DT) {
            FrenetPath fp;

            QuarticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, Ti);

            for (double t = 0.0; t <= Ti; t += DT) {
                fp.t.push_back(t);
                fp.d.push_back(lat_qp.calc_point(t));
                fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
                fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
            }

            for (double tv = TARGET_SPEED - D_T_S * N_S_SAMPLE; tv <= TARGET_SPEED + D_T_S * N_S_SAMPLE; tv += D_T_S) {
                FrenetPath tfp = fp;
                QuarticPolynomial lon_qp(s0, c_speed, c_accel, tv, 0.0, Ti);

                for (double t : fp.t) {
                    tfp.s.push_back(lon_qp.calc_point(t));
                    tfp.s_d.push_back(lon_qp.calc_first_derivative(t));
                    tfp.s_dd.push_back(lon_qp.calc_second_derivative(t));
                    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
                }

                double Jp = std::accumulate(tfp.d_ddd.begin(), tfp.d_ddd.end(), 0.0, [](double sum, double val) { return sum + val * val; });
                double Js = std::accumulate(tfp.s_ddd.begin(), tfp.s_ddd.end(), 0.0, [](double sum, double val) { return sum + val * val; });
                double ds = std::pow(TARGET_SPEED - tfp.s_d.back(), 2);

                tfp.cd = K_J * Jp + K_T * Ti + K_D * std::pow(tfp.d.back(), 2);
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds;
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv;

                frenet_paths.push_back(tfp);
            }
        }
    }

    return frenet_paths;
}

std::vector<FrenetPath> calc_global_paths(std::vector<FrenetPath>& fplist, const CubicSpline2D& csp) {
    for (auto& fp : fplist) {
        for (size_t i = 0; i < fp.s.size(); ++i) {
            auto [ix, iy] = csp.calc_position(fp.s[i]);
            if (std::isnan(ix) || std::isnan(iy)) {
                break;
            }
            double i_yaw = std::atan2(fp.d[i], fp.s[i]);
            double di = fp.d[i];
            double fx = ix + di * std::cos(i_yaw + M_PI / 2.0);
            double fy = iy + di * std::sin(i_yaw + M_PI / 2.0);
            fp.x.push_back(fx);
            fp.y.push_back(fy);
        }

        for (size_t i = 0; i < fp.x.size() - 1; ++i) {
            double dx = fp.x[i + 1] - fp.x[i];
            double dy = fp.y[i + 1] - fp.y[i];
            fp.yaw.push_back(std::atan2(dy, dx));
            fp.ds.push_back(std::hypot(dx, dy));
        }

        fp.yaw.push_back(fp.yaw.back());
        fp.ds.push_back(fp.ds.back());

        for (size_t i = 0; i < fp.yaw.size() - 1; ++i) {
            fp.c.push_back((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i]);
        }
    }

    return fplist;
}

bool check_collision(const FrenetPath& fp, const std::vector<std::pair<double, double>>& ob) {
    for (const auto& [ox, oy] : ob) {
        for (size_t i = 0; i < fp.x.size(); ++i) {
            double d = std::pow(fp.x[i] - ox, 2) + std::pow(fp.y[i] - oy, 2);
            if (d <= std::pow(ROBOT_RADIUS, 2)) {
                return false;
            }
        }
    }
    return true;
}

std::vector<FrenetPath> check_paths(std::vector<FrenetPath>& fplist, const std::vector<std::pair<double, double>>& ob) {
    std::vector<FrenetPath> valid_paths;
    for (auto& fp : fplist) {
        if (std::any_of(fp.s_d.begin(), fp.s_d.end(), [](double v) { return v > MAX_SPEED; })) {
            continue;
        }
        if (std::any_of(fp.s_dd.begin(), fp.s_dd.end(), [](double a) { return std::abs(a) > MAX_ACCEL; })) {
            continue;
        }
        if (std::any_of(fp.c.begin(), fp.c.end(), [](double c) { return std::abs(c) > MAX_CURVATURE; })) {
            continue;
        }
        if (!check_collision(fp, ob)) {
            continue;
        }
        valid_paths.push_back(fp);
    }
    return valid_paths;
}

std::pair<FrenetPath, std::vector<FrenetPath>> frenet_optimal_planning(const CubicSpline2D& csp, double s0, double c_speed, double c_accel, double c_d, double c_d_d, double c_d_dd, const std::vector<std::pair<double, double>>& ob) {
    auto fplist = calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0);
    fplist = calc_global_paths(fplist, csp);
    fplist = check_paths(fplist, ob);

    double min_cost = std::numeric_limits<double>::infinity();
    FrenetPath best_path;
    for (const auto& fp : fplist) {
        if (fp.cf < min_cost) {
            min_cost = fp.cf;
            best_path = fp;
        }
    }

    return std::make_pair(best_path, fplist);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, CubicSpline2D> generate_target_course(const std::vector<double>& x, const std::vector<double>& y) {
    CubicSpline2D csp(x, y);
    std::vector<double> s;
    for (double i = 0.0; i < csp.s.back(); i += 0.1) {
        s.push_back(i);
    }

    std::vector<double> rx, ry, ryaw, rk;
    for (double i_s : s) {
        auto [ix, iy] = csp.calc_position(i_s);
        rx.push_back(ix);
        ry.push_back(iy);
        ryaw.push_back(std::atan2(iy, ix));
        rk.push_back(0.0); // Placeholder for curvature calculation
    }

    return std::make_tuple(rx, ry, ryaw, rk, csp);
}