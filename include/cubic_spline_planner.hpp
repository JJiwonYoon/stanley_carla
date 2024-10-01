#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

class CubicSpline1D {
public:
    CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y) {
        if (x.size() != y.size()) {
            throw std::invalid_argument("x and y must be the same size");
        }

        size_t nx = x.size();
        std::vector<double> h(nx - 1);
        for (size_t i = 0; i < nx - 1; ++i) {
            h[i] = x[i + 1] - x[i];
            if (h[i] <= 0) {
                throw std::invalid_argument("x coordinates must be sorted in ascending order");
            }
        }

        this->x = x;
        this->y = y;
        this->nx = nx;
        this->a = y;

        Eigen::MatrixXd A = calc_A(h);
        Eigen::VectorXd B = calc_B(h, a);
        Eigen::VectorXd c = A.colPivHouseholderQr().solve(B);

        this->c = std::vector<double>(c.data(), c.data() + c.size());

        for (size_t i = 0; i < nx - 1; ++i) {
            double d = (this->c[i + 1] - this->c[i]) / (3.0 * h[i]);
            double b = (this->a[i + 1] - this->a[i]) / h[i] - h[i] / 3.0 * (2.0 * this->c[i] + this->c[i + 1]);
            this->d.push_back(d);
            this->b.push_back(b);
        }
    }

    double calc_position(double t) const {
        if (t < x.front() || t > x.back()) {
            throw std::out_of_range("t is out of the range of x");
        }

        size_t i = search_index(t);
        double dx = t - x[i];
        return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    }

private:
    std::vector<double> x, y, a, b, c, d;
    size_t nx;

    size_t search_index(double t) const {
        auto it = std::lower_bound(x.begin(), x.end(), t);
        return std::distance(x.begin(), it) - 1;
    }

    Eigen::MatrixXd calc_A(const std::vector<double>& h) const {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
        A(0, 0) = 1.0;
        for (size_t i = 0; i < nx - 1; ++i) {
            if (i != nx - 2) {
                A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
            }
            A(i + 1, i) = h[i];
            A(i, i + 1) = h[i];
        }
        A(0, 1) = 0.0;
        A(nx - 1, nx - 2) = 0.0;
        A(nx - 1, nx - 1) = 1.0;
        return A;
    }

    Eigen::VectorXd calc_B(const std::vector<double>& h, const std::vector<double>& a) const {
        Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
        for (size_t i = 0; i < nx - 2; ++i) {
            B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
        }
        return B;
    }
};

class CubicSpline2D {
public:
    CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y) {
        s = calc_s(x, y);
        sx = CubicSpline1D(s, x);
        sy = CubicSpline1D(s, y);
    }

    std::pair<double, double> calc_position(double s) const {
        double x = sx.calc_position(s);
        double y = sy.calc_position(s);
        return std::make_pair(x, y);
    }

private:
    std::vector<double> s;
    CubicSpline1D sx, sy;

    std::vector<double> calc_s(const std::vector<double>& x, const std::vector<double>& y) const {
        std::vector<double> s(x.size());
        s[0] = 0.0;
        for (size_t i = 1; i < x.size(); ++i) {
            double dx = x[i] - x[i - 1];
            double dy = y[i] - y[i - 1];
            s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
        }
        return s;
    }
};