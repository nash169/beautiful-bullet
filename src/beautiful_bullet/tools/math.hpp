#ifndef BEAUTIFUL_BULLET_TOOLS_MATH_HPP
#define BEAUTIFUL_BULLET_TOOLS_MATH_HPP

#include <Eigen/Geometry>

namespace beautiful_bullet {
    namespace tools {
        Eigen::Matrix<double, 6, 1> lieAlgebraSE3(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation)
        {
            Eigen::AngleAxisd aa(orientation);

            Eigen::Vector3d omega = aa.angle() * aa.axis();

            Eigen::Matrix3d omega_skew;
            omega_skew << 0, -omega(2), omega(1),
                omega(2), 0, -omega(0),
                -omega(1), omega(0), 0;

            double theta = omega.norm(), A = std::sin(theta) / theta, B = (1 - std::cos(theta)) / std::pow(theta, 2);

            return (Eigen::Matrix<double, 6, 1>() << (Eigen::Matrix3d::Identity() - 0.5 * omega_skew + (1 - 0.5 * A / B) / std::pow(theta, 2) * omega_skew * omega_skew) * position, omega).finished();
        }
    } // namespace tools

} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_TOOLS_MATH_HPP