/*
    This file is part of beautiful-bullet.

    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

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

        // Check here if this can be improved
        Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& M)
        {
            double epsilon = std::numeric_limits<double>::epsilon();

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

            Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();

            double tolerance = epsilon * std::max(M.cols(), M.rows()) * sing_vals.array().abs()(0);
            Eigen::MatrixXd S = Eigen::MatrixXd::Zero(M.rows(), M.cols());

            for (size_t i = 0; i < sing_vals.size(); i++)
                if (std::fabs(sing_vals(i)) > tolerance)
                    S(i, i) = 1.0 / sing_vals(i);

            return svd.matrixV() * S.transpose() * svd.matrixU().adjoint();
        }
    } // namespace tools

} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_TOOLS_MATH_HPP