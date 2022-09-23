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

#include <Eigen/Dense>
#include <beautiful_bullet/bodies/MultiBody.hpp>
#include <iostream>
#include <utils_lib/DerivativeChecker.hpp>

// #include <beautiful_bullet/tools/math.hpp>

// #include <pinocchio/algorithm/frames.hpp>
// #include <pinocchio/algorithm/jacobian.hpp>
// #include <pinocchio/algorithm/joint-configuration.hpp>
// #include <pinocchio/algorithm/kinematics.hpp>
// #include <pinocchio/algorithm/rnea.hpp>
// #include <pinocchio/parsers/urdf.hpp>

using namespace beautiful_bullet;
using namespace utils_lib;

Eigen::Matrix3d expTransOperator(const Eigen::Matrix3d& R) // u->t se3->SE3 (V)
{
    Eigen::AngleAxisd aa(R);
    Eigen::Vector3d omega = aa.angle() * aa.axis();
    Eigen::Matrix3d omega_x = (Eigen::Matrix3d() << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1), omega(0), 0).finished();

    double theta = omega.norm(),
           A = std::sin(theta) / theta,
           B = (1 - std::cos(theta)) / std::pow(theta, 2),
           C = (1 - A) / std::pow(theta, 2);

    return Eigen::Matrix3d::Identity() + B * omega_x + C * omega_x * omega_x;
}

Eigen::Matrix3d logTransOperator(const Eigen::Matrix3d& R) // t->u SE3->se3 (V^-1)
{
    Eigen::AngleAxisd aa(R);
    Eigen::Vector3d omega = aa.angle() * aa.axis();
    Eigen::Matrix3d omega_x = (Eigen::Matrix3d() << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1), omega(0), 0).finished();

    double theta = omega.norm(),
           A = std::sin(theta) / theta,
           B = (1 - std::cos(theta)) / std::pow(theta, 2);

    return Eigen::Matrix3d::Identity() - 0.5 * omega_x + (1 - A / 2 / B) / std::pow(theta, theta) * omega_x * omega_x;
}

struct DerivativesTest {
    DerivativesTest()
    {
        _frame = "lbr_iiwa_link_7";
        _model.loadBody("models/iiwa_bullet/model.urdf");
    }

    Eigen::Matrix<double, 3, 1> function(const Eigen::Matrix<double, 7, 1>& q)
    {
        return _model.framePose(q, _frame).head(3);
    }

    Eigen::Matrix<double, 3, 7> gradient(const Eigen::Matrix<double, 7, 1>& q)
    {
        return _model.frameOrientation(q, _frame) * _model.jacobian(q, _frame).block(0, 0, 3, 7);
    }

    Eigen::Matrix<double, 3, 7> hessian(const Eigen::Matrix<double, 7, 1>& q, const Eigen::Matrix<double, 7, 1>& dq)
    {
        Eigen::Vector3d omega = (_model.jacobian(q, _frame) * dq).tail(3);
        Eigen::Matrix3d omega_x = (Eigen::Matrix3d() << 0, -omega(2), omega(1), omega(2), 0, -omega(0), -omega(1), omega(0), 0).finished(),
                        R = _model.frameOrientation(q, _frame),
                        R_dot = R * omega_x;

        return R_dot * _model.jacobian(q, _frame).block(0, 0, 3, 7) + R * _model.jacobianDerivative(q, dq, _frame).block(0, 0, 3, 7);
    }

protected:
    std::string _frame;
    bodies::MultiBody _model;
};

int main(int argc, char const* argv[])
{
    // // Model
    // pinocchio::Model model;
    // pinocchio::urdf::buildModel("models/iiwa_bullet/model.urdf", model);
    // pinocchio::Data data(model);

    // Frame ID
    // const std::string name = "lbr_iiwa_link_7";
    // const int id = model.getFrameId(name);

    // Random joint position and velocity
    Eigen::Matrix<double, 7, 1> q = Eigen::VectorXd::Random(7),
                                v = Eigen::VectorXd::Random(7);

    DerivativeChecker<double> checker(7, 3);

    DerivativesTest test;

    auto f = std::bind(&DerivativesTest::function, &test, std::placeholders::_1);
    auto g = std::bind(&DerivativesTest::gradient, &test, std::placeholders::_1);
    auto h = std::bind(&DerivativesTest::hessian, &test, std::placeholders::_1, std::placeholders::_2);

    if (checker.checkGradientMulti(f, g))
        std::cout << "The gradient is CORRECT!" << std::endl;
    else
        std::cout << "The gradient is NOT correct!" << std::endl;

    if (checker.checkHessianMulti(f, g, h))
        std::cout << "The hessian is CORRECT!" << std::endl;
    else
        std::cout << "The hessian is NOT correct!" << std::endl;

    // Forward Kinematics
    // pinocchio::forwardKinematics(model, data, q, v);
    // pinocchio::framesForwardKinematics(model, data, q);

    // Jacobian
    // pinocchio::Data::Matrix6x J(6, model.nv);
    // J.setZero();
    // pinocchio::computeFrameJacobian(model, data, q, id, J);

    // // Pose
    // pinocchio::SE3 pose = data.oMf[id];
    // Eigen::Vector3d trans = pose.translation();
    // Eigen::Matrix3d rot = pose.rotation();

    // std::cout << trans.transpose() << std::endl;
    // std::cout << rot.transpose() << std::endl;

    // // Compare
    // std::cout << "Jacobian derivation" << std::endl;
    // std::cout << (J * v).transpose() << std::endl;

    // std::cout << "Lie Algebra derivation" << std::endl;
    // std::cout << pinocchio::log6(pose).toVector().transpose() << std::endl;

    return 0;
}
