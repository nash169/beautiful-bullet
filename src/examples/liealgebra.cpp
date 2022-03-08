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

#include <iostream>

#include <Eigen/Core>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

int main(int argc, char const* argv[])
{
    // Model
    pinocchio::Model model;
    pinocchio::urdf::buildModel("models/iiwa_bullet/model.urdf", model);
    pinocchio::Data data(model);

    // Frame ID
    const std::string name = "lbr_iiwa_link_7";
    const int id = model.getFrameId(name);

    // Random joint position and velocity
    Eigen::Matrix<double, 7, 1> q = Eigen::VectorXd::Random(7),
                                v = Eigen::VectorXd::Random(7);

    // Forward Kinematics
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::framesForwardKinematics(model, data, q);

    // Jacobian
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, id, J);

    // Pose
    pinocchio::SE3 pose = data.oMf[id];
    Eigen::Vector3d trans = pose.translation();
    Eigen::Matrix3d rot = pose.rotation();

    // Compare
    std::cout << "Jacobian derivation" << std::endl;
    std::cout << (J * v).transpose() << std::endl;

    std::cout << "Lie Algebra derivation" << std::endl;
    std::cout << pinocchio::log6(pose).toVector().transpose() << std::endl;

    return 0;
}
