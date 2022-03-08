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

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

int main(int argc, char const* argv[])
{
    pinocchio::Model model;
    pinocchio::urdf::buildModel("models/iiwa_bullet/model.urdf", model);
    pinocchio::Data data(model);
    pinocchio::Data::Matrix6x J1(6, model.nv), J2(6, model.nv), J3(6, model.nv), J4(6, model.nv);

    const std::string name = "lbr_iiwa_link_7";
    const int id = model.getFrameId(name);
    Eigen::VectorXd q(7);
    q << 0., 0.7, 0.4, 0.6, 0.3, 0.5, 0.1;

    pinocchio::framesForwardKinematics(model, data, Eigen::VectorXd::Zero(7));
    pinocchio::SE3 initPose = data.oMf[id];
    pinocchio::framesForwardKinematics(model, data, q);
    pinocchio::SE3 endPose = data.oMf[id];

    std::cout << data.oMf.size() << std::endl;

    // pinocchio::forwardKinematics(*_model, *_data, _q, _v);
    if (model.existFrame(name))
        std::cout << "Frame ID: " << model.getFrameId(name) << std::endl;
    if (model.existJointName(name))
        std::cout << "Joint ID: " << model.getJointId(name) << std::endl;
    if (model.existBodyName(name))
        std::cout << "Body ID: " << model.getBodyId(name) << std::endl;

    std::cout << "Initial Pose" << std::endl;
    std::cout << initPose.translation().transpose() << std::endl;
    std::cout << initPose.rotation() << std::endl;

    std::cout << "Final Pose" << std::endl;
    std::cout << endPose.translation().transpose() << std::endl;
    std::cout << endPose.rotation() << std::endl;

    // Joint Jacobian
    J1.setZero();
    pinocchio::computeJointJacobian(model, data, q, 7, J1);
    std::cout << "J1" << std::endl;
    std::cout << J1 << std::endl;

    // Full joint Jacobian
    J2.setZero();
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::getJointJacobian(model, data, 7, pinocchio::LOCAL, J2);
    std::cout << "J2" << std::endl;
    std::cout << J2 << std::endl;

    // Frame Jacobian
    J3.setZero();
    pinocchio::computeFrameJacobian(model, data, q, id, J3);
    std::cout << "J3" << std::endl;
    std::cout << J3 << std::endl;

    // Full frame Jacobians
    // pinocchio::getFrameJacobian(*_model, *_data, _model->getFrameId("lbr_iiwa_joint_7"), pinocchio::LOCAL, J);

    std::cout << "Jacobian pose: " << (J3 * (q - Eigen::VectorXd::Zero(7))).transpose() << std::endl;
    std::cout << "se3 pose: " << pinocchio::log6(endPose.actInv(initPose)).toVector().transpose() << std::endl;

    return 0;
}
