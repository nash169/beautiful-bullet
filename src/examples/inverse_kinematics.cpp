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

#include <beautiful_bullet/Simulator.hpp>

#ifdef GRAPHICS
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>
#endif

#include <beautiful_bullet/bodies/MultiBody.hpp>

#include <Magnum/EigenIntegration/Integration.h>

using namespace beautiful_bullet;

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v)
{
    return (Eigen::Matrix3d() << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0).finished();
}

Eigen::Matrix3d rotationAlign(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
    Eigen::Vector3d k = u.cross(v);
    Eigen::Matrix3d K = skewSymmetric(k);
    double c = u.dot(v);

    return Eigen::Matrix3d::Identity() + K + K * K / (1 + c);
}

int main(int argc, char const* argv[])
{
    // Create simulator
    Simulator simulator;

// Add graphics
#ifdef GRAPHICS
    simulator.setGraphics(std::make_unique<graphics::MagnumGraphics>());
#endif

    // Add ground
    simulator.addGround();

    // Rigid body
    double radius = 0.4;
    Eigen::Vector3d center = Eigen::Vector3d(0.8, 0.0, 0.5);
    bodies::SphereParams paramsSphere;
    paramsSphere.setRadius(radius - 0.1).setMass(0.0).setFriction(0.5).setColor("grey");
    bodies::RigidBodyPtr sphere = std::make_shared<bodies::RigidBody>("sphere", paramsSphere);
    sphere->setPosition(center[0], center[1], center[2]);

    // MultiBody
    bodies::MultiBodyPtr iiwa = std::make_shared<bodies::MultiBody>("models/iiwa/urdf/iiwa14.urdf");

    Eigen::Vector3d xDes = radius * Eigen::Vector3d(-1.0, 0.0, 1.0).normalized() + center;
    Eigen::Matrix3d oDes = rotationAlign(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(1.0, 0.0, -1.0).normalized());

    std::string frame = "lbr_iiwa_link_7";
    Eigen::VectorXd q_ref = (Eigen::Matrix<double, 7, 1>() << 0, 0, 0, -M_PI / 4, 0, M_PI / 4, 0).finished();
    iiwa->setState(q_ref);
    Eigen::VectorXd state = iiwa->inverseKinematics(xDes, oDes, frame);

    (*iiwa)
        .setState(state)
        .activateGravity();

    simulator.add(iiwa, sphere);

    Vector3 translation(Eigen::Vector3f(xDes.cast<float>()));
    Matrix3 rotation(Eigen::Matrix3f(oDes.cast<float>()));

    static_cast<graphics::MagnumGraphics&>(simulator.graphics())
        .app()
        .addFrame()
        .setTransformation(Matrix4::translation(translation) * Matrix4(rotation) * Matrix4::scaling({0.6, 0.6, 0.6}));

    Vector3 translation_ee(Eigen::Vector3f(iiwa->framePosition().cast<float>()));
    Matrix3 rotation_ee(Eigen::Matrix3f(iiwa->frameOrientation().cast<float>()));

    static_cast<graphics::MagnumGraphics&>(simulator.graphics())
        .app()
        .addFrame()
        .setTransformation(Matrix4::translation(translation_ee) * Matrix4(rotation_ee) * Matrix4::scaling({0.5, 0.5, 0.5}));

    // Run simulation
    simulator.run();

    std::cout << xDes.transpose() << std::endl;
    std::cout << iiwa->framePosition().transpose() << std::endl;
    std::cout << "-" << std::endl;
    std::cout << oDes << std::endl;
    std::cout << iiwa->frameOrientation() << std::endl;

    return 0;
}
