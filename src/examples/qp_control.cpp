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

#include <beautiful_bullet/Simulator.hpp>

#ifdef GRAPHICS
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>
#endif

#include <control_lib/controllers/LinearDynamics.hpp>
#include <control_lib/controllers/QuadraticProgramming.hpp>

#include <beautiful_bullet/bodies/MultiBody.hpp>

using namespace beautiful_bullet;
using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
    };

    struct quadratic_programming : public defaults::quadratic_programming {
    };

    struct linear_dynamics : public defaults::linear_dynamics {
    };
};

int main(int argc, char const* argv[])
{
    // Model
    bodies::MultiBodyPtr franka = std::make_shared<bodies::MultiBody>("models/franka/urdf/panda.urdf");

    // Controller
    controllers::QuadraticProgramming<Params> ctr(7, 6);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(7, 7),
                    R = Eigen::MatrixXd::Identity(7, 7);

    spatial::SE3 x(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    ctr.energyMinimization(Q)
        .controlSaturation(R)
        .modelDynamics(*franka)
        .positionLimits(*franka)
        .velocityLimits(*franka)
        .effortLimits(*franka)
        .init();

    std::cout << ctr.action(x).transpose() << std::endl;

    // ctr.dynamicsTracking(Q, Target());
    // ctr.controlSaturation(R);
    // ctr.slackVariable();
    // ctr.modelDynamics(Model());
    // ctr.inverseKinematics(Model(), Reference());
    // ctr.inverseDynamics(Model(), Reference());
    // ctr.positionLimits(Model());
    // ctr.velocityLimits(Model());
    // ctr.accelerationLimits(Model());
    // ctr.effortLimits(Model());

    return 0;
}