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

#include <control_lib/controllers/Feedback.hpp>
#include <control_lib/controllers/LinearDynamics.hpp>

#include <beautiful_bullet/bodies/MultiBody.hpp>

using namespace beautiful_bullet;
using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
    };

    struct feedback : public defaults::feedback {
    };

    struct linear_dynamics : public defaults::linear_dynamics {
    };
};

class OperationSpaceCtr : public control::MultiBodyCtr {
public:
    OperationSpaceCtr() : control::MultiBodyCtr(ControlMode::OPERATIONSPACE)
    {
        // step
        _dt = 0.01;

        // set controlled frame
        _frame = "lbr_iiwa_link_7";

        // set controller gains
        Eigen::MatrixXd D = 1 * Eigen::MatrixXd::Identity(6, 6);
        _controller.setDamping(D);

        // set ds gains
        Eigen::MatrixXd A = 0.1 * Eigen::MatrixXd::Identity(6, 6);
        _ds.setDynamicsMatrix(A);

        // goal
        Eigen::Vector3d xDes(0.365308, -0.0810892, 1.13717);
        xDes += Eigen::Vector3d(0, -1, 0);
        Eigen::Matrix3d oDes;
        oDes << 0.591427, -0.62603, 0.508233,
            0.689044, 0.719749, 0.0847368,
            -0.418848, 0.300079, 0.857041;
        _sDes._trans = xDes;
        _sDes._rot = oDes;

        // set reference
        _ds.setReference(spatial::SE3(oDes, xDes));
    }

    Eigen::VectorXd action(bodies::MultiBody& body) override
    {
        // current state
        spatial::SE3 sCurr(body.framePose(_frame));
        sCurr._vel = body.frameVelocity(_frame);

        // reference state
        spatial::SE3 sRef;
        sRef._vel = _ds.action(sCurr);

        return _controller.setReference(sRef).action(sCurr);
    }

protected:
    // step
    double _dt;

    // reference DS
    spatial::SE3 _sDes;

    // ds
    controllers::LinearDynamics<Params, spatial::SE3> _ds;

    // controller
    controllers::Feedback<Params, spatial::SE3> _controller;
};

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

    // Multi Bodies
    bodies::MultiBodyPtr franka = std::make_shared<bodies::MultiBody>("models/franka/urdf/panda.urdf");

    (*franka)
        .activateGravity();

    simulator.add(franka);

    // Run simulation
    simulator.run();

    return 0;
}