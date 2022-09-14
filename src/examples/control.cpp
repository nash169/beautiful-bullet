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

// Simulator
#include <beautiful_bullet/Simulator.hpp>

// Graphics
#ifdef GRAPHICS
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>
#endif

// Controllers
#include <control_lib/controllers/Feedback.hpp>
#include <control_lib/controllers/LinearDynamics.hpp>
#include <control_lib/controllers/QuadraticProgramming.hpp>

using namespace beautiful_bullet;
using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
    };

    struct feedback : public defaults::feedback {
    };

    struct linear_dynamics : public defaults::linear_dynamics {
    };

    struct quadratic_programming : public defaults::quadratic_programming {
    };
};

struct OperationSpaceFeedback : public control::MultiBodyCtr {
    OperationSpaceFeedback(const spatial::SE3& target) : control::MultiBodyCtr(ControlMode::OPERATIONSPACE)
    {
        // step
        _dt = 0.01;

        // set controlled frame
        _frame = "lbr_iiwa_link_7";

        // set ds gains and reference
        Eigen::MatrixXd A = 0.1 * Eigen::MatrixXd::Identity(6, 6);
        _ds.setDynamicsMatrix(A);
        _ds.setReference(target);

        // set controller gains
        Eigen::MatrixXd D = 1 * Eigen::MatrixXd::Identity(6, 6);
        _feedback.setDamping(D);
    }

    Eigen::VectorXd action(bodies::MultiBody& body) override
    {
        // current state
        spatial::SE3 sCurr(body.framePose(_frame));
        sCurr._vel = body.frameVelocity(_frame);

        // reference state
        spatial::SE3 sRef;
        sRef._vel = _ds.action(sCurr);

        return _feedback.setReference(sRef).action(sCurr);
    }

    // step
    double _dt;

    // ds & feedback
    controllers::LinearDynamics<Params, spatial::SE3> _ds;
    controllers::Feedback<Params, spatial::SE3> _feedback;
};

struct TargetDynamics {
    TargetDynamics()
    {
        // set ds gains and reference
        Eigen::MatrixXd A = 0.1 * Eigen::MatrixXd::Identity(6, 6);
        _ds.setDynamicsMatrix(A);
    }

    void setTarget(const spatial::SE3& target)
    {
        _ds.setReference(target);
    }

    void update(const spatial::SE3& curr)
    {
        _ds.update(curr);
    }

    const Eigen::VectorXd& velocity() const
    {
        return _ds.output();
    }

    controllers::LinearDynamics<Params, spatial::SE3> _ds;
};

struct ConfigurationSpaceQP : public control::MultiBodyCtr {
    ConfigurationSpaceQP(const spatial::SE3& target) : control::MultiBodyCtr(ControlMode::CONFIGURATIONSPACE)
    {
        // step
        _dt = 0.01;

        // set controlled frame
        _frame = "lbr_iiwa_link_7";

        // set ds
        _ds.setTarget(target);

        // set qp
        Eigen::MatrixXd Q = 1 * Eigen::MatrixXd::Identity(7, 7),
                        R = 0.1 * Eigen::MatrixXd::Identity(7, 7);
        _qp
            .setDimensions(7, 6)
            .energyMinimization(Q)
            .controlSaturation(R);
    }

    Eigen::VectorXd action(bodies::MultiBody& body) override
    {
        // reference state
        spatial::SE3 sCurr(body.framePose(_frame));
        _ds.update(sCurr);

        // update qp
        _qp
            .modelDynamics(body)
            // .positionLimits(body)
            // .velocityLimits(body)
            // .effortLimits(body)
            // .inverseKinematics(body, _ds)
            .init();

        return _qp.action(sCurr);
    }

    // step
    double _dt;

    // ds & feedback
    TargetDynamics _ds;
    controllers::QuadraticProgramming<Params, spatial::SE3> _qp;
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
    bodies::MultiBodyPtr iiwaBullet = std::make_shared<bodies::MultiBody>("models/iiwa_bullet/model.urdf"),
                         iiwa = std::make_shared<bodies::MultiBody>("models/iiwa/urdf/iiwa14.urdf");

    // Task space target
    Eigen::Vector3d xDes(0.365308, -1.0810892, 1.13717);
    Eigen::Matrix3d oDes = (Eigen::Matrix3d() << 0.591427, -0.62603, 0.508233, 0.689044, 0.719749, 0.0847368, -0.418848, 0.300079, 0.857041).finished();
    spatial::SE3 tDes(oDes, xDes);

    // Set controlled robot
    (*iiwaBullet)
        .setPosition(0, -1, 0)
        // .addControllers(std::make_unique<OperationSpaceFeedback>(tDes))
        .addControllers(std::make_unique<ConfigurationSpaceQP>(tDes))
        .activateGravity();

    // Reference configuration
    Eigen::VectorXd state(7);
    state << 0., 0.7, 0.4, 0.6, 0.3, 0.5, 0.1;

    // Set reference robot
    (*iiwa)
        .setState(state)
        .setPosition(0, 1, 0)
        .activateGravity();

    // Add robots and run simulation
    simulator.add(iiwaBullet, iiwa);
    simulator.run();

    return 0;
}
