#include <iostream>

#include <beautiful_bullet/Simulator.hpp>

#ifdef GRAPHICS
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>
#endif

using namespace beautiful_bullet;

// class SphereControl : public Control {
// public:
//     SphereControl() : Control(ControlMode::OPERATIONSPACE) {}

//     // ~SphereControl() { delete _controller; }

//     void init() override
//     {
//         // Space dimension
//         _dim = 7;

//         // Init your controller
//         _controller = std::make_shared<controllers::Feedback>(ControlSpace::LINEAR | ControlSpace::QUATERNION, _dim, _dim, 0.01);

//         // Define controller internal paramters
//         Eigen::MatrixXd pGains = 10 * Eigen::MatrixXd::Identity(_dim, _dim),
//                         dGains = 2 * Eigen::MatrixXd::Identity(_dim, _dim);

//         _controller->setGains("p", pGains);
//         _controller->setGains("d", dGains);

//         // Set reference
//         Eigen::VectorXd target = Eigen::VectorXd::Zero(2 * _dim);
//         target.head(_dim) << -0.5, 0, 1, 0, 0, 0, 1;
//         _controller->setReference(target);
//     }

//     Eigen::VectorXd update(const Eigen::VectorXd& state) override { return _controller->update(state); }

// protected:
//     // Controller
//     std::shared_ptr<controllers::Feedback> _controller;

//     // Space dimension
//     size_t _dim;
// };

int main(int argc, char** argv)
{
    // Create simulator
    Simulator simulator;

// Add graphics
#ifdef GRAPHICS
    simulator.setGraphics(std::make_unique<graphics::MagnumGraphics>());
#endif

    // Add ground
    simulator.addGround();

    // Create object
    SphereParams params;
    params.setRadius(0.2)
        .setMass(0.0)
        .setFriction(0.5)
        .setPose(Eigen::Vector3d(0.5, 0, 1))
        .setColor("grey");

    Object sphere("sphere", params);

    // Add object to simulator
    simulator.addObjects(sphere);

    // Create agent
    Agent franka("models/franka/urdf/panda.urdf");

    // Add agent to simulator
    simulator.addAgents(franka);

    // Run simulation
    simulator.run();

    return 0;
}