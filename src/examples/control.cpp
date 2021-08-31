#include <iostream>

#include <beautiful_bullet/Simulator.hpp>

#ifdef GRAPHICS
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>
#endif

#include <control_lib/controllers/Feedback.hpp>

using namespace beautiful_bullet;
using namespace control_lib;

class ArmConfigurationControl : public Control {
public:
    ArmConfigurationControl() : Control(ControlMode::CONFIGURATIONSPACE) {}

    // ~ArmConfigurationControl() { delete _controller; }

    void init() override
    {
        // Space dimension
        _dim = 7;

        // Init your controller
        _controller = std::make_shared<controllers::Feedback>(ControlSpace::LINEAR, _dim, _dim, 0.01);

        // Define controller internal paramters
        Eigen::MatrixXd pGains = 1 * Eigen::MatrixXd::Identity(_dim, _dim),
                        dGains = 0.5 * Eigen::MatrixXd::Identity(_dim, _dim);

        _controller->setGains("p", pGains);
        _controller->setGains("d", dGains);

        // Set reference
        Eigen::VectorXd target = Eigen::VectorXd::Zero(2 * _dim);
        target.head(_dim) << 0., 0.2, 0.4, 0.3, 0.1, 0.5, 0.;
        _controller->setReference(target);
    }

    Eigen::VectorXd update(const Eigen::VectorXd& state) override { return _controller->update(state); }

protected:
    // Controller
    std::shared_ptr<controllers::Feedback> _controller;

    // Space dimension
    size_t _dim;
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

    // Create agent
    Agent iiwaBullet("models/iiwa_bullet/model.urdf"), iiwa("models/iiwa/urdf/iiwa14.urdf");

    // Set agents pose
    iiwaBullet.setPose(0, -1, 0);
    iiwa.setPose(0, 1, 0);

    // Set agents state
    Eigen::VectorXd state(7);
    state << 0, 0.2, 0, 0, 0, 0, 0;
    iiwaBullet.setState(state);

    // Add controllers
    iiwa.addControllers(std::make_shared<ArmConfigurationControl>());

    // Add agent to simulator
    simulator.addAgents(iiwaBullet, iiwa);

    // Run simulation
    simulator.run();

    return 0;
}