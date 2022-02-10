#include <iostream>

#include <beautiful_bullet/Simulator.hpp>

#ifdef GRAPHICS
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>
#endif

#include <control_lib/controllers/Feedback.hpp>

#include <CommonInterfaces/CommonGUIHelperInterface.h>
#include <ExampleBrowser/OpenGLGuiHelper.h>
#include <Importers/ImportURDFDemo/BulletUrdfImporter.h>
#include <Importers/ImportURDFDemo/MyMultiBodyCreator.h>
#include <Importers/ImportURDFDemo/URDF2Bullet.h>
#include <OpenGLWindow/SimpleOpenGL3App.h>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <beautiful_bullet/tools/math.hpp>

using namespace beautiful_bullet;
using namespace control_lib;

class ConfigurationSpaceControl : public Control {
public:
    ConfigurationSpaceControl() : Control(ControlMode::CONFIGURATIONSPACE) {}

    void init() override
    {
        // Space dimension
        _dim = 7;

        // Init your controller
        _controller = std::make_unique<controllers::Feedback>(ControlSpace::LINEAR, _dim, _dim, 0.01);

        // Define controller internal paramters
        Eigen::MatrixXd pGains = 100 * Eigen::MatrixXd::Identity(_dim, _dim),
                        dGains = 20 * Eigen::MatrixXd::Identity(_dim, _dim);

        pGains(5, 5) = 10;
        pGains(6, 6) = 5;
        dGains(5, 5) = 2;
        dGains(6, 6) = 1;

        _controller->setGains("p", pGains);
        _controller->setGains("d", dGains);

        // Set reference
        Eigen::VectorXd target = Eigen::VectorXd::Zero(2 * _dim);
        target.head(_dim) << 0., 0.7, 0.4, 0.6, 0.3, 0.5, 0.1;
        _controller->setReference(target);
    }

    Eigen::VectorXd update(const Eigen::VectorXd& state) override { return _controller->update(state); }

protected:
    // Controller
    std::unique_ptr<controllers::Feedback> _controller;

    // Space dimension
    size_t _dim;
};

class OperationSpaceControl : public Control {
public:
    OperationSpaceControl() : Control(ControlMode::OPERATIONSPACE) {}

    void init() override
    {
        // Space dimension (we use here se3 lie algebra representation)
        _dim = 6;

        // Init your controller
        _controller = std::make_shared<controllers::Feedback>(ControlSpace::LINEAR | ControlSpace::ANGLEAXIS, _dim, _dim, 0.01);

        // Define controller internal paramters
        Eigen::MatrixXd pGains = 1 * Eigen::MatrixXd::Identity(_dim, _dim),
                        dGains = 0 * Eigen::MatrixXd::Identity(_dim, _dim);

        _controller->setGains("p", pGains);
        _controller->setGains("d", dGains);

        // target position
        _xDes << 0.324141, -0.0879529, 1.06775;

        // target orientation
        _oDes << -0.650971, 0.508233, -0.563859,
            -0.613746, 0.0847368, 0.784943,
            0.446713, 0.857041, 0.256765;
        Eigen::AngleAxisd aaDes(_oDes);

        // Set reference
        Eigen::VectorXd target = Eigen::VectorXd::Zero(2 * _dim);
        target.head(3) << _oDes.transpose() * _xDes;
        target.segment(3, 3) = aaDes.angle() * aaDes.axis();
        _controller->setReference(target);
    }

    Eigen::VectorXd update(const Eigen::VectorXd& state) override
    {
        Eigen::VectorXd mod_state(state.size());
        mod_state.head(3) = _oDes.transpose() * state.head(3);
        mod_state.tail(3) = state.tail(3);

        return _controller->update(state);
    }

protected:
    // Controller
    std::shared_ptr<controllers::Feedback> _controller;

    // Space dimension
    size_t _dim;

    // Desired target
    Eigen::Vector3d _xDes;
    Eigen::Matrix3d _oDes;
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
    iiwaBullet.setPosition(0, -1, 0);
    iiwa.setPosition(0, 1, 0);

    // Set agents state
    Eigen::VectorXd state(7);
    state << 0., 0.7, 0.4, 0.6, 0.3, 0.5, 0.1;
    iiwa.setState(state);

    // Inverse Kinematics
    Eigen::Vector3d xDes(0.365308, -0.0810892, 1.13717);
    Eigen::Matrix3d oDes;
    oDes << 0.591427, -0.62603, 0.508233,
        0.689044, 0.719749, 0.0847368,
        -0.418848, 0.300079, 0.857041;

    // iiwaBullet.setState(iiwaBullet.inverseKinematics(xDes, oDes));

    // Control
    iiwaBullet.addControllers(std::make_unique<OperationSpaceControl>());
    // iiwaBullet.addControllers(std::make_unique<ConfigurationSpaceControl>());

    // Add agent to simulator
    simulator.addAgents(iiwaBullet, iiwa);

    // Run simulation
    simulator.run();

    return 0;
}