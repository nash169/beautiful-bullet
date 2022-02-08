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

    // Try inverse kinematics
    // std::cout << iiwaBullet.poseJoint().transpose() << std::endl;

    // Eigen::Vector3d xCurr(0, 0, 1.18);
    // Eigen::Matrix3d oCurr;
    // oCurr << -1, 0, 0,
    //     0, 0, 1,
    //     0, 1, 0;

    Eigen::Vector3d xDes(0.324141, -0.0879529, 1.06775);
    Eigen::Matrix3d oDes;
    oDes << -0.650971, 0.508233, -0.563859,
        -0.613746, 0.0847368, 0.784943,
        0.446713, 0.857041, 0.256765;

    // control_lib::utils::ControlState poseCurr(6, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS),
    //     poseDes(6, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS);

    // pinocchio::SE3 pose(oDes, xDes);
    // Eigen::Matrix<double, 6, 1> liealgebra = pinocchio::log6(pose);

    // std::cout << liealgebra.transpose() << std::endl;

    // Eigen::AngleAxisd aaxis(oDes);

    // std::cout << aaxis.angle() * aaxis.axis().transpose() << std::endl;

    // std::cout << beautiful_bullet::tools::lieAlgebraSE3(xDes, oDes).transpose() << std::endl;

    // std::cout << iiwaBullet.inverseKinematics(xDes, oDes).transpose() << std::endl;

    // std::cout << iiwaBullet.poseJoint().transpose() << std::endl;

    // poseCurr.setState(iiwaBullet.poseJoint());

    // Set agents state
    Eigen::VectorXd state(7);
    state << 0., 0.7, 0.4, 0.6, 0.3, 0.5, 0.1;
    iiwaBullet.setState(state);

    // poseDes.setState(iiwaBullet.poseJoint());

    // pinocchio::SE3 oMdes(oDes, xDes), oMcurr(oCurr, xCurr);

    // std::cout << poseDes.getPos().transpose() << std::endl;
    // std::cout << pinocchio::log6(oMdes).toVector().transpose() << std::endl;

    // Eigen::AngleAxisd temp(oDes.transpose() * oCurr);
    // std::cout << temp.angle() * temp.axis() << std::endl;

    // pinocchio::SE3 dMi = oMdes.actInv(oMcurr);
    // std::cout << pinocchio::log6(dMi).toVector().transpose() << std::endl;
    // std::cout << dMi.translation().transpose() << std::endl;
    // std::cout << (poseCurr - poseDes)._pose.transpose() - 1 << std::endl;

    // double k = 2.3;

    // Eigen::Matrix<double, 3, 1> errOrientation = (poseCurr - poseDes)._orientation;
    // Eigen::Matrix3d mat = oDes.transpose() * oCurr;
    // Eigen::Matrix3d mat = Eigen::AngleAxisd(errOrientation.norm(), errOrientation.normalized()).toRotationMatrix();

    // std::cout << (poseCurr - poseDes).getPos().transpose() << std::endl;
    // std::cout << k * beautiful_bullet::tools::lieAlgebraSE3(oDes.transpose() * (poseCurr - poseDes)._pose, mat).transpose() << std::endl;

    // std::cout << iiwaBullet.poseJoint().transpose() << std::endl;

    // Add controllers
    // iiwa.setState(iiwa.inverseKinematics(xDes, oDes));
    iiwa.addControllers(std::make_unique<OperationSpaceControl>());
    // iiwa.addControllers(std::make_unique<ConfigurationSpaceControl>());

    // Add agent to simulator
    simulator.addAgents(iiwaBullet, iiwa);

    // Run simulation
    simulator.run();

    return 0;
}