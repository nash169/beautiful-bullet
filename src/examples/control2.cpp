#include <beautiful_bullet/Simulator2.hpp>

#ifdef GRAPHICS
#include <beautiful_bullet/graphics/MagnumGraphics2.hpp>
#endif

#include <control_lib/controllers/Feedback2.hpp>

using namespace beautiful_bullet;
using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
    };

    struct feedback : public defaults::feedback {
    };
};

class OperationSpaceCtr : public control::MultiBodyCtr {
public:
    OperationSpaceCtr() : control::MultiBodyCtr(ControlMode::OPERATIONSPACE)
    {
        // Set controlled frame
        _frame = "lbr_iiwa_link_7";

        // Set gains
        Eigen::MatrixXd K = 10 * Eigen::MatrixXd::Identity(6, 6),
                        D = 2 * Eigen::MatrixXd::Identity(6, 6);

        _controller.setStiffness(K)
            .setDamping(D);

        // Goal
        Eigen::Vector3d xDes(0.365308, -0.0810892, 1.13717);
        Eigen::Matrix3d oDes;
        oDes << 0.591427, -0.62603, 0.508233,
            0.689044, 0.719749, 0.0847368,
            -0.418848, 0.300079, 0.857041;

        _controller.setReference(spatial::SE3(oDes, xDes));
    }

    Eigen::VectorXd action(bodies::MultiBody& body) override
    {
        return _controller.action(spatial::SE3(body.framePose(_frame)));
    }

protected:
    controllers::Feedback2<Params, spatial::SE3> _controller;
};

int main(int argc, char const* argv[])
{
    // Create simulator
    Simulator2 simulator;

// Add graphics
#ifdef GRAPHICS
    simulator.setGraphics(std::make_unique<graphics::MagnumGraphics2>());
#endif

    // Add ground
    simulator.addGround();

    // Rigid Bodies
    bodies::BoxParams boxParams;
    boxParams.setSize(0.2, 0.2, 0.2).setMass(0.1).setFriction(0.5).setColor("red");

    bodies::SphereParams paramsSphere;
    paramsSphere.setRadius(0.2).setMass(0.1).setFriction(0.5).setColor("green");

    bodies::CylinderParams paramsCylinder;
    paramsCylinder.setRadius1(0.2).setRadius2(0.2).setHeight(0.2).setMass(0.1).setFriction(0.5).setColor("blue");

    bodies::RigidBody cube("box", boxParams), sphere("sphere", paramsSphere), cylinder("cylinder", paramsCylinder);

    // Multi Bodies
    bodies::MultiBody iiwaBullet("models/iiwa_bullet/model.urdf"), iiwa("models/iiwa/urdf/iiwa14.urdf");

    Eigen::VectorXd state(7);
    state << 0., 0.7, 0.4, 0.6, 0.3, 0.5, 0.1;
    iiwa;

    // Add bodies to simulation
    simulator.add(
        cube.setPosition(1, 0, 5),
        sphere.setPosition(-1, -0, 5),
        cylinder.setPosition(0, 0, 5).setOrientation(M_PI / 2, 0, 0),
        iiwaBullet.setPosition(0, -1, 0)
            .addControllers(std::make_unique<OperationSpaceCtr>())
            .activateGravity(),
        iiwa.setState(state)
            .setPosition(0, 1, 0)
            .activateGravity());

    // Run simulation
    simulator.run();

    return 0;
}
