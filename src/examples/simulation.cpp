#include <iostream>

#include <beautiful_bullet/Simulator.hpp>

#ifdef GRAPHICS
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>
#endif

using namespace beautiful_bullet;

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

    // Objects params
    BoxParams paramsBox;
    paramsBox.setSize(0.5, 0.5, 0.5)
        .setMass(0.1)
        .setFriction(0.5)
        .setColor("red");

    SphereParams paramsSphere;
    paramsSphere.setRadius(0.5)
        .setMass(0.1)
        .setFriction(0.5)
        .setColor("green");

    // Create objects
    Object box("box", paramsBox), sphere("sphere", paramsSphere);

    // Add object to simulator
    simulator.addObjects(box.setPosition(0, 2, 10),
        sphere.setPosition(0, -2, 10));

    // Create agent
    Agent iiwaBullet("models/iiwa_bullet/model.urdf"), iiwa("models/iiwa/urdf/iiwa14.urdf"), franka("models/franka/urdf/panda.urdf");

    // Set agents pose
    iiwaBullet.setPosition(2, -2, 0);
    iiwa.setPosition(2, 2, 0);

    // Set agents state
    Eigen::VectorXd state(7);
    state << 0, 0.2, 0, 0, 0, 0, 0;
    iiwaBullet.setState(state);

    // Add agent to simulator
    simulator.addAgents(iiwaBullet, iiwa, franka);

    // Run simulation
    simulator.run();

    return 0;
}