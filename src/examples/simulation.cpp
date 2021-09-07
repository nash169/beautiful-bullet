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

    // Create object
    BoxParams params1, params2;
    params1.setSize(0.5, 0.5, 0.5)
        .setMass(0.1)
        .setFriction(0.5)
        .setColor("red");

    params2 = params1;
    params2.setColor("green");

    Object cube1("box", params1), cube2("box", params2);

    // Add object to simulator
    simulator.addObjects(cube1.setPosition(0, 2, 10),
        cube2.setPosition(0, -2, 10));

    // Create agent
    Agent iiwaBullet("models/iiwa_bullet/model.urdf"), iiwa("models/iiwa/urdf/iiwa14.urdf"),
        franka("models/franka/urdf/panda.urdf");

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