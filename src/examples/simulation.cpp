#include <iostream>

#include <robot_bullet/Simulator.hpp>

#ifdef GRAPHICS
#include <robot_bullet/graphics/MagnumGraphics.hpp>
#endif

using namespace robot_bullet;

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
        // .setFriction(0.5)
        .setPose(Eigen::Vector3d(0, 2, 10))
        .setColor("red");

    params2 = params1;
    params2.setPose(Eigen::Vector3d(0, -2, 10))
        .setColor("green");

    Object cube1("box", params1), cube2("box", params2);

    // Add object to simulator
    simulator.addObjects(cube1, cube2);

    // Create agent
    Agent iiwaBullet("models/iiwa_bullet/model.urdf"),
        iiwa("models/iiwa/urdf/iiwa14.urdf"),
        franka("models/franka/urdf/panda.urdf");

    iiwaBullet.setPose(2, -2, 0);
    iiwa.setPose(2, 2, 0);

    // Add agent to simulator
    simulator.addAgents(iiwaBullet, iiwa, franka);

    // Run simulation
    simulator.run();

    return 0;
}