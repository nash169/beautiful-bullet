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
    BoxParams params;
    params.setSize(0.5, 0.5, 0.5)
        .setMass(0.1)
        // .setFriction(0.5)
        .setPose(Eigen::Vector3d(0, 2, 10))
        .setColor("red");

    Object cube("box", params);

    // Add object to simulator
    simulator.addObjects(std::move(cube));

    // Create agent
    Agent iiwa("models/iiwa/model.urdf");

    // Add agent to simulator
    simulator.addAgents(std::move(iiwa));

    // Run simulation
    simulator.run();

    return 0;
}