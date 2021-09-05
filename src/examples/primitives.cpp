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
    params1.setSize(0.5, 0.5, 0.5) // half sides
        .setMass(0.1)
        .setFriction(0.5)
        .setPose(Eigen::Vector3d(0, 2, 10))
        .setColor("red");

    SphereParams paramsSphere;
    paramsSphere.setRadius(0.5)
        .setMass(0.1)
        .setFriction(0.5)
        .setPose(Eigen::Vector3d(0, -2, 2))
        .setColor("green");

    // params2 = params1;
    // params2.setPose(Eigen::Vector3d(0, -2, 0.5))
    //     .setColor("green");

    Object cube("box", params1), sphere("sphere", paramsSphere);

    // Add object to simulator
    simulator.addObjects(cube, sphere);

    // Run simulation
    simulator.run();

    return 0;
}