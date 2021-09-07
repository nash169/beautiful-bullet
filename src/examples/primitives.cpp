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

    // Objects' params
    BoxParams boxParams;
    boxParams.setSize(0.5, 0.5, 0.5).setMass(0.1).setFriction(0.5).setColor("red");

    SphereParams paramsSphere;
    paramsSphere.setRadius(0.5).setMass(0.1).setFriction(0.5).setColor("green");

    CylinderParams paramsCylinder;
    paramsCylinder.setRadius1(0.5).setRadius2(0.5).setHeight(0.5).setMass(0.1).setFriction(0.5).setColor("blue");

    CapsuleParams paramsCapsule;
    paramsCapsule.setRadius(0.5).setHeight(0.5).setMass(0.1).setFriction(0.5).setColor("yellow");

    // Create objects
    Object cube("box", boxParams),
        sphere("sphere", paramsSphere),
        cylinder("cylinder", paramsCylinder),
        capsule("capsule", paramsCapsule);

    // Add object to simulator
    simulator.addObjects(
        cube.setPosition(0, 2, 3),
        sphere.setPosition(0, -2, 3),
        cylinder.setPosition(0, 0, 0.5).setOrientation(M_PI / 2, 0, 0),
        capsule.setPosition(-2, 0, 2).setOrientation(0, M_PI / 2, 0));

    // Run simulation
    simulator.run();

    return 0;
}