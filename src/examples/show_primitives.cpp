/*
    This file is part of beautiful-bullet.

    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

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

    // Rigid Bodies Params
    bodies::BoxParams boxParams;
    boxParams.setSize(0.5, 0.5, 0.5).setMass(0.1).setFriction(0.5).setColor("red");

    bodies::SphereParams paramsSphere;
    paramsSphere.setRadius(0.5).setMass(0.1).setFriction(0.5).setColor("green");

    bodies::CylinderParams paramsCylinder;
    paramsCylinder.setRadius1(0.5).setRadius2(0.5).setHeight(0.5).setMass(0.1).setFriction(0.5).setColor("blue");

    bodies::CapsuleParams paramsCapsule;
    paramsCapsule.setRadius(0.5).setHeight(0.5).setMass(0.1).setFriction(0.5).setColor("yellow");

    // Rigid Bodies
    bodies::RigidBodyPtr cube = std::make_shared<bodies::RigidBody>("box", boxParams),
                         sphere = std::make_shared<bodies::RigidBody>("sphere", paramsSphere),
                         cylinder = std::make_shared<bodies::RigidBody>("cylinder", paramsCylinder),
                         capsule = std::make_shared<bodies::RigidBody>("capsule", paramsCapsule);

    cube->setPosition(0, 2, 3);
    sphere->setPosition(0, -2, 3);
    cylinder->setPosition(0, 0, 0.5)
        .setOrientation(M_PI / 2, 0, 0);
    capsule->setPosition(-2, 0, 2)
        .setOrientation(0, M_PI / 2, 0);

    // Add object to simulator
    simulator.add(cube, sphere, cylinder, capsule);

    // Run simulation
    simulator.run();

    return 0;
}