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

    // RigidBodies params
    bodies::BoxParams paramsBox;
    paramsBox.setSize(0.5, 0.5, 0.5).setMass(0.1).setFriction(0.5).setColor("red");

    bodies::SphereParams paramsSphere;
    paramsSphere.setRadius(0.5).setMass(0.1).setFriction(0.5).setColor("green");

    // Create RigidBodies
    bodies::RigidBodyPtr box = std::make_shared<bodies::RigidBody>("box", paramsBox),
                         sphere = std::make_shared<bodies::RigidBody>("sphere", paramsSphere);

    box->setPosition(0, 2, 10);
    sphere->setPosition(0, -2, 10);

    // MultiBodies params
    Eigen::VectorXd state(7);
    state << 0, 0.2, 0.1, 0.2, 0.3, 0.7, 0.2;

    // Create agent
    bodies::MultiBodyPtr iiwaBullet = std::make_shared<bodies::MultiBody>("models/iiwa_bullet/model.urdf"),
                         iiwa = std::make_shared<bodies::MultiBody>("models/iiwa/urdf/iiwa14.urdf"),
                         franka = std::make_shared<bodies::MultiBody>("models/franka/panda.urdf");

    iiwaBullet->setPosition(2, -2, 0);
    iiwa->setPosition(2, 2, 0);


    franka->activateGravity().setState(state);

    // Add objects to simulator
    simulator.add(box, sphere, iiwaBullet, iiwa, franka);

    // Run simulation
    simulator.run();

    return 0;
}