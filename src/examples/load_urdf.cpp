#include <iostream>

#include <robot_bullet/Simulator.hpp>

#include <robot_bullet/Agent.hpp>

#ifdef GRAPHICS
#include <robot_bullet/graphics/MagnumGraphics.hpp>
#endif

using namespace robot_bullet;

int main(int argc, char** argv)
{
    Simulator sim;

    sim.addGround();

    Agent iiwa(sim, "models/iiwa/model.urdf");

    AgentParams cube_params(2.0, {0., 0., 0.}, {1, 2, 3});
    Agent cube(sim, "box", cube_params);

    btCollisionShape* temp = cube.getAgent()->getCollisionShape();

    std::string str(temp->getName());

    btBoxShape* box_shape = new btBoxShape({1, 2, 3});

    std::cout << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().x() << " "
              << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().y() << " "
              << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().z() << std::endl;

    std::cout << sim.getWorld()->getNumMultibodies() << std::endl;

    return 0;
}