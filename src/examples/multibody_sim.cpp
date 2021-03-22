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

    // Create graphics
#ifdef GRAPHICS
    sim.setGraphics(std::make_unique<graphics::MagnumGraphics>());
#endif

    sim.addGround();

    // Agent iiwa(sim, "models/iiwa/model.urdf");

    AgentParams cube_params(2.0, {0., 0., 5.}, {1, 2, 3});
    Agent cube(sim, "box", cube_params);

    // std::cout << temp.size() << std::endl;

    // btCollisionShape* temp = cube.getBody()->getCollisionShape();

    // std::string str(temp->getName());

    // std::cout << str << std::endl;

    // btBoxShape* box_shape = new btBoxShape({1, 2, 3});

    // std::cout << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().x() << " "
    //           << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().y() << " "
    //           << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().z() << std::endl;

    // std::cout << sim.getWorld()->getNumMultibodies() << std::endl;

    sim.run();

    return 0;
}