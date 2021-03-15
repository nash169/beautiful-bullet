#include <iostream>

#include <robot_bullet/Simulator.hpp>

#include <robot_bullet/Agent.hpp>

using namespace robot_bullet;

int main(int argc, char** argv)
{
    Simulator my_sim;

    my_sim.addGround();

    // Agent iiwa(my_sim, "models/iiwa/model.urdf");

    // Cube 1
    Agent cube1(my_sim, "box");

    // Cube 2
    AgentParams cube_params(1.0, {1., 1., 1});
    Agent cube2(my_sim, "box", cube_params);

    return 0;
}