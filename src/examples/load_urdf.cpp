#include <iostream>

#include <robot_bullet/Simulator.hpp>

#include <robot_bullet/Agent.hpp>

using namespace robot_bullet;

int main(int argc, char** argv)
{
    Simulator my_sim;

    Agent iiwa(my_sim, "models/iiwa/model.urdf");

    return 0;
}