#include <iostream>
#include <robot_bullet/Simulator.hpp>
#include <robot_bullet/importers/ImporterURDF.hpp>

using namespace robot_bullet;

int main(int argc, char** argv)
{
    Simulator my_sim();

    importers::ImporterURDF importer;

    if (importer.loadURDF("models/iiwa/model.urdf"))
        std::cout << "hello" << std::endl;

    return 0;
}