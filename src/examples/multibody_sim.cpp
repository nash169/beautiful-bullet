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

    Agent iiwa(sim, "models/iiwa/model.urdf");

    // AgentParams cube_params(1.0, "green", {0., 0., 5.}, {0.5, 0.5, 0.5});
    // AgentParams cube_params2(1.0, "blue", {0., 0, 15.}, {1., 1., 1.});

    // Agent cube(sim, "box", cube_params);
    // Agent cube2(sim, "box", cube_params2);

    // std::cout << temp.size() << std::endl;

    // btCollisionShape* temp = cube.getBody()->getCollisionShape();

    // std::string str(temp->getName());

    // std::cout << str << std::endl;

    // btBoxShape* box_shape = new btBoxShape({1, 2, 3});

    // std::cout << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().x() << " "
    //           << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().y() << " "
    //           << static_cast<btBoxShape*>(temp)->getHalfExtentsWithMargin().z() << std::endl;

    // std::cout << sim.getWorld()->getNumMultibodies() << std::endl;

    // for (size_t i = 0; i < iwis.getMultiBody()->getNumLinks(); i++) {
    //     std::cout << "Link " << i << ": " << iwis.getMultiBody()->getRVector(i).x() << " "
    //               << iwis.getMultiBody()->getRVector(i).y() << " "
    //               << iwis.getMultiBody()->getRVector(i).z() << std::endl;
    // }

    // std::cout << "Link " << 5 << ": " << iwis.getMultiBody()->getBasePos().x() << " "
    //           << iwis.getMultiBody()->getBasePos().y() << " "
    //           << iwis.getMultiBody()->getBasePos().z() << std::endl;

    // btInverseDynamicsBullet3::mat33* world_T_body = new btInverseDynamicsBullet3::mat33();
    // iwis.update();
    // std::cout << iwis.getInverseModel()->getBodyTransform(0, world_T_body) << std::endl;
    // btTransform temp(*static_cast<btMatrix3x3*>(world_T_body));

    sim.run();

    return 0;
}