#ifndef ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP
#define ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP

#include <iostream>
#include <memory>

#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <magnum_dynamics/MagnumApp.hpp>

#include "robot_bullet/Simulator.hpp"
#include "robot_bullet/graphics/AbstractGraphics.hpp"

namespace robot_bullet {
    namespace graphics {
        using namespace magnum_dynamics;

        class MagnumGraphics : public AbstractGraphics {
        public:
            MagnumGraphics() : _done(false), _pause(false)
            {
            }

            ~MagnumGraphics()
            {
                delete _app;
            }

            void init(Simulator& sim) override
            {
                int argc = 0;
                char** argv = NULL;

                _app = new MagnumApp({argc, argv});

                // Create ground if present
                if (sim.getGround()) {
                    btVector3 dimension = static_cast<btBoxShape*>(sim.getGround()->getCollisionShape())->getHalfExtentsWithMargin();
                    std::cout << dimension.x() << " "
                              << dimension.y() << " "
                              << dimension.z() << std::endl;
                    _app->add("cube", "", Matrix4::scaling({dimension.x(), dimension.y(), dimension.z()}), 0xffffff_rgbf);

                    auto motionState = new BulletIntegration::MotionState{*(_app->getObjects().back())};
                    sim.getGround()->setMotionState(&motionState->btMotionState());
                    sim.getGround()->setWorldTransform(btTransform(_app->getObjects().back()->transformationMatrix()));
                }

                for (auto& agent : sim.getAgents()) {
                    if (agent->getType() & AgentType::MULTIBODY) {
                    }
                    else if (agent->getType() & AgentType::RIGIDBODY) {
                        if (agent->getType() & AgentType::BOX) {
                            btVector3 dimension = static_cast<btBoxShape*>(agent->getBody()->getCollisionShape())->getHalfExtentsWithMargin(),
                                      origin = agent->getBody()->getCenterOfMassPosition();

                            btQuaternion orientation = agent->getBody()->getOrientation();

                            // _app->add("cube", "", Matrix4::scaling({2.0f, 1.0, 1.0f}), 0xff0000_rgbf);
                            _app->add("cube", "", Matrix4::scaling({2.0f, 1.0, 1.0f}), 0x00ff00_rgbf);
                        }
                        else if (agent->getType() & AgentType::SPHERE) {
                        }

                        auto motionState = new BulletIntegration::MotionState{*(_app->getObjects().back())};
                        // agent->getBody()->setWorldTransform(btTransform(_app->getObjects().back()->transformationMatrix()));
                        // agent->getBody()->setMotionState(&motionState->btMotionState());
                    }
                }
            }

            bool done() override
            {
                return _done;
            }

            bool pause() override
            {
                return _pause;
            }

            bool refresh() override
            {
                return _app->mainLoopIteration();
            }

        protected:
            bool _done, _pause;

            MagnumApp* _app;
        };
    } // namespace graphics
} // namespace robot_bullet

#endif // ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP

// for (size_t i = 0; i < sim.getWorld()->getNonStaticRigidBodies().size(); i++) {
//     std::string shape_type(sim.getWorld()->getNonStaticRigidBodies()[i]->getCollisionShape()->getName());

//     if (!shape_type.compare("Box")) {

//         btVector3 dimension = static_cast<btBoxShape*>(sim.getWorld()->getNonStaticRigidBodies()[i]->getCollisionShape())->getHalfExtentsWithMargin();

//         _app->add("cube", "", Matrix4::scaling(Vector3{0.5f}), 0xff0000_rgbf);
//     }
//     else if (!shape_type.compare("Sphere")) {
//     }

//     auto motionState = new BulletIntegration::MotionState{*(_app->getObjects().back())};
//     sim.getWorld()->getNonStaticRigidBodies()[i]->setMotionState(&motionState->btMotionState());
// }