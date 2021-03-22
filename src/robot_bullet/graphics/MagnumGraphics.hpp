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

        Color4 getColor(const std::string& color)
        {
            if (!color.compare("red"))
                return Color4::red();
            else if (!color.compare("green"))
                return Color4::green();
            else if (!color.compare("blue"))
                return Color4::blue();
        }

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
                    btVector3 dimension = static_cast<btBoxShape*>(sim.getGround()->getCollisionShape())->getHalfExtentsWithMargin(),
                              origin = sim.getGround()->getCenterOfMassPosition();

                    btQuaternion orientation = sim.getGround()->getOrientation();

                    _app->add("cube", "", Matrix4(Quaternion(orientation).toMatrix()) * Matrix4::translation(Vector3(origin)) * Matrix4::scaling(Vector3(dimension)), 0xffffff_rgbf);

                    auto motionState = new BulletIntegration::MotionState{*(_app->getObjects().back())};
                    sim.getGround()->setMotionState(&motionState->btMotionState());
                    // sim.getGround()->setCollisionFlags(sim.getGround()->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
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

                            _app->add("cube", "", Matrix4(Quaternion(orientation).toMatrix()) * Matrix4::translation(Vector3(origin)), getColor(agent->getParams().material));
                        }
                        else if (agent->getType() & AgentType::SPHERE) {
                        }

                        auto motionState = new BulletIntegration::MotionState{*(_app->getObjects().back())};
                        agent->getBody()->setMotionState(&motionState->btMotionState());

                        // This should not be called
                        // agent->getBody()->setCollisionFlags(agent->getBody()->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);

                        // agent->getBody()->setWorldTransform(btTransform(_app->getObjects().back()->transformationMatrix()));
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