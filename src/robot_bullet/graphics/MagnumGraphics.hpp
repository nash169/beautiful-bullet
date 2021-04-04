#ifndef ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP
#define ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP

#include <iostream>
#include <memory>
#include <string>

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
            else
                return Color4::red();
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

                // Add ground if present
                if (sim.getGround()) {
                    btVector3 dimension = static_cast<btBoxShape*>(sim.getGround()->getCollisionShape())->getHalfExtentsWithMargin(),
                              origin = sim.getGround()->getCenterOfMassPosition();

                    btQuaternion orientation = sim.getGround()->getOrientation();

                    _app->add("cube", "", Matrix4(Quaternion(orientation).toMatrix()) * Matrix4::translation(Vector3(origin)), 0x000ff0_rgbf, Matrix4::scaling(Vector3(dimension)));

                    auto motionState = new BulletIntegration::MotionState{*(_app->manipulator().children().last())};
                    sim.getGround()->setMotionState(&motionState->btMotionState());
                }

                // Add agents
                for (auto& agent : sim.getAgents()) {
                    if (agent->getType() & AgentType::MULTIBODY) {
                        // std::cout << "hello" << std::endl;
                        for (size_t i = 0; i < agent->getMultiBody().getNumLinks(); i++) {
                            // Visual info
                            importers::LinkVisual vis = agent->getVisual(i);

                            auto* obj_last = _app->manipulator().children().last();

                            // Add all the meshes belonging to a body
                            for (size_t j = 0; j < vis.getNumMeshes(); j++)
                                _app->add(vis.meshes[j]);

                            auto it = _mapObject2Frame.insert(std::make_pair(new Object3D(&_app->manipulator()),
                                &agent->getMultiBody().getLinkCollider(i)->getWorldTransform()));

                            if (it.second) {
                                for (Object3D* child = obj_last->nextSibling(); child; child = child->nextSibling())
                                    child->setParent(it.first->first);
                            }
                        }

                        // // Apply transformations
                        // for (auto& map : _mapObject2Frame)
                        //     map.first->setTransformation(Matrix4(*map.second));
                    }
                    else if (agent->getType() & AgentType::RIGIDBODY) {
                        if (agent->getType() & AgentType::BOX) {
                            btVector3 dimension = static_cast<btBoxShape*>(agent->getRigidBody().getCollisionShape())->getHalfExtentsWithMargin(),
                                      origin = agent->getRigidBody().getCenterOfMassPosition();

                            btQuaternion orientation = agent->getRigidBody().getOrientation();

                            _app->add("cube", "", Matrix4(Quaternion(orientation).toMatrix()) * Matrix4::translation(Vector3(origin)), getColor(agent->getParams().material), Matrix4::scaling(Vector3(dimension)));
                        }
                        else if (agent->getType() & AgentType::SPHERE) {
                        }

                        auto motionState = new BulletIntegration::MotionState{*(_app->manipulator().children().last())};
                        agent->getRigidBody().setMotionState(&motionState->btMotionState());
                    }
                }
            }

            bool
            done() override
            {
                return _done;
            }

            bool pause() override
            {
                return _pause;
            }

            bool refresh() override
            {
                // Apply transformations
                for (auto& map : _mapObject2Frame)
                    map.first->setTransformation(Matrix4(*map.second));

                return _app->mainLoopIteration();
            }

        protected:
            bool _done, _pause;

            MagnumApp* _app;

            std::unordered_map<Object3D*, btTransform*> _mapObject2Frame;
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