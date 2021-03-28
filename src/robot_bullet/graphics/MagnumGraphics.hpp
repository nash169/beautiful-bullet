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

                    auto motionState = new BulletIntegration::MotionState{*(_app->getObjects().back())};
                    sim.getGround()->setMotionState(&motionState->btMotionState());
                }

                // Add agents
                for (auto& agent : sim.getAgents()) {
                    if (agent->getType() & AgentType::MULTIBODY) {
                        // // Body transformations
                        // std::vector<btTransform> tr = agent->getLinkPos2();

                        // Visual meshes
                        std::vector<importers::LinkVisual> vis = agent->getVisual();

                        for (size_t i = 0; i < vis.size(); i++) {
                            // Store number of meshes before adding new element
                            int num_obj = _app->getNumObjects();

                            // Add all the meshes belonging to a body
                            for (size_t j = 0; j < vis[i].getNumMeshes(); j++) {
                                _app->add(vis[i].meshes[j]);
                                // _app->add(vis[i].meshes[j], "", Matrix4(tr[i]));
                            }
                            std::vector<Object3D*> int_vec = _app->getObjects();
                            std::vector<Object3D*> sub_vec = {int_vec.begin() + num_obj, int_vec.begin() + _app->getNumObjects()};
                            _multibody_map[vis[i].id] = sub_vec;
                        }

                        // Store transformation
                        std::unordered_map<std::string, btTransform*> agent_tr = agent->getMapTransform();
                        _multibody_transform.insert(agent_tr.begin(), agent_tr.end());

                        // Apply transformations
                        for (auto& map : _multibody_map)
                            for (auto& vec : map.second)
                                vec->setTransformation(Matrix4(*_multibody_transform[map.first]));
                    }
                    else if (agent->getType() & AgentType::RIGIDBODY) {
                        if (agent->getType() & AgentType::BOX) {
                            btVector3 dimension = static_cast<btBoxShape*>(agent->getBody()->getCollisionShape())->getHalfExtentsWithMargin(),
                                      origin = agent->getBody()->getCenterOfMassPosition();

                            btQuaternion orientation = agent->getBody()->getOrientation();

                            _app->add("cube", "", Matrix4(Quaternion(orientation).toMatrix()) * Matrix4::translation(Vector3(origin)), getColor(agent->getParams().material), Matrix4::scaling(Vector3(dimension)));
                        }
                        else if (agent->getType() & AgentType::SPHERE) {
                        }

                        auto motionState = new BulletIntegration::MotionState{*(_app->getObjects().back())};
                        agent->getBody()->setMotionState(&motionState->btMotionState());
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

            std::unordered_map<std::string, std::vector<Object3D*>> _multibody_map;
            std::unordered_map<std::string, btTransform*> _multibody_transform;
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