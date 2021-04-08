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
        // using namespace Math::Literals;

        Color4 getColor(const std::string& color)
        {
            if (!color.compare("Red"))
                return Color4::red();
            else if (!color.compare("Green"))
                return Color4::green();
            else if (!color.compare("Blue"))
                return Color4::blue();
            else if (!color.compare("Grey"))
                return Color4(Vector3{128, 128, 128});
            else
                return Color4::red();
        }

        class MagnumGraphics : public AbstractGraphics {
        public:
            MagnumGraphics() : _done(false), _pause(false)
            {
                _identity = new btTransform();
                _identity->setIdentity();
                _temp = new btTransform();
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

                    _app->add("cube", "", Matrix4(Quaternion(orientation).toMatrix()) * Matrix4::translation(Vector3(origin)), 0x585858_rgbf, Matrix4::scaling(Vector3(dimension)));

                    auto motionState = new BulletIntegration::MotionState{*(_app->manipulator().children().last())};
                    sim.getGround()->setMotionState(&motionState->btMotionState());
                }

                // Add agents
                for (auto& agent : sim.getAgents()) {
                    if (agent->getType() & AgentType::MULTIBODY) {
                        btAlignedObjectArray<btQuaternion> world_to_local;
                        btAlignedObjectArray<btVector3> local_origin;

                        agent->getMultiBody().forwardKinematics(world_to_local, local_origin);

                        for (size_t i = 0; i < 2; i++) { // agent->getMultiBody().getNumLinks()
                            // Last object in the manipulator
                            auto* obj_last = _app->manipulator().children().last();

                            // Get array of visual information for link i
                            auto vis = agent->getLinkVisual(i);
                            auto col = agent->getLinkCollision(i);
                            auto link = agent->getLink(i);

                            // Add all the meshes belonging to a body
                            for (size_t j = 0; j < vis.size(); j++) {
                                _app->add(vis[j].m_geometry.m_meshFileName, "", Matrix4(), getColor(vis[j].m_materialName), Matrix4::scaling(Vector3(vis[j].m_geometry.m_meshScale)));
                                // _app->addFrame(Matrix4(col[j].m_linkLocalFrame));
                            }

                            auto it = _mapObject2Frame.insert(std::make_pair(new Object3D(&_app->manipulator()), _identity));

                            if (it.second) {
                                for (Object3D* child = obj_last->nextSibling(); child; child = child->nextSibling())
                                    child->setParent(it.first->first);
                            }

                            // btVector3 temp = agent->getMultiBody().getRVector(0);

                            btTransform tr = link.m_linkTransformInWorld; // agent->getMultiBody().getLink(i).m_cachedWorldTransform; // agent->getMultiBody().getLink(i).m_cachedWorldTransform
                            // tr.setIdentity();
                            // tr.setOrigin(temp);
                            // if (i)
                            _app->addFrame(Matrix4(tr));
                            // if (i == 0)
                            // it.first->first->setTransformation(Matrix4(tr));
                            // else
                            //     it.first->first->setTransformation(Matrix4::translation(Vector3{0.f, 0.f, 0.5f}) * Matrix4(tr)); // Matrix4::rotationY(1.414_radf) *
                        }

                        // Base link

                        // importers::LinkVisual vis = agent->getVisual(0);

                        // auto* obj_last = _app->manipulator().children().last();

                        // // Add all the meshes belonging to a body
                        // for (size_t j = 0; j < vis.getNumMeshes(); j++)
                        //     _app->add(vis.meshes[j]);

                        // auto it = _mapObject2Frame.insert(std::make_pair(new Object3D(&_app->manipulator()), _identity));

                        // if (it.second) {
                        //     for (Object3D* child = obj_last->nextSibling(); child; child = child->nextSibling())
                        //         child->setParent(it.first->first);
                        // }

                        // vis = agent->getVisual(1);

                        // obj_last = _app->manipulator().children().last();

                        // // Add all the meshes belonging to a body
                        // for (size_t j = 0; j < vis.getNumMeshes(); j++)
                        //     _app->add(vis.meshes[j]);

                        // it = _mapObject2Frame.insert(std::make_pair(new Object3D(&_app->manipulator()), _identity));

                        // if (it.second) {
                        //     for (Object3D* child = obj_last->nextSibling(); child; child = child->nextSibling())
                        //         child->setParent(it.first->first);
                        // }

                        // btVector3 temp = agent->getMultiBody().localPosToWorld(0, agent->getMultiBody().getRVector(1));

                        // btTransform tr;
                        // tr.setIdentity();
                        // tr.setOrigin(temp);

                        // // it.first->first->setTransformation(Matrix4(tr));

                        // std::cout << agent->getMultiBody().getBasePos().x() << " "
                        //           << agent->getMultiBody().getBasePos().y() << " "
                        //           << agent->getMultiBody().getBasePos().z() << std::endl;

                        // std::cout << agent->getMultiBody().getLink(0).m_cachedWorldTransform.getOrigin().x() << " "
                        //           << agent->getMultiBody().getLink(0).m_cachedWorldTransform.getOrigin().y() << " "
                        //           << agent->getMultiBody().getLink(0).m_cachedWorldTransform.getOrigin().z() << std::endl;

                        // Other links
                        // for (size_t i = 1; i < 3; i++) {
                        //     importers::LinkVisual vis = agent->getVisual(i);

                        //     auto* obj_last = _app->manipulator().children().last();

                        //     // Add all the meshes belonging to a body
                        //     for (size_t j = 0; j < vis.getNumMeshes(); j++)
                        //         _app->add(vis.meshes[j]);

                        //     *_temp = agent->getMultiBody().getLinkCollider(i - 1)->getWorldTransform();

                        //     auto it = _mapObject2Frame.insert(std::make_pair(new Object3D(&_app->manipulator()),
                        //         _temp));

                        //     if (it.second) {
                        //         for (Object3D* child = obj_last->nextSibling(); child; child = child->nextSibling())
                        //             child->setParent(it.first->first);
                        //     }
                        // }

                        // // _app->addFrame(Matrix4(agent->getMultiBody().getLinkCollider(0)->getWorldTransform()));

                        // _app->addFrame(Matrix4());

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
                // // Apply transformations
                // for (auto& map : _mapObject2Frame)
                //     map.first->setTransformation(Matrix4(*map.second));

                return _app->mainLoopIteration();
            }

        protected:
            bool _done, _pause;

            MagnumApp* _app;

            std::unordered_map<Object3D*, btTransform*> _mapObject2Frame;

            btTransform *_identity, *_temp;
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