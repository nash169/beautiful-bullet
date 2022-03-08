/*
    This file is part of beautiful-bullet.

    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef BEAUTIFULBULLET_GRAPHICS_MAGNUMGRAPHICS_HPP
#define BEAUTIFULBULLET_GRAPHICS_MAGNUMGRAPHICS_HPP

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>

#include <graphics_lib/Graphics.hpp>
#include <graphics_lib/tools/helper.hpp>

#include "beautiful_bullet/Simulator.hpp"
#include "beautiful_bullet/graphics/AbstractGraphics.hpp"

using namespace graphics_lib;

namespace beautiful_bullet {
    namespace graphics {
        class MagnumGraphics : public AbstractGraphics {
        public:
            MagnumGraphics() : AbstractGraphics()
            {
                int argc = 0;
                char** argv = NULL;

                _app = std::unique_ptr<Graphics>(new Graphics({argc, argv}));

                _done = false;
            }

            bool init(Simulator& simulator) override
            {
                // Set camera pose
                _app->camera3D().setPose(Vector3{6., 0., 2.});

                // Add objects to graphics
                for (auto& object : simulator.objects()) {
                    // Object pose
                    btTransform pose;
                    pose.setIdentity();
                    pose.setOrigin(object.body()->getCenterOfMassPosition());
                    pose.setRotation(object.body()->getOrientation());

                    if (object.type() == bodies::BodyType::BOX) {
                        auto motionState = new BulletIntegration::MotionState{
                            _app->addPrimitive("cube")
                                .addPriorTransformation(Matrix4::scaling(Vector3(static_cast<btBoxShape*>(object.body()->getCollisionShape())->getHalfExtentsWithMargin())))
                                .setColor(tools::color(object.params().color))
                                .setTransformation(Matrix4(pose))};

                        // Override motion state
                        object.body()->setMotionState(&motionState->btMotionState());
                    }
                    else if (object.type() == bodies::BodyType::SPHERE) {
                        auto motionState = new BulletIntegration::MotionState{
                            _app->addPrimitive("sphere")
                                .addPriorTransformation(
                                    Matrix4::scaling(
                                        Vector3(
                                            static_cast<bodies::SphereParams&>(object.params()).radius,
                                            static_cast<bodies::SphereParams&>(object.params()).radius,
                                            static_cast<bodies::SphereParams&>(object.params()).radius)))
                                .setColor(tools::color(object.params().color))
                                .setTransformation(Matrix4(pose))};

                        // Override motion state
                        object.body()->setMotionState(&motionState->btMotionState());
                    }
                    else if (object.type() == bodies::BodyType::CYLINDER) {
                        auto motionState = new BulletIntegration::MotionState{
                            _app->addPrimitive("cylinder")
                                .addPriorTransformation(
                                    Matrix4::scaling(
                                        Vector3(
                                            static_cast<bodies::CylinderParams&>(object.params()).radius2,
                                            static_cast<bodies::CylinderParams&>(object.params()).height,
                                            static_cast<bodies::CylinderParams&>(object.params()).radius1)))
                                .setColor(tools::color(object.params().color))
                                .setTransformation(Matrix4(pose))};

                        // Override motion state
                        object.body()->setMotionState(&motionState->btMotionState());
                    }
                    else if (object.type() == bodies::BodyType::CAPSULE) {
                        auto motionState = new BulletIntegration::MotionState{
                            _app->addPrimitive("capsule")
                                .addPriorTransformation(
                                    Matrix4::scaling(
                                        Vector3(
                                            static_cast<bodies::CapsuleParams&>(object.params()).radius,
                                            static_cast<bodies::CapsuleParams&>(object.params()).radius,
                                            static_cast<bodies::CapsuleParams&>(object.params()).height)))
                                .setColor(tools::color(object.params().color))
                                .setTransformation(Matrix4(pose))};

                        // Override motion state
                        object.body()->setMotionState(&motionState->btMotionState());
                    }
                    else
                        std::cerr << "Shape not found." << std::endl;
                }

                // Add agents to graphics
                for (auto& agent : simulator.agents()) {
                    auto model = agent.loader().model();

                    const urdf::Link* root = model->getRoot().get();

                    if (root->name == "world")
                        root = root->child_links[0].get();

                    if (!createVisualRecursive(model.get(), root, agent.body(), agent.loader().path()))
                        return false;
                }

                return true;
            }

            bool refresh() override
            {
                // Apply transformations
                for (auto& map : _mapTransform)
                    map.first->setTransformation(Matrix4(*map.second));

                return _app->mainLoopIteration();
            }

        protected:
            // Magnum-Dynamics application
            std::unique_ptr<Graphics> _app;

            // for multibody
            std::unordered_map<objects::ObjectHandle3D*, btTransform*> _mapTransform;

            bool createVisualRecursive(const urdf::ModelInterface* model, const urdf::Link* node, btMultiBody* multibody, const std::string& path, int index = -1)
            {
                for (auto& visual : node->visual_array) {
                    // Origin
                    btTransform transformVisual = linkFrames(visual.get());

                    // Geometry
                    switch (visual->geometry->type) {
                    case urdf::Geometry::SPHERE: {
                        urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(visual->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("sphere")
                                                                           .addPriorTransformation(Matrix4::scaling(Vector3(sphere->radius, sphere->radius, sphere->radius))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::BOX: {
                        urdf::Box* box = dynamic_cast<urdf::Box*>(visual->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("box")
                                                                           .addPriorTransformation(Matrix4::scaling(Vector3(box->dim.x, box->dim.y, box->dim.z))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::CYLINDER: {
                        urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(visual->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("cylinder")
                                                                           .addPriorTransformation(Matrix4::scaling(Vector3(cylinder->radius, cylinder->radius, cylinder->length))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::MESH: {
                        urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(visual->geometry.get());

                        // add visual frame transformation
                        auto it = _mapTransform.insert(std::make_pair(&_app->import(path + mesh->filename).addPriorTransformation(Matrix4(inertiaFrame(node).inverse()) * Matrix4(transformVisual) * Matrix4::scaling(Vector3(mesh->scale.x, mesh->scale.y, mesh->scale.z))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        if (visual->material)
                            it.first->first->setColor(Color4(visual->material->color.r, visual->material->color.g, visual->material->color.b, visual->material->color.a));

                        break;
                    }
                    default:
                        return false;
                    }
                }

                for (std::size_t i = 0; i < node->child_links.size(); ++i) {
                    if (!createVisualRecursive(model, node->child_links[i].get(), multibody, path, index + 1 + i))
                        return false;
                }

                return true;
            }

            bool createCollisionRecursive(const urdf::ModelInterface* model, const urdf::Link* node, btMultiBody* multibody, const std::string& path, int index = -1)
            {
                for (auto& collision : node->collision_array) {
                    // Collision
                    btTransform transformCollision = linkFrames(collision.get());

                    switch (collision->geometry->type) {
                    case urdf::Geometry::SPHERE: {
                        urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(collision->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("sphere")
                                                                           .addPriorTransformation(Matrix4::scaling(Vector3(sphere->radius, sphere->radius, sphere->radius))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::BOX: {
                        urdf::Box* box = dynamic_cast<urdf::Box*>(collision->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("box")
                                                                           .addPriorTransformation(Matrix4::scaling(Vector3(box->dim.x, box->dim.y, box->dim.z))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::CYLINDER: {
                        urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(collision->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("cylinder")
                                                                           .addPriorTransformation(Matrix4::scaling(Vector3(cylinder->radius, cylinder->radius, cylinder->length))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::MESH: {
                        urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(collision->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->import(path + mesh->filename).addPriorTransformation(Matrix4(inertiaFrame(node).inverse()) * Matrix4(transformCollision) * Matrix4::scaling(Vector3(mesh->scale.x, mesh->scale.y, mesh->scale.z))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    default:
                        return false;
                    }
                }

                for (std::size_t i = 0; i < node->child_links.size(); ++i) {
                    if (!createCollisionRecursive(model, node->child_links[i].get(), multibody, path, index + 1 + i))
                        return false;
                }

                return true;
            }

            template <typename Type>
            btTransform linkFrames(const Type* type)
            {
                btTransform frame;
                frame.setIdentity();
                frame.setOrigin(btVector3(type->origin.position.x, type->origin.position.y, type->origin.position.z));
                frame.setRotation(btQuaternion(type->origin.rotation.x, type->origin.rotation.y, type->origin.rotation.z, type->origin.rotation.w));

                return frame;
            }

            /* Node's parent joint -> inertia frame */
            btTransform inertiaFrame(const urdf::Link* node)
            {
                btTransform frame;
                frame.setIdentity();
                frame.setOrigin(btVector3(node->inertial->origin.position.x, node->inertial->origin.position.y, node->inertial->origin.position.z));
                frame.setRotation(btQuaternion(node->inertial->origin.rotation.x, node->inertial->origin.rotation.y, node->inertial->origin.rotation.z, node->inertial->origin.rotation.w));

                return frame;
            }

            /* Node's parent joint -> visual frame */
            btTransform visualFrame(const urdf::Visual* visual)
            {
                btTransform frame;
                frame.setIdentity();
                frame.setOrigin(btVector3(visual->origin.position.x, visual->origin.position.y, visual->origin.position.z));
                frame.setRotation(btQuaternion(visual->origin.rotation.x, visual->origin.rotation.y, visual->origin.rotation.z, visual->origin.rotation.w));

                return frame;
            }

            /* Parent joint -> joint */
            btTransform jointFrame(const urdf::Joint* joint)
            {
                // Get joint pose
                urdf::Pose pose = joint->parent_to_joint_origin_transform;

                // Bullet transformation from parent to child
                btTransform parentToJoint;
                parentToJoint.setIdentity();
                parentToJoint.setOrigin(btVector3(pose.position.x, pose.position.y, pose.position.z));
                parentToJoint.setRotation(btQuaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w));

                return parentToJoint;
            }

            /* Base -> Node's parent joint */
            void jointFrameWorld(const urdf::Link* node, int index, btTransform& frameWorld)
            {
                frameWorld = jointFrame(node->parent_joint.get()) * frameWorld;
                index--;

                if (index >= 0) {
                    jointFrameWorld(node->getParent().get(), index, frameWorld);
                }
            }
        };
    } // namespace graphics
} // namespace beautiful_bullet

#endif // BEAUTIFULBULLET_GRAPHICS_MAGNUMGRAPHICS_HPP