#ifndef ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP
#define ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

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
            else if (!color.compare("grey"))
                return Color4(0x585858_rgbf);
            else
                return Color4::red();
        }

        class MagnumGraphics : public AbstractGraphics {
        public:
            MagnumGraphics() : AbstractGraphics()
            {
                int argc = 0;
                char** argv = NULL;

                _app = std::unique_ptr<MagnumApp>(new MagnumApp({argc, argv}));

                _done = false;
            }

            bool init(Simulator& simulator) override
            {
                // Set camera pose
                _app->camera()->setPose({10., 0., 5.});

                // Add objects to graphics
                for (auto& object : simulator.objects()) {
                    if (object.type() == ObjectType::BOX) {
                        btTransform pose;
                        pose.setIdentity();
                        pose.setOrigin(object.body()->getCenterOfMassPosition());
                        pose.setRotation(object.body()->getOrientation());

                        // Override motion state
                        auto motionState = new BulletIntegration::MotionState{
                            _app->addPrimitive("cube")
                                .setPrimitiveTransformation(Matrix4::scaling(Vector3(static_cast<btBoxShape*>(object.body()->getCollisionShape())->getHalfExtentsWithMargin())))
                                .setTransformation(Matrix4(pose))
                                .setColor(getColor(object.color()))};

                        object.body()->setMotionState(&motionState->btMotionState());
                    }
                }

                // Add agents to graphics
                for (auto& agent : simulator.agents()) {
                    auto model = agent.loader()->model();

                    const urdf::Link* root = model->getRoot().get();

                    if (!createVisualRecursive(model.get(), root, agent.body(), agent.loader()->path()))
                        return false;

                    // // Apply transformations
                    // for (auto& map : _mapTransform)
                    //     map.first->setTransformation(Matrix4(*map.second));
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
            std::unique_ptr<MagnumApp> _app;

            // for multibody
            std::unordered_map<magnum_dynamics::Object*, btTransform*> _mapTransform;

            bool createVisualRecursive(const urdf::ModelInterface* model, const urdf::Link* node, btMultiBody* multibody, const std::string& path, int index = -1)
            {
                for (auto& visual : node->visual_array) {
                    // Origin
                    btTransform transform;
                    transform.setIdentity();
                    transform.setOrigin(btVector3(visual->origin.position.x, visual->origin.position.y, visual->origin.position.z));
                    transform.setRotation(btQuaternion(visual->origin.rotation.x, visual->origin.rotation.y, visual->origin.rotation.z, visual->origin.rotation.w));

                    // Geometry
                    switch (visual->geometry->type) {
                    case urdf::Geometry::SPHERE: {
                        urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(visual->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("sphere")
                                                                           .setPrimitiveTransformation(Matrix4::scaling(Vector3(sphere->radius, sphere->radius, sphere->radius))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::BOX: {
                        urdf::Box* box = dynamic_cast<urdf::Box*>(visual->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("box")
                                                                           .setPrimitiveTransformation(Matrix4::scaling(Vector3(box->dim.x, box->dim.y, box->dim.z))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::CYLINDER: {
                        urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(visual->geometry.get());

                        auto it = _mapTransform.insert(std::make_pair(&_app->addPrimitive("cylinder")
                                                                           .setPrimitiveTransformation(Matrix4::scaling(Vector3(cylinder->radius, cylinder->radius, cylinder->length))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        break;
                    }
                    case urdf::Geometry::MESH: {
                        urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(visual->geometry.get());

                        // add visual frame transformation
                        auto it = _mapTransform.insert(std::make_pair(&_app->import(path + mesh->filename)
                                                                           .setPrimitiveTransformation(Matrix4(inertiaFrame(node).inverse()) * Matrix4::scaling(Vector3(mesh->scale.x, mesh->scale.y, mesh->scale.z))),
                            (index == -1) ? &multibody->getBaseCollider()->getWorldTransform() : &multibody->getLinkCollider(index)->getWorldTransform()));

                        // if (index >= 0) {
                        //     btTransform temp = multibody->getLinkCollider(index)->getWorldTransform() * inertiaFrame(node).inverse();
                        //     std::cout << "LINK GET: " << index << std::endl;
                        //     std::cout << temp.getOrigin().x() << " " << temp.getOrigin().y() << " " << temp.getOrigin().z() << std::endl;
                        //     std::cout << temp.getRotation().x() << " " << temp.getRotation().y() << " " << temp.getRotation().z() << " " << temp.getRotation().w() << std::endl;
                        // }
                        // else {
                        //     btTransform temp = multibody->getBaseCollider()->getWorldTransform() * inertiaFrame(node).inverse();
                        //     std::cout << "LINK GET: " << index << std::endl;
                        //     std::cout << temp.getOrigin().x() << " " << temp.getOrigin().y() << " " << temp.getOrigin().z() << std::endl;
                        //     std::cout << temp.getRotation().x() << " " << temp.getRotation().y() << " " << temp.getRotation().z() << " " << temp.getRotation().w() << std::endl;
                        // }
                        break;
                    }
                    default:
                        return false;
                    }

                    // Material
                    // it.first->first->setColor(Color4(visual->material->color.r, visual->material->color.g, visual->material->color.b, visual->material->color.a));
                }

                for (std::size_t i = 0; i < node->child_links.size(); ++i) {
                    if (!createVisualRecursive(model, node->child_links[0].get(), multibody, path, index + 1 + i))
                        return false;
                }

                return true;
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
        };
    } // namespace graphics
} // namespace robot_bullet

#endif // ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP