#ifndef BEAUTIFUL_BULLET_OBJECT_HPP
#define BEAUTIFUL_BULLET_OBJECT_HPP

#include <memory>

#include "beautiful_bullet/Control.hpp"
#include "beautiful_bullet/Types.hpp"

#include <BulletCollision/btBulletCollisionCommon.h>

namespace beautiful_bullet {
    class Object {
    public:
        // Constructor
        Object(const std::string& shape, const ObjectParams& params)
        {
            if (!shape.compare("box")) {
                /* Set box type */
                _type = ObjectType::BOX;

                /* Set box color */
                _color = params.color; // memorize only for graphics (not very clean)

                /* Create box shape */
                btBoxShape* box_shape = new btBoxShape(static_cast<const BoxParams&>(params).size);

                /* Create box rigid body */
                _body = createRigidBody(params.mass, params.transform, box_shape);

                // Set friction
                _body->setFriction(params.friction);
            }
            else if (!shape.compare("sphere")) {
                /* Set sphere type */
                _type = ObjectType::SPHERE;
            }
            else {
                std::cerr << "Shape not available." << std::endl;
                return;
            }
        }

        Object() = default;

        // No copy constructor (check explicit and default)
        // Object(const Object&) = delete; // it might not work just because I'm with clang

        // ~Object()
        // {
        //     for (auto& controller : _controllers)
        //         delete controller;
        // }

        // Get type
        const ObjectType& type() const { return _type; }

        // Get type
        const std::string& color() const { return _color; }

        // Get body pointer
        btRigidBody* body() { return _body; }

        // // Add controllers
        // template <typename... Args>
        // Object& addControllers(Control* controller, Args... args)
        // {
        //     _controllers.push_back(controller);

        //     if constexpr (sizeof...(args) > 0)
        //         addControllers(args...);

        //     return *this;
        // }

        // Update model
        void update() {}

    protected:
        // Bullet rigid body
        btRigidBody* _body = nullptr;

        // Object type
        ObjectType _type;

        // Object color
        std::string _color;

        // // Controllers
        // std::vector<Control*> _controllers;

        // Create Rigid Body
        btRigidBody* createRigidBody(const btScalar& mass, const btTransform& transform, btCollisionShape* shape)
        {
            // Assert shape
            btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.f);

            // Calculate inertia
            btVector3 localInertia(0, 0, 0);
            if (isDynamic)
                shape->calculateLocalInertia(mass, localInertia);

            // using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
            btDefaultMotionState* myMotionState = new btDefaultMotionState(transform);

            // rigid body basic information
            btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

            // create rigid body
            btRigidBody* body = new btRigidBody(cInfo);

            return body;
        }
    }; // namespace robot_raisim
} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_OBJECT_HPP