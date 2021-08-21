#ifndef ROBOT_BULLET_OBJECT_HPP
#define ROBOT_BULLET_OBJECT_HPP

#include <memory>

#include "robot_bullet/Types.hpp"

#include <BulletCollision/btBulletCollisionCommon.h>

namespace robot_bullet {
    class Object {
    public:
        // Constructor
        Object(const std::string& shape, const ObjectParams& params)
        {
            if (!shape.compare("box")) {
                /* Set box type */
                _type = ObjectType::BOX;

                /* Set box color */
                _color = params.color;

                /* Create box shape */
                btBoxShape* box_shape = new btBoxShape(static_cast<const BoxParams&>(params).size);
                // simulator.getCollisionShapes().push_back(box_shape);

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

        // Get type
        const ObjectType& type() const { return _type; }

        // Get type
        const std::string& color() const { return _color; }

        // Get body pointer
        btRigidBody* body() { return _body; }

        // Update model
        void update() {}

    protected:
        // Bullet rigid body
        btRigidBody* _body = nullptr;

        // Object type
        ObjectType _type;

        // Object color
        std::string _color;

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
} // namespace robot_bullet

#endif // ROBOT_BULLET_OBJECT_HPP