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

                /* Set params */
                _params = std::make_shared<BoxParams>(static_cast<const BoxParams&>(params));

                /* Create box shape */
                btBoxShape* boxShape = new btBoxShape(static_cast<const BoxParams&>(params).size);

                /* Create box rigid body */
                _body = createRigidBody(params.mass, params.transform, boxShape);

                // Set friction
                _body->setFriction(params.friction);
            }
            else {
                btCollisionShape* childShape;

                if (!shape.compare("sphere")) {
                    /* Set sphere type */
                    _type = ObjectType::SPHERE;

                    /* Set params */
                    _params = std::make_shared<SphereParams>(static_cast<const SphereParams&>(params));

                    /* Create box shape */
                    childShape = new btSphereShape(static_cast<const SphereParams&>(params).radius);
                }
                else if (!shape.compare("cylinder")) {
                    /* Set cylinder type */
                    _type = ObjectType::CYLINDER;

                    /* Set params */
                    _params = std::make_shared<CylinderParams>(static_cast<const CylinderParams&>(params));

                    /* Create box shape */
                    btVector3 halfExtents(static_cast<const CylinderParams&>(params).radius1, static_cast<const CylinderParams&>(params).radius2, static_cast<const CylinderParams&>(params).height);
                    childShape = new btCylinderShape(halfExtents);
                }
                else if (!shape.compare("capsule")) {
                    /* Set cylinder type */
                    _type = ObjectType::CAPSULE;

                    /* Set params */
                    _params = std::make_shared<CapsuleParams>(static_cast<const CapsuleParams&>(params));

                    /* Create box shape */
                    childShape = new btCapsuleShape(static_cast<const CapsuleParams&>(params).radius, static_cast<const CapsuleParams&>(params).height);
                }
                else {
                    std::cerr << "Shape not available." << std::endl;
                    return;
                }

                /* Create compound shape */
                btCompoundShape* colShape = new btCompoundShape();
                colShape->addChildShape(btTransform::getIdentity(), childShape);

                /* Dynamic properties */
                btVector3 localInertia(0, 0, 0);
                if (params.mass)
                    colShape->calculateLocalInertia(params.mass, localInertia);

                /* Add rigid body */
                _body = createRigidBody(params.mass, params.transform, colShape);

                // Set friction
                _body->setFriction(params.friction);
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

        /* Get object params */
        std::shared_ptr<ObjectParams> params() { return _params; }
        // ObjectParams* params() { return _params; }

        /* Get type */
        const ObjectType& type() const { return _type; }

        /* Get body pointer */
        btRigidBody* body() { return _body; }

        /* Set pose */
        Object& setPose(const double& x, const double& y, const double& z)
        {
            btTransform transformation;
            transformation.setIdentity();
            transformation.setOrigin(btVector3(x, y, z));
            _body->setCenterOfMassTransform(transformation);

            return *this;
        }

        /* Set Orientation */
        Object& setOrientation() // here decide what to pass
        {
            return *this;
        }

        /* Add controllers */
        template <typename... Args>
        Object& addControllers(std::shared_ptr<Control> controller, Args... args)
        {
            // Add controller
            _controllers.push_back(controller);

            // Init controller
            _controllers.back()->init();

            if constexpr (sizeof...(args) > 0)
                addControllers(args...);

            return *this;
        }

        // Update model
        void update() {}

    protected:
        // Bullet rigid body
        btRigidBody* _body = nullptr;

        // Object type
        ObjectType _type;

        // Object params (still shared for problems in moving the object with unique_ptr)
        // Pointer in order to avoid object slicing
        // https://stackoverflow.com/questions/8777724/store-derived-class-objects-in-base-class-variables
        // https://stackoverflow.com/questions/1541031/is-it-possible-for-slicing-to-occur-with-smart-pointers
        std::shared_ptr<ObjectParams> _params;

        // Controllers (for now shared because of issues in moving the agent object)
        std::vector<std::shared_ptr<Control>> _controllers;

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