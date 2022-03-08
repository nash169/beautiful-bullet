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

#include "beautiful_bullet/bodies/RigidBody.hpp"

namespace beautiful_bullet {
    namespace bodies {
        // Constructor
        RigidBody::RigidBody(const std::string& shape, const BodyParams& params)
        {
            if (!shape.compare("box")) {
                /* Set box type */
                _type = BodyType::BOX;

                /* Set params */
                _params = std::make_unique<BoxParams>(static_cast<const BoxParams&>(params));

                /* Create box shape */
                btBoxShape* boxShape = new btBoxShape(static_cast<const BoxParams&>(params).size);

                /* Create box rigid body */
                _body = createRigidBody(params.mass, btTransform::getIdentity(), boxShape);

                // Set friction
                _body->setFriction(params.friction);
            }
            else {
                btCollisionShape* childShape;

                if (!shape.compare("sphere")) {
                    /* Set sphere type */
                    _type = BodyType::SPHERE;

                    /* Set params */
                    _params = std::make_unique<SphereParams>(static_cast<const SphereParams&>(params));

                    /* Create box shape */
                    childShape = new btSphereShape(static_cast<const SphereParams&>(params).radius);
                }
                else if (!shape.compare("cylinder")) {
                    /* Set cylinder type */
                    _type = BodyType::CYLINDER;

                    /* Set params */
                    _params = std::make_unique<CylinderParams>(static_cast<const CylinderParams&>(params));

                    /* Create box shape */
                    btVector3 halfExtents(static_cast<const CylinderParams&>(params).radius1, static_cast<const CylinderParams&>(params).radius2, static_cast<const CylinderParams&>(params).height);
                    childShape = new btCylinderShape(halfExtents);
                }
                else if (!shape.compare("capsule")) {
                    /* Set cylinder type */
                    _type = BodyType::CAPSULE;

                    /* Set params */
                    _params = std::make_unique<CapsuleParams>(static_cast<const CapsuleParams&>(params));

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

                // /* Dynamic properties */
                // btVector3 localInertia(0, 0, 0);
                // if (params.mass)
                //     colShape->calculateLocalInertia(params.mass, localInertia);

                /* Add rigid body */
                _body = createRigidBody(params.mass, btTransform::getIdentity(), colShape);

                // Set friction
                _body->setFriction(params.friction);
            }

            // Default no gravity compensation
            _gravity = false;
        }

        // Move constructor
        RigidBody::RigidBody(RigidBody&& other) noexcept
        {
            // Move RigidBody pointer
            _body = other._body;
            other._body = nullptr;

            // Move (copy) object type
            _type = other._type;
            _gravity = other._gravity;

            // Move params
            _params = std::move(other._params);

            // Move controllers
            for (auto& controller : other._controllers)
                _controllers.push_back(std::move(controller));

            // Clear other controllers
            other._controllers.clear();
            other._controllers.shrink_to_fit();
        }

        RigidBody::~RigidBody()
        {
            // Clear controllers
            _controllers.clear();
            _controllers.shrink_to_fit();
        }

        /* Get object params */
        BodyParams& RigidBody::params() { return *_params.get(); }

        /* Get type */
        const BodyType& RigidBody::type() const { return _type; }

        /* Get body pointer */
        btRigidBody* RigidBody::body() { return _body; }

        /* Set pose */
        RigidBody& RigidBody::setPosition(const double& x, const double& y, const double& z)
        {
            btTransform transformation = _body->getCenterOfMassTransform();
            transformation.setOrigin(btVector3(x, y, z));
            _body->setCenterOfMassTransform(transformation);

            return *this;
        }

        /* Set Orientation */
        RigidBody& RigidBody::setOrientation(const double& roll, const double& pitch, const double& yaw) // here decide what to pass
        {
            btTransform transformation = _body->getCenterOfMassTransform();
            transformation.setRotation(btQuaternion(yaw, pitch, roll));
            _body->setCenterOfMassTransform(transformation);

            return *this;
        }

        RigidBody& RigidBody::activateGravity()
        {
            _gravity = true;
            return *this;
        }

        template <typename... Args>
        RigidBody& RigidBody::addControllers(std::unique_ptr<control::RigidBodyCtr> controller, Args... args)
        {
            // Add controller
            _controllers.push_back(std::move(controller));

            // Init controller
            // _controllers.back()->init();

            if constexpr (sizeof...(args) > 0)
                addControllers(args...);

            return *this;
        }

        void RigidBody::update()
        {
            // Vector of applied forces
            Eigen::Matrix<double, 6, 1> f = Eigen::VectorXd::Zero(6);

            // Gravity compenstation
            if (_gravity)
                f(2) += 9.81 / _body->getInvMass();

            // Controllers
            for (auto& controller : _controllers)
                f += controller->action(*this);

            // Apply forces
            _body->applyCentralForce(btVector3(f(0), f(1), f(2)));
            _body->applyTorque(btVector3(f(3), f(4), f(5)));
        }

        btRigidBody* RigidBody::createRigidBody(const btScalar& mass, const btTransform& transform, btCollisionShape* shape)
        {
            // Assert shape
            btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

            // rigidbody is dynamic if and only if mass is non zero, otherwise static
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
    } // namespace bodies
} // namespace beautiful_bullet
