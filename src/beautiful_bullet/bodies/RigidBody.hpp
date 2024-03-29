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

#ifndef BEAUTIFULBULLET_BODIES_RIGIDBODY_HPP
#define BEAUTIFULBULLET_BODIES_RIGIDBODY_HPP

#include "beautiful_bullet/common/pointers.hpp"
#include "beautiful_bullet/control/RigidBody.h"

#include <BulletCollision/btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <iostream>
#include <memory>
#include <vector>

namespace beautiful_bullet {
    namespace bodies {
        enum class BodyType : unsigned int {
            SPHERE = 1 << 0,
            CAPSULE = 1 << 1,
            CYLINDER = 1 << 2,
            BOX = 1 << 3,
            RIGIDBODY = SPHERE | CAPSULE | CYLINDER | BOX
        };

        struct BodyParams {
            BodyParams() : mass(0), friction(0), color("grey") {}

            BodyParams& setMass(const btScalar& mass)
            {
                this->mass = mass;
                return *this;
            }

            BodyParams& setFriction(const btScalar& friction)
            {
                this->friction = friction;
                return *this;
            }

            BodyParams& setColor(const std::string& color)
            {
                this->color = color;
                return *this;
            }

            // Dynamics properties
            btScalar mass, friction;

            // Color
            std::string color;
        };

        struct BoxParams : public BodyParams {
            BoxParams() : BodyParams(), size(btVector3(1, 1, 1))
            {
            }

            BoxParams& setSize(const double& x, const double& y, const double& z)
            {
                this->size = btVector3(x, y, z);
                return *this;
            }

            // Size
            btVector3 size;
        };

        struct SphereParams : public BodyParams {
            SphereParams() : BodyParams(), radius(1)
            {
            }

            SphereParams& setRadius(const btScalar& radius)
            {
                this->radius = radius;
                return *this;
            }

            // Radius
            btScalar radius;
        };

        struct CylinderParams : public BodyParams {
            CylinderParams() : BodyParams(), radius1(1), radius2(1), height(1)
            {
            }

            CylinderParams& setRadius1(const double& r1)
            {
                this->radius1 = r1;
                return *this;
            }

            CylinderParams& setRadius2(const double& r2)
            {
                this->radius2 = r2;
                return *this;
            }

            CylinderParams& setHeight(const double& h)
            {
                this->height = h;
                return *this;
            }

            // Params
            btScalar radius1, radius2, height;
        };

        struct CapsuleParams : public BodyParams {
            CapsuleParams() : BodyParams(), radius(1), height(1)
            {
            }

            CapsuleParams& setRadius(const double& r)
            {
                this->radius = r;
                return *this;
            }

            CapsuleParams& setHeight(const double& h)
            {
                this->height = h;
                return *this;
            }

            // Size
            btScalar radius, height;
        };

        class RigidBody {
        public:
            // Constructor
            RigidBody(const std::string& shape, const BodyParams& params);

            // Default constructor
            RigidBody() = default;

            // Move constructor
            RigidBody(RigidBody&& other) noexcept;

            // No copy constructor (check explicit and default)
            RigidBody(const RigidBody&) = delete; // it might not work just because I'm with clang

            ~RigidBody();

            /* Get object params */
            BodyParams& params();

            /* Get type */
            const BodyType& type() const;

            /* Get body pointer */
            btRigidBody* body();

            /* Set pose */
            RigidBody& setPosition(const double& x, const double& y, const double& z);

            /* Set Orientation */
            RigidBody& setOrientation(const double& roll, const double& pitch, const double& yaw);

            /* Activate gravity compensation */
            RigidBody& activateGravity();

            /* Add controllers */
            template <typename... Args>
            RigidBody& addControllers(std::unique_ptr<control::RigidBodyCtr> controller, Args... args);

            // Update model
            void update();

        protected:
            // Bullet rigid body
            btRigidBody* _body = nullptr;

            // RigidBody type
            BodyType _type;

            // RigidBody params (still shared for problems in moving the object with unique_ptr)
            // Pointer in order to avoid object slicing
            // https://stackoverflow.com/questions/8777724/store-derived-class-objects-in-base-class-variables
            // https://stackoverflow.com/questions/1541031/is-it-possible-for-slicing-to-occur-with-smart-pointers
            std::unique_ptr<BodyParams> _params;

            // Gravity compensation
            bool _gravity;

            // Controllers (for now shared because of issues in moving the agent object)
            std::vector<std::unique_ptr<control::RigidBodyCtr>> _controllers;

            // Create Rigid Body
            btRigidBody* createRigidBody(const btScalar& mass, const btTransform& transform, btCollisionShape* shape);
        };

        COMMON_DECLARE_SHARED(RigidBody)
    } // namespace bodies
} // namespace beautiful_bullet

#endif // BEAUTIFULBULLET_BODIES_RIGIDBODY_HPP