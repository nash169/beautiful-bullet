#ifndef BEAUTIFUL_BULLET_TYPES_HPP
#define BEAUTIFUL_BULLET_TYPES_HPP

#include <Eigen/Geometry>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>

namespace beautiful_bullet {
    // check here if it useful to depend on corrade
    enum class ObjectType : unsigned int {
        SPHERE = 1 << 0,
        CAPSULE = 1 << 1,
        CYLINDER = 1 << 2,
        BOX = 1 << 3,
        RIGIDBODY = SPHERE | CAPSULE | CYLINDER | BOX
    };

    struct ObjectParams {
        ObjectParams() : mass(0), friction(0), color("grey")
        {
            transform.setIdentity();
        }

        ObjectParams& setMass(const btScalar& mass)
        {
            this->mass = mass;
            return *this;
        }

        ObjectParams& setFriction(const btScalar& friction)
        {
            this->friction = friction;
            return *this;
        }

        ObjectParams& setColor(const std::string& color)
        {
            this->color = color;
            return *this;
        }

        // ObjectParams& setPose(const Eigen::Vector3d& pose)
        // {
        //     transform.setOrigin(btVector3(pose(0), pose(1), pose(2)));
        //     return *this;
        // }

        ObjectParams& setRotation(const btScalar& angle, const Eigen::Vector3d& axis)
        {
            transform.setRotation(btQuaternion(btVector3(axis(0), axis(1), axis(2)), angle));
            return *this;
        }

        // Dynamics properties
        btScalar mass, friction;

        // Frame
        btTransform transform;

        // Color
        std::string color;
    };

    struct BoxParams : public ObjectParams {
        BoxParams() : ObjectParams(), size(btVector3(1, 1, 1))
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

    struct SphereParams : public ObjectParams {
        SphereParams() : ObjectParams(), radius(1)
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

    struct CylinderParams : public ObjectParams {
        CylinderParams() : ObjectParams(), radius1(1), radius2(1), height(1)
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

    struct CapsuleParams : public ObjectParams {
        CapsuleParams() : ObjectParams(), radius(1), height(1)
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
} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_TYPES_HPP