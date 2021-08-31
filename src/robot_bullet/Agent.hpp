#ifndef ROBOT_BULLET_AGENT_HPP
#define ROBOT_BULLET_AGENT_HPP

// Standard
#include <iostream>
#include <unordered_map>

// Bullet
#include <BulletCollision/btBulletCollisionCommon.h>
#include <LinearMath/btVector3.h>

// Pinocchio
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "robot_bullet/utils/BulletLoader.hpp"

namespace robot_bullet {
    class Agent {
    public:
        /* Constructor */
        Agent(const std::string& file)
        {
            // Create loader
            _loader = std::make_shared<utils::BulletLoader>();

            // Get multibody
            _body = _loader->parseMultiBody(file);

            // Pinocchio model
            pinocchio::urdf::buildModel(file, _model);
            _data = pinocchio::Data(_model); // _model.gravity.linear(Eigen::Vector3d(0, 0, -9.81));
            
            // Init agent internal state variable
            _q.setZero(_body->getNumDofs());
            _v.setZero(_body->getNumDofs());

            // Store inertia frame of the root node
            rootFrame = _body->getBaseWorldTransform();
        }

        Agent() = default;

        // No copy constructor (check explicit and default)
        // Agent(const Agent&) = delete;

        // Destroyer
        // ~Agent() { delete _body; }

        /* Get multibody pointer */
        btMultiBody* body() { return _body; }

        /* Get agent state */
        const Eigen::VectorXd& state() { return _q; }

        /* Get agent state derivative */
        const Eigen::VectorXd& velocity() { return _v; }

        /* Get Bullet loader */
        std::shared_ptr<utils::BulletLoader> loader(){return _loader;}

        /* Set agent (base) pose */
        Agent& setPose(const double& x, const double& y, const double& z)
        {
            _body->setBasePos(btVector3(x, y, z) + rootFrame.getOrigin());

            btAlignedObjectArray<btQuaternion> scratch_q;
            btAlignedObjectArray<btVector3> scratch_m;
            _body->forwardKinematics(scratch_q, scratch_m);
            _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

            return *this;
        }

        /* Set agent state */
        Agent& setState(const Eigen::VectorXd& q)
        {
            for (size_t i = 0; i < _body->getNumDofs(); i++)
                _body->setJointPos(i, q(i));

            btAlignedObjectArray<btQuaternion> scratch_q;
            btAlignedObjectArray<btVector3> scratch_m;
            _body->forwardKinematics(scratch_q, scratch_m);
            _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

            return *this;
        }

        /* Update model */
        void update()
        {
            // Get joint pose and vel
            for (size_t i = 0; i < _body->getNumDofs(); i++) {
                _q(i) = _body->getJointPos(i);
                _v(i) = _body->getJointVel(i);
            }

            // Get non-linear effects for Pinocchio
            const Eigen::VectorXd& tau = pinocchio::nonLinearEffects(_model, _data, _q, _v);
            // const Eigen::VectorXd& tau = pinocchio::computeGeneralizedGravity(_model, _data, _q);

            // Set gravity compensation
            for (size_t i = 0; i < _body->getNumDofs(); i++) {
                double maxForce = _body->getLink(i).m_jointMaxForce,
                       force;

                if (tau(i) >= maxForce)
                    force = maxForce;
                else if (tau(i) <= -maxForce)
                    force = -maxForce;
                else
                    force = tau(i);

                _body->addJointTorque(i, force);
            }
        }

    protected:
        // Agent's state (pos and vel)
        Eigen::VectorXd _q, _v;

        // Bullet MultiBody Object
        btMultiBody* _body = nullptr;

        // Dynamics model
        pinocchio::Data _data;
        pinocchio::Model _model;

        // Loader
        std::shared_ptr<utils::BulletLoader> _loader;

        // Root link inertia frame
        btTransform rootFrame;
    }; // namespace robot_raisim
} // namespace robot_bullet

#endif // ROBOT_BULLET_AGENT_HPP