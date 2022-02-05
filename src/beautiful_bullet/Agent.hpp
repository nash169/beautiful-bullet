#ifndef BEAUTIFUL_BULLET_AGENT_HPP
#define BEAUTIFUL_BULLET_AGENT_HPP

// Standard
#include <iostream>
#include <unordered_map>

// Bullet
#include <BulletCollision/btBulletCollisionCommon.h>
#include <LinearMath/btVector3.h>

// Pinocchio
#ifdef USE_PINOCCHIO
#include <pinocchio/multibody/fwd.hpp>
#endif

#include "beautiful_bullet/Control.hpp"
#include "beautiful_bullet/utils/BulletLoader.hpp"

namespace beautiful_bullet {
    class Agent {
    public:
        /* Constructor */
        Agent(const std::string& file, int flags = 0);

        Agent() = default;

        // Move constructor
        Agent(Agent&& other) noexcept;

        // No copy constructor (check explicit and default)
        Agent(const Agent&) = delete;

        // Destroyer (not needed beacause of smart pointer; try to move towards raw or unique_ptr)
        ~Agent();

        /* Get multibody pointer */
        btMultiBody* body() { return _body; }

        /* Get position */
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> poseJoint(const size_t& index = -1);

        /* Get agent state */
        const Eigen::VectorXd& state() { return _q; }

        /* Get agent state derivative */
        const Eigen::VectorXd& velocity() { return _v; }

        /* Get Bullet loader */
        utils::BulletLoader& loader() { return _loader; }
        // std::shared_ptr<utils::BulletLoader> loader() { return _loader; }

        /* Set multibody */ // (remember to init state here)
        Agent& setBody(btMultiBody* body)
        {
            _body = body;

            return *this;
        };

        /* Set agent (base) pose */
        Agent& setPosition(const double& x, const double& y, const double& z)
        {
            _body->setBasePos(btVector3(x, y, z) + _rootFrame.getOrigin());

            btAlignedObjectArray<btQuaternion> scratch_q;
            btAlignedObjectArray<btVector3> scratch_m;
            _body->forwardKinematics(scratch_q, scratch_m);
            _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

            return *this;
        }

        Agent& setOrientation(const double& roll, const double& pitch, const double& yaw)
        {
            _body->setWorldToBaseRot(btQuaternion(yaw, pitch, roll));

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

        /* Add controllers */
        template <typename... Args>
        Agent& addControllers(std::unique_ptr<Control> controller, Args... args)
        {
            // Add controller
            _controllers.push_back(std::move(controller));

            // Init controller
            _controllers.back()->init();

            if constexpr (sizeof...(args) > 0)
                addControllers(std::move(args)...);

            return *this;
        }

#ifdef USE_PINOCCHIO
        /* Inverse Kinematics */
        Eigen::VectorXd inverseKinematics(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const size_t& index = -1);
#endif

        /* Update model */
        void update();

    protected:
        // Agent's state (pos and vel)
        Eigen::VectorXd _q, _v;

        // Bullet MultiBody Object
        btMultiBody* _body = nullptr;

// Dynamics model
#ifdef USE_PINOCCHIO
        // Try to have them as pointers to alleviate compiling time required by Pinocchio
        pinocchio::Data* _data = nullptr;
        pinocchio::Model* _model = nullptr;
        // pinocchio::Data _data;
        // pinocchio::Model _model;
#endif

        // Loader
        utils::BulletLoader _loader;
        // std::shared_ptr<utils::BulletLoader> _loader;

        // Root link inertia frame
        btTransform _rootFrame;

        // Controllers (for now shared because of issues in moving the agent object)
        std::vector<std::unique_ptr<Control>> _controllers;

        // Clip force
        inline void clipForce(const int& index, double& force)
        {
            double maxForce = _body->getLink(index).m_jointMaxForce;

            if (force >= maxForce)
                force = maxForce;
            else if (force <= -maxForce)
                force = -maxForce;
        }
    };
} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_AGENT_HPP