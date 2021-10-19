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
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#endif

#include "beautiful_bullet/Control.hpp"
#include "beautiful_bullet/utils/BulletLoader.hpp"

namespace beautiful_bullet {
    class Agent {
    public:
        /* Constructor */
        Agent(const std::string& file, int flags = 0)
        {
            // Create loader
            // _loader = std::make_shared<utils::BulletLoader>();

            // Get multibody
            _body = _loader.parseMultiBody(file, flags);

// Pinocchio model
#ifdef USE_PINOCCHIO
            pinocchio::urdf::buildModel(file, _model);
            _data = pinocchio::Data(_model); // _model.gravity.linear(Eigen::Vector3d(0, 0, -9.81));
#endif

            // Init agent internal state variable
            _q.setZero(_body->getNumDofs());
            _v.setZero(_body->getNumDofs());

            // Store inertia frame of the root node
            _rootFrame = _body->getBaseWorldTransform();
        }

        Agent() = default;

        // Move constructor
        Agent(Agent&& other) noexcept
        {
            // Move (copy) state
            _q = other._q;
            _v = other._v;

            // Move RigidBody pointer
            _body = other._body;
            other._body = nullptr;

// Move Pinocchio objects
#ifdef USE_PINOCCHIO
            _data = other._data;
            _model = other._model;
#endif

            // Move loader
            _loader = other._loader;

            // Move root frame
            _rootFrame = other._rootFrame;

            // Move controllers
            for (auto& controller : other._controllers)
                _controllers.push_back(std::move(controller));

            // Clear other controllers
            other._controllers.clear();
            other._controllers.shrink_to_fit();
        }

        // No copy constructor (check explicit and default)
        Agent(const Agent&) = delete;

        // Destroyer (not needed beacause of smart pointer; try to move towards raw or unique_ptr)
        ~Agent()
        {
            _controllers.clear();
            _controllers.shrink_to_fit();
        }

        /* Get multibody pointer */
        btMultiBody* body() { return _body; }

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

        /* Update model */
        void update()
        {
            // Get joint pose and vel
            for (size_t i = 0; i < _body->getNumDofs(); i++) {
                _q(i) = _body->getJointPos(i);
                _v(i) = _body->getJointVel(i);
            }

            // Command force
            Eigen::VectorXd tau = Eigen::VectorXd::Zero(_body->getNumDofs());

// Gravity compensation
#ifdef USE_PINOCCHIO
            tau += pinocchio::nonLinearEffects(_model, _data, _q, _v); // pinocchio::computeGeneralizedGravity(_model, _data, _q);
#endif

            // Control
            for (auto& controller : _controllers)
                tau += controller->control(_q, _v);

            // Set gravity compensation
            for (size_t i = 0; i < _body->getNumDofs(); i++) {
                // Clip force
                clipForce(i, tau(i));

                // Apply force
                _body->addJointTorque(i, tau(i));
            }
        }

    protected:
        // Agent's state (pos and vel)
        Eigen::VectorXd _q, _v;

        // Bullet MultiBody Object
        btMultiBody* _body = nullptr;

// Dynamics model
#ifdef USE_PINOCCHIO
        pinocchio::Data _data;
        pinocchio::Model _model;
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