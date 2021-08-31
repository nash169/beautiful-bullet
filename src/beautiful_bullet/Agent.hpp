#ifndef BEAUTIFUL_BULLET_AGENT_HPP
#define BEAUTIFUL_BULLET_AGENT_HPP

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

#include "beautiful_bullet/Control.hpp"
#include "beautiful_bullet/utils/BulletLoader.hpp"

namespace beautiful_bullet {
    class Agent {
    public:
        /* Constructor */
        Agent(const std::string& file, int flags = 0)
        {
            // Create loader
            _loader = std::make_shared<utils::BulletLoader>();

            // Get multibody
            _body = _loader->parseMultiBody(file, flags);

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

        // // Destroyer
        // ~Agent()
        // {
        //     for (auto& controller : _controllers)
        //         delete controller;
        // }

        /* Get multibody pointer */
        btMultiBody* body() { return _body; }

        /* Get agent state */
        const Eigen::VectorXd& state() { return _q; }

        /* Get agent state derivative */
        const Eigen::VectorXd& velocity() { return _v; }

        /* Get Bullet loader */
        std::shared_ptr<utils::BulletLoader> loader() { return _loader; }

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

        /* Add controllers */
        template <typename... Args>
        Agent& addControllers(std::shared_ptr<Control> controller, Args... args)
        {
            // Add controller
            _controllers.push_back(controller);

            // Init controller
            _controllers.back()->init();

            if constexpr (sizeof...(args) > 0)
                addControllers(args...);

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
            tau += pinocchio::nonLinearEffects(_model, _data, _q, _v); // pinocchio::computeGeneralizedGravity(_model, _data, _q);

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
        pinocchio::Data _data;
        pinocchio::Model _model;

        // Loader
        std::shared_ptr<utils::BulletLoader> _loader;

        // Root link inertia frame
        btTransform rootFrame;

        // Controllers
        std::vector<std::shared_ptr<Control>> _controllers;

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