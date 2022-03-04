#ifndef BEAUTIFUL_BULLET_MULTIBODY_HPP
#define BEAUTIFUL_BULLET_MULTIBODY_HPP

// Standard
#include <iostream>
#include <unordered_map>

// Bullet
#include <BulletCollision/btBulletCollisionCommon.h>
#include <LinearMath/btVector3.h>

// Pinocchio
#include <pinocchio/multibody/fwd.hpp>

#include "beautiful_bullet/control/MultiBody.h"
#include "beautiful_bullet/utils/BulletLoader.hpp"

namespace beautiful_bullet {
    namespace bodies {
        class MultiBody {
        public:
            /* Constructor */
            MultiBody(const std::string& file, int flags = 0);

            MultiBody() = default;

            // Move constructor
            MultiBody(MultiBody&& other) noexcept;

            // No copy constructor (check explicit and default)
            MultiBody(const MultiBody&) = delete;

            // Destroyer (not needed beacause of smart pointer; try to move towards raw or unique_ptr)
            ~MultiBody();

            /* Get multibody pointer */
            btMultiBody* body();

            /* Get agent state */
            const Eigen::VectorXd& state();

            /* Get agent state derivative */
            const Eigen::VectorXd& velocity();

            /* Get position */
            Eigen::Matrix<double, 6, 1> poseJoint(const int& index = -1);

            /* Get Jacobian */
            Eigen::MatrixXd jacobian(const int& index = -1);

            /* Get Bullet loader */
            utils::BulletLoader& loader() { return _loader; }

            /* Set multibody */
            MultiBody& setBody(btMultiBody* body);

            /* Set agent (base) position */
            MultiBody& setPosition(const double& x, const double& y, const double& z);

            /* Set agent (base) orientation */
            MultiBody& setOrientation(const double& roll, const double& pitch, const double& yaw);

            /* Set agent state */
            MultiBody& setState(const Eigen::VectorXd& q);

            /* Set agent state */
            MultiBody& setVelocity(const Eigen::VectorXd& v);

            /* Add controllers */
            template <typename... Args>
            MultiBody& addControllers(std::unique_ptr<control::MultiBodyCtr> controller, Args... args);

            /* Inverse Kinematics */
            Eigen::VectorXd inverseKinematics(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const std::string& index = "");

            /* Update model */
            void update();

        protected:
            // MultiBody's state (pos and vel)
            // Don't know if it is good to keep a copy of the body states here
            Eigen::VectorXd _q, _v;

            // Bullet MultiBody Object
            btMultiBody* _body = nullptr;

            // Dynamics model (ty to have them as pointers to alleviate compiling time required by Pinocchio)
            // raw pointers because smart pointers apparently needs to know the size of the object
            pinocchio::Data* _data = nullptr; // pinocchio::Data _data;
            pinocchio::Model* _model = nullptr; // pinocchio::Model _model;

            // Loader
            utils::BulletLoader _loader; // std::shared_ptr<utils::BulletLoader> _loader;

            // Root link inertia frame
            btTransform _rootFrame;

            // Controllers (for now shared because of issues in moving the agent object)
            std::vector<std::unique_ptr<control::MultiBodyCtr>> _controllers;

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
    } // namespace bodies
} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_MULTIBODY_HPP