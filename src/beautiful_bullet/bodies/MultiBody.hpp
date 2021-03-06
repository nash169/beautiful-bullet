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

#ifndef BEAUTIFULBULLET_BODIES_MULTIBODY_HPP
#define BEAUTIFULBULLET_BODIES_MULTIBODY_HPP

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

            /* Get multibody joint state */
            const Eigen::VectorXd& state() const;
            const Eigen::VectorXd& upperLimits() const;
            const Eigen::VectorXd& lowerLimits() const;

            /* Get multibody joint state derivative */
            const Eigen::VectorXd& velocity() const;
            const Eigen::VectorXd& velocityLimits() const;

            /* Get multibody joint state second derivative */
            Eigen::VectorXd acceleration();

            /* Get multibody joint torques */
            const Eigen::VectorXd& torques() const;
            const Eigen::VectorXd& torquesLimits() const;

            /* Get Dynamics */
            Eigen::MatrixXd inertiaMatrix();
            Eigen::MatrixXd coriolisMatrix();
            Eigen::VectorXd gravityVector();
            Eigen::VectorXd nonLinearEffects();
            Eigen::VectorXd inverseDynamics(const Eigen::VectorXd& ddq);

            /* Get pose of the frame */
            Eigen::Vector3d framePosition(const std::string& frame = "");
            Eigen::Matrix3d frameOrientation(const std::string& frame = "");
            Eigen::Matrix<double, 6, 1> framePose(const std::string& frame = "");

            /* Get velocity of the frame */
            Eigen::Matrix<double, 6, 1> frameVelocity(const std::string& frame = "");

            /* Get Jacobian */
            Eigen::MatrixXd jacobian(const std::string& frame = "");

            /* Get Hessian (Jacobian time variation precisely) */
            Eigen::MatrixXd hessian(const std::string& frame = "");

            /* Get Bullet loader */
            utils::BulletLoader& loader() { return _loader; }

            /* Set multibody */
            MultiBody& setBody(btMultiBody* body);

            /* Set multibody (base) position */
            MultiBody& setPosition(const double& x, const double& y, const double& z);

            /* Set multibody (base) orientation */
            MultiBody& setOrientation(const double& roll, const double& pitch, const double& yaw);

            /* Set multibody state */
            MultiBody& setState(const Eigen::VectorXd& q);

            /* Set multibody state */
            MultiBody& setVelocity(const Eigen::VectorXd& v);

            /* Set torques */
            MultiBody& setTorques(const Eigen::VectorXd& tau);

            /* Activate gravity compensation */
            MultiBody& activateGravity();

            /* Add controllers */
            template <typename... Args>
            MultiBody& addControllers(std::unique_ptr<control::MultiBodyCtr> controller, Args... args)
            {
                // Add controller
                _controllers.push_back(std::move(controller));

                // Init controller
                // _controllers.back()->init();

                if constexpr (sizeof...(args) > 0)
                    addControllers(std::move(args)...);

                return *this;
            }

            /* Inverse Kinematics */
            Eigen::VectorXd inverseKinematics(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const std::string& frame = "", const Eigen::VectorXd* ref = nullptr, const double& w = 1.0);

            /* Update model */
            void update();

        protected:
            // Gravity compensation
            bool _gravity;

            // MultiBody's state (pos and vel)
            // Don't know if it is good to keep a copy of the body states here
            Eigen::Vector3d _x;
            Eigen::Matrix3d _r;
            Eigen::VectorXd _q, _v, _tau;

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

#endif // BEAUTIFULBULLET_BODIES_MULTIBODY_HPP