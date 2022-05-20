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

#include "beautiful_bullet/bodies/MultiBody.hpp"
#include "beautiful_bullet/tools/math.hpp"

// Pinocchio
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace beautiful_bullet {
    namespace bodies {
        MultiBody::MultiBody(const std::string& file, int flags)
        {
            // Get multibody
            _body = _loader.parseMultiBody(file, flags);

            // Init agent internal state variable
            _x.setZero();
            _r.setIdentity();
            _q.setZero(_body->getNumDofs());
            _v.setZero(_body->getNumDofs());
            _tau.setZero(_body->getNumDofs());

            // Store inertia frame of the root node
            // (not keeping track of it at the moment)
            _rootFrame = _body->getBaseWorldTransform();

            // Pinocchio model
            _model = new pinocchio::Model();
            pinocchio::urdf::buildModel(file, *_model);

            // Pinocchio data
            _data = new pinocchio::Data(*_model);

            // Update pinocchio forward kinematics
            pinocchio::forwardKinematics(*_model, *_data, _q, _v);

            // Default no gravity compensation
            _gravity = false;
        }

        MultiBody::MultiBody(MultiBody&& other) noexcept
        {
            // Move (copy) state
            _x = other._x;
            _r = other._r;
            _q = other._q;
            _v = other._v;
            _tau = other._tau;
            _gravity = other._gravity;

            // Move RigidBody pointer
            _body = other._body;
            other._body = nullptr;

            // Move Pinocchio objects
            _data = other._data;
            other._data = nullptr;

            _model = other._model;
            other._model = nullptr;

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

        MultiBody::~MultiBody() // (not needed beacause of smart pointer; try to move towards raw or unique_ptr)
        {
            _controllers.clear();
            _controllers.shrink_to_fit();

            // Delete pinocchio objects
            delete _data;
            delete _model;
        }

        btMultiBody* MultiBody::body()
        {
            return _body;
        }

        const Eigen::VectorXd& MultiBody::state() const
        {
            return _q;
        }

        const Eigen::VectorXd& MultiBody::upperLimits() const // using pinocchio for the moment because Bullet is bad
        {
            return _model->upperPositionLimit;
        }

        const Eigen::VectorXd& MultiBody::lowerLimits() const
        {
            return _model->lowerPositionLimit;
        }

        const Eigen::VectorXd& MultiBody::velocity() const
        {
            return _v;
        }

        const Eigen::VectorXd& MultiBody::velocityLimits() const
        {
            return _model->velocityLimit;
        }

        Eigen::VectorXd MultiBody::acceleration()
        {
            // Using pinocchio to compute the forward dynamics
            pinocchio::aba(*_model, *_data, state(), velocity(), _tau);
            return _data->ddq;
        }

        const Eigen::VectorXd& MultiBody::torques() const
        {
            return _tau;
        }

        const Eigen::VectorXd& MultiBody::torquesLimits() const
        {
            return _model->effortLimit;
        }

        Eigen::MatrixXd MultiBody::inertiaMatrix()
        {
            pinocchio::crba(*_model, *_data, _q);
            _data->M.triangularView<Eigen::StrictlyLower>() = _data->M.transpose().triangularView<Eigen::StrictlyLower>();
            return _data->M;
        }

        Eigen::MatrixXd MultiBody::coriolisMatrix()
        {
            pinocchio::computeCoriolisMatrix(*_model, *_data, _q, _v);
            return _data->C;
        }

        Eigen::VectorXd MultiBody::gravityVector()
        {
            pinocchio::computeGeneralizedGravity(*_model, *_data, _q);
            return _data->g;
        }

        Eigen::VectorXd MultiBody::nonLinearEffects()
        {
            return pinocchio::nonLinearEffects(*_model, *_data, _q, _v);
        }

        Eigen::VectorXd MultiBody::inverseDynamics(const Eigen::VectorXd& ddq)
        {
            pinocchio::rnea(*_model, *_data, _q, _v, ddq);
            return _data->tau;
        }

        Eigen::Vector3d MultiBody::framePosition(const std::string& frame)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Compute the forward kinematics and update frame placements
            pinocchio::framesForwardKinematics(*_model, *_data, _q);

            // Get frame pose
            pinocchio::SE3 oMf = _data->oMf[FRAME_ID];

            return oMf.translation() + _x;
        }

        Eigen::Matrix3d MultiBody::frameOrientation(const std::string& frame)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Compute the forward kinematics and update frame placements
            pinocchio::framesForwardKinematics(*_model, *_data, _q);

            // Get frame pose
            pinocchio::SE3 oMf = _data->oMf[FRAME_ID];

            return oMf.rotation(); // add transformation for orientation base
        }

        Eigen::Matrix<double, 6, 1> MultiBody::framePose(const std::string& frame)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Compute the forward kinematics and update frame placements
            pinocchio::framesForwardKinematics(*_model, *_data, _q);

            // Get frame pose
            pinocchio::SE3 oMf = _data->oMf[FRAME_ID];
            Eigen::AngleAxisd rot(oMf.rotation());

            return (Eigen::Matrix<double, 6, 1>() << oMf.translation() + _x, rot.angle() * rot.axis()).finished(); // add transformation for orientation base
        }

        Eigen::Matrix<double, 6, 1> MultiBody::frameVelocity(const std::string& frame)
        {
            return jacobian(frame) * _v;
        }

        Eigen::MatrixXd MultiBody::jacobian(const std::string& frame)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Init Jacobian
            pinocchio::Data::Matrix6x J(6, _model->nv);
            J.setZero();

            // Compute the jacobian
            pinocchio::computeFrameJacobian(*_model, *_data, _q, FRAME_ID, J);

            return J;
        }

        Eigen::MatrixXd MultiBody::hessian(const std::string& frame)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Init Jacobian
            pinocchio::Data::Matrix6x H(6, _model->nv);
            H.setZero();

            // Compute the jacobian
            pinocchio::computeJointJacobiansTimeVariation(*_model, *_data, _q, _v);
            pinocchio::getFrameJacobianTimeVariation(*_model, *_data, FRAME_ID, pinocchio::LOCAL, H);

            return H;
        }

        MultiBody& MultiBody::setBody(btMultiBody* body) // (remember to init state here)
        {
            _body = body;

            return *this;
        };

        MultiBody& MultiBody::setPosition(const double& x, const double& y, const double& z)
        {
            _x << x, y, z;

            _body->setBasePos(btVector3(x, y, z) + _rootFrame.getOrigin());

            btAlignedObjectArray<btQuaternion> scratch_q;
            btAlignedObjectArray<btVector3> scratch_m;
            _body->forwardKinematics(scratch_q, scratch_m);
            _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

            return *this;
        }

        /* Set agent (base) orientation */
        MultiBody& MultiBody::setOrientation(const double& roll, const double& pitch, const double& yaw)
        {
            // add base orientation allocation _r

            _body->setWorldToBaseRot(btQuaternion(yaw, pitch, roll));

            btAlignedObjectArray<btQuaternion> scratch_q;
            btAlignedObjectArray<btVector3> scratch_m;
            _body->forwardKinematics(scratch_q, scratch_m);
            _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

            return *this;
        }

        /* Set agent state */
        MultiBody& MultiBody::setState(const Eigen::VectorXd& q)
        {
            // Update agent state
            _q = q;

            // Update bullet body state
            for (size_t i = 0; i < _body->getNumDofs(); i++)
                _body->setJointPos(i, _q(i));

            // Update bullet world state
            btAlignedObjectArray<btQuaternion> scratch_q;
            btAlignedObjectArray<btVector3> scratch_m;
            _body->forwardKinematics(scratch_q, scratch_m);
            _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

            // Update pinocchio state (anything else necessary?)
            pinocchio::forwardKinematics(*_model, *_data, _q, _v);

            return *this;
        }

        /* Set agent state */
        MultiBody& MultiBody::setVelocity(const Eigen::VectorXd& v)
        {
            // Update agent state
            _v = v;

            // Update bullet body state
            for (size_t i = 0; i < _body->getNumDofs(); i++)
                _body->setJointVel(i, _v(i));

            // Update bullet world state
            btAlignedObjectArray<btQuaternion> scratch_q;
            btAlignedObjectArray<btVector3> scratch_m;
            _body->forwardKinematics(scratch_q, scratch_m);
            _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

            // Update pinocchio state (anything else necessary?)
            pinocchio::forwardKinematics(*_model, *_data, _q, _v);

            return *this;
        }

        MultiBody& MultiBody::setTorques(const Eigen::VectorXd& tau)
        {
            _tau = tau;

            // Update bullet body state
            for (size_t i = 0; i < _body->getNumDofs(); i++)
                _body->addJointTorque(i, _tau(i));

            return *this;
        }

        MultiBody& MultiBody::activateGravity()
        {
            _gravity = true;
            return *this;
        }

        /* Inverse Kinematics */
        Eigen::VectorXd MultiBody::inverseKinematics(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const std::string& frame, const Eigen::VectorXd* ref, const double& w)
        {
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            Eigen::Vector3d pos = position;
            for (size_t i = 0; i < 3; i++)
                pos[i] -= _body->getBasePos()[i];

            std::cout << pos.transpose() << std::endl;

            // Desired Pose
            const pinocchio::SE3 oMdes(orientation, pos);

            // Jacobian
            pinocchio::Data::Matrix6x J(6, _model->nv);
            J.setZero();

            // Optim params
            const int IT_MAX = 1000;
            const double eps = 1e-8, DT = 1e-1, damp = 1e-6;

            // Loop
            bool success = false;
            typedef Eigen::Matrix<double, 6, 1> Vector6d;
            Vector6d err;
            Eigen::VectorXd v(_model->nv);

            // Init configuration
            Eigen::VectorXd q = _q;

            for (int i = 0;; i++) {
                // Compute the forward kinematics and update frame placements
                pinocchio::framesForwardKinematics(*_model, *_data, q);

                // Compute error between current and desired pose
                const pinocchio::SE3 dMf = oMdes.actInv(_data->oMf[FRAME_ID]);
                err = pinocchio::log6(dMf).toVector();

                // Check if stopping
                if (err.norm() < eps) {
                    success = true;
                    break;
                }
                if (i >= IT_MAX) {
                    success = false;
                    break;
                }

                // Compute the jacobian
                pinocchio::computeFrameJacobian(*_model, *_data, q, FRAME_ID, J);

                // Calculate damped pseudo-inverse (here why not applying Mantegazza method?)
                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += damp;

                // Calculate velocity
                v.noalias() = -J.transpose() * JJt.ldlt().solve(err);

                if (ref)
                    v.noalias() += w * (Eigen::MatrixXd::Identity(_model->nv, _model->nv) - tools::pseudoInverse(J) * J) * (*ref - q);

                // Calculate configuration
                q = pinocchio::integrate(*_model, q, v * DT);

                if (!(i % 10))
                    std::cout << i << ": error = " << err.transpose() << std::endl;
            }

            return q;
        }

        /* Update model */
        void MultiBody::update()
        {
            // Get joint pose, vel and effort
            for (size_t i = 0; i < _body->getNumDofs(); i++) {
                _q(i) = _body->getJointPos(i);
                _v(i) = _body->getJointVel(i);
                _tau(i) = _body->getJointTorque(i);
            }

            std::cout << _tau.transpose() << std::endl;

            // Gravity compensation
            if (_gravity)
                _tau += pinocchio::nonLinearEffects(*_model, *_data, _q, _v); // pinocchio::computeGeneralizedGravity(*_model, *_data, _q);

            // Control
            for (auto& controller : _controllers) {
                if (controller->mode() == ControlMode::CONFIGURATIONSPACE)
                    _tau += controller->action(*this);
                else if (controller->mode() == ControlMode::OPERATIONSPACE)
                    _tau += jacobian(controller->frame()).transpose() * controller->action(*this);
            }

            // ugly solution (fix this)
            _body->clearForcesAndTorques();

            // Set torques
            for (size_t i = 0; i < _body->getNumDofs(); i++) {
                // Clip force
                clipForce(i, _tau(i));

                // Apply force
                _body->addJointTorque(i, _tau(i));
            }
        }
    } // namespace bodies
} // namespace beautiful_bullet