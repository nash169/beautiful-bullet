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
        MultiBody::MultiBody()
        {
            // Gravity compensation off
            _gravity = false;
        }

        MultiBody::MultiBody(const std::string& file, int flags) : MultiBody()
        {
            loadBody(file, flags);
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
            _data = std::move(other._data);

            _model = std::move(other._model);

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

        MultiBody::~MultiBody() // (not needed because of smart pointer; try to move towards raw or unique_ptr)
        {
            _controllers.clear();
            _controllers.shrink_to_fit();
        }

        btMultiBody* MultiBody::body() { return _body; }

        Eigen::VectorXd MultiBody::state() const { return _q; }
        Eigen::VectorXd MultiBody::positionLower() const { return _model->lowerPositionLimit; }
        Eigen::VectorXd MultiBody::positionUpper() const { return _model->upperPositionLimit; }

        Eigen::VectorXd MultiBody::velocity() const { return _v; }
        Eigen::VectorXd MultiBody::velocityLower() const { return -_model->velocityLimit; }
        Eigen::VectorXd MultiBody::velocityUpper() const { return _model->velocityLimit; }

        Eigen::VectorXd MultiBody::acceleration()
        {
            // Using pinocchio to compute the forward dynamics
            pinocchio::aba(*_model, *_data, _q, _v, _tau);
            return _data->ddq;
        }
        Eigen::VectorXd MultiBody::accelerationLower() const { return -_model->velocityLimit * 10; }
        Eigen::VectorXd MultiBody::accelerationUpper() const { return _model->velocityLimit * 10; }

        Eigen::VectorXd MultiBody::effort() const { return _tau; }
        Eigen::VectorXd MultiBody::effortLower() const { return -_model->effortLimit; }
        Eigen::VectorXd MultiBody::effortUpper() const { return _model->effortLimit; }

        Eigen::VectorXd MultiBody::inverseDynamics(const Eigen::VectorXd& ddq)
        {
            pinocchio::rnea(*_model, *_data, _q, _v, ddq);
            return _data->tau;
        }

        Eigen::MatrixXd MultiBody::inertiaMatrix(const Eigen::VectorXd& q)
        {
            pinocchio::crba(*_model, *_data, q);
            _data->M.triangularView<Eigen::StrictlyLower>() = _data->M.transpose().triangularView<Eigen::StrictlyLower>();
            return _data->M;
        }

        Eigen::MatrixXd MultiBody::coriolisMatrix(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
        {
            pinocchio::computeCoriolisMatrix(*_model, *_data, q, dq);
            return _data->C;
        }

        Eigen::VectorXd MultiBody::gravityVector(const Eigen::VectorXd& q)
        {
            pinocchio::computeGeneralizedGravity(*_model, *_data, q);
            return _data->g;
        }

        Eigen::VectorXd MultiBody::nonLinearEffects(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
        {
            return pinocchio::nonLinearEffects(*_model, *_data, q, dq);
        }

        Eigen::MatrixXd MultiBody::selectionMatrix(const Eigen::VectorXd& tau)
        {
            return Eigen::MatrixXd::Identity(tau.size(), tau.size());
        }

        Eigen::MatrixXd MultiBody::inertiaMatrix() { return inertiaMatrix(_q); }
        Eigen::MatrixXd MultiBody::coriolisMatrix() { return coriolisMatrix(_q, _v); }
        Eigen::VectorXd MultiBody::gravityVector() { return gravityVector(_q); }
        Eigen::VectorXd MultiBody::nonLinearEffects() { return nonLinearEffects(_q, _v); }
        Eigen::MatrixXd MultiBody::selectionMatrix() { return selectionMatrix(_tau); }

        Eigen::MatrixXd MultiBody::jacobian(const Eigen::VectorXd& q, const std::string& frame, const pinocchio::ReferenceFrame& rf)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Init Jacobian
            pinocchio::Data::Matrix6x J(6, _model->nv);
            J.setZero();

            pinocchio::computeFrameJacobian(*_model, *_data, q, FRAME_ID, rf, J);

            return J;
        }

        Eigen::MatrixXd MultiBody::jacobianDerivative(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const std::string& frame, const pinocchio::ReferenceFrame& rf)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Init Jacobian
            pinocchio::Data::Matrix6x H(6, _model->nv);
            H.setZero();

            // Compute the jacobian
            pinocchio::computeJointJacobiansTimeVariation(*_model, *_data, q, dq);
            pinocchio::getFrameJacobianTimeVariation(*_model, *_data, FRAME_ID, rf, H);

            return H;
        }

        Eigen::Vector3d MultiBody::framePosition(const Eigen::VectorXd& q, const std::string& frame)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Compute the forward kinematics and update frame placements
            pinocchio::forwardKinematics(*_model, *_data, q);
            pinocchio::updateFramePlacement(*_model, *_data, FRAME_ID);

            // Get frame pose
            pinocchio::SE3 oMf = _data->oMf[FRAME_ID];

            return oMf.translation() + _x;
        }

        Eigen::Matrix3d MultiBody::frameOrientation(const Eigen::VectorXd& q, const std::string& frame)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Compute the forward kinematics and update frame placements
            pinocchio::framesForwardKinematics(*_model, *_data, q);

            // Get frame pose
            pinocchio::SE3 oMf = _data->oMf[FRAME_ID];

            return oMf.rotation(); // add transformation for orientation base
        }

        Eigen::Matrix<double, 6, 1> MultiBody::framePose(const Eigen::VectorXd& q, const std::string& frame)
        {
            // Get frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Compute the forward kinematics and update frame placements
            pinocchio::framesForwardKinematics(*_model, *_data, q);

            // Get frame pose
            pinocchio::SE3 oMf = _data->oMf[FRAME_ID];
            Eigen::AngleAxisd rot(oMf.rotation());

            return (Eigen::Matrix<double, 6, 1>() << oMf.translation() + _x, rot.angle() * rot.axis()).finished(); // add transformation for orientation base
        }

        Eigen::Matrix<double, 6, 1> MultiBody::frameVelocity(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, const std::string& frame)
        {
            return jacobian(q, frame) * dq;
        }

        Eigen::MatrixXd MultiBody::jacobian(const std::string& frame, const pinocchio::ReferenceFrame& rf) { return jacobian(_q, frame, rf); }
        Eigen::MatrixXd MultiBody::jacobianDerivative(const std::string& frame, const pinocchio::ReferenceFrame& rf) { return jacobianDerivative(_q, _v, frame, rf); }
        Eigen::Vector3d MultiBody::framePosition(const std::string& frame) { return framePosition(_q, frame); }
        Eigen::Matrix3d MultiBody::frameOrientation(const std::string& frame) { return frameOrientation(_q, frame); }
        Eigen::Matrix<double, 6, 1> MultiBody::framePose(const std::string& frame) { return framePose(_q, frame); }
        Eigen::Matrix<double, 6, 1> MultiBody::frameVelocity(const std::string& frame) { return frameVelocity(_q, _v, frame); }

        utils::BulletLoader& MultiBody::loader()
        {
            return _loader;
        }

        MultiBody& MultiBody::loadBody(const std::string& file, int flags)
        {
            // Get multibody
            _body = _loader.parseMultiBody(file, flags);

            // Init agent internal state variable
            _q.setZero(_body->getNumDofs());
            _v.setZero(_body->getNumDofs());
            _tau.setZero(_body->getNumDofs());

            // Store inertia frame of the root node
            // (not keeping track of it at the moment)
            _rootFrame = _body->getBaseWorldTransform();
            _x.setZero();
            // _x = Eigen::Map<Eigen::Vector3f>(static_cast<float*>(_rootFrame.getOrigin().m_floats));
            _r.setIdentity();

            // Pinocchio model
            _model = std::make_unique<pinocchio::Model>(pinocchio::Model());
            // _model = new pinocchio::Model();
            pinocchio::urdf::buildModel(file, *_model);

            // Pinocchio data
            _data = std::make_unique<pinocchio::Data>(pinocchio::Data(*_model));
            // _data = new pinocchio::Data(*_model);

            // Update pinocchio forward kinematics
            pinocchio::forwardKinematics(*_model, *_data, _q, _v);

            return *this;
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
        Eigen::VectorXd MultiBody::inverseKinematics(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const std::string& frame) // const Eigen::VectorXd* ref, const double& w
        {
            // Frame ID
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Target SE3
            const pinocchio::SE3 oMdes(orientation, position);

            // Optimization params
            const int IT_MAX = 1000;
            const double eps = 1e-4, DT = 1e-1, damp = 1e-6;

            // Initial state
            bool success = false;
            Eigen::VectorXd q = _q;
            Eigen::VectorXd v(_model->nv);
            Eigen::Matrix<double, 6, 1> err;
            pinocchio::Data::Matrix6x J(6, _model->nv);
            J.setZero();

            for (int i = 0;; i++) {
                // Update frame
                pinocchio::forwardKinematics(*_model, *_data, q);
                pinocchio::updateFramePlacement(*_model, *_data, FRAME_ID);

                // Compute SE3/se3 error
                const pinocchio::SE3 dMi = oMdes.actInv(_data->oMf[FRAME_ID]);
                err = pinocchio::log6(dMi).toVector();

                // Breaking condition
                if (err.norm() < eps) {
                    success = true;
                    break;
                }
                if (i >= IT_MAX) {
                    success = false;
                    break;
                }

                pinocchio::computeJointJacobians(*_model, *_data);
                pinocchio::getFrameJacobian(*_model, *_data, FRAME_ID, pinocchio::LOCAL, J);

                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += damp;
                v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
                q = pinocchio::integrate(*_model, q, v * DT);
                // if (!(i % 10))
                //     std::cout << i << ": error = " << err.transpose() << std::endl;
                // if (ref) null-space ik
                //     v.noalias() += w * (Eigen::MatrixXd::Identity(_model->nv, _model->nv) - tools::pseudoInverse(J) * J) * (*ref - q);
            }

            // if (success) {
            //     std::cout << "Convergence achieved!" << std::endl;
            // }
            // else {
            //     std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
            // }

            // std::cout << "\nresult: " << q.transpose() << std::endl;
            // std::cout << "\nfinal error: " << err.transpose() << std::endl;

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

        inline void MultiBody::clipForce(const int& index, double& force)
        {
            double maxForce = _body->getLink(index).m_jointMaxForce;

            if (force >= maxForce)
                force = maxForce;
            else if (force <= -maxForce)
                force = -maxForce;
        }
    } // namespace bodies
} // namespace beautiful_bullet