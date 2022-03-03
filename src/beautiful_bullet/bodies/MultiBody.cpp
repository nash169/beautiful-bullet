#include "beautiful_bullet/bodies/MultiBody.hpp"

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <control_lib/controllers/Feedback.hpp>

namespace beautiful_bullet {
    namespace bodies {
        MultiBody::MultiBody(const std::string& file, int flags)
        {
            // Create loader
            // _loader = std::make_shared<utils::BulletLoader>();

            // Get multibody
            _body = _loader.parseMultiBody(file, flags);

            // Init agent internal state variable
            _q.setZero(_body->getNumDofs());
            _v.setZero(_body->getNumDofs());

            // Store inertia frame of the root node
            // (not keeping track of it at the moment)
            _rootFrame = _body->getBaseWorldTransform();

            // Pinocchio model
            _model = new pinocchio::Model();
            pinocchio::urdf::buildModel(file, *_model);

            // Pinocchio data
            _data = new pinocchio::Data(*_model); // _model.gravity.linear(Eigen::Vector3d(0, 0, -9.81));

            // Update pinocchio forward kinematics
            pinocchio::forwardKinematics(*_model, *_data, _q, _v);
        }

        MultiBody::MultiBody(MultiBody&& other) noexcept
        {
            // Move (copy) state
            _q = other._q;
            _v = other._v;

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

        btMultiBody* MultiBody::body() { return _body; }

        const Eigen::VectorXd& MultiBody::state() { return _q; }

        const Eigen::VectorXd& MultiBody::velocity() { return _v; }

        Eigen::Matrix<double, 6, 1> MultiBody::poseJoint(const int& index)
        {
            /* This might be very slow (find a more efficient way) */
            // Update pinocchio state
            pinocchio::forwardKinematics(*_model, *_data, _q, _v);

            // Get joint pose
            pinocchio::SE3 oMi = _data->oMi[(index == -1 || index >= _model->nv) ? _model->nv - 1 : index];
            Eigen::AngleAxisd rot(oMi.rotation());

            // std::cout << oMi.translation().transpose() << std::endl;
            // std::cout << oMi.rotation() << std::endl;

            return (Eigen::Matrix<double, 6, 1>() << oMi.translation(), rot.angle() * rot.axis()).finished();
        }

        Eigen::MatrixXd MultiBody::jacobian(const int& index)
        {
            pinocchio::Data::Matrix6x J(6, _model->nv);
            J.setZero();
            pinocchio::computeJointJacobian(*_model, *_data, _q, (index == -1 || index >= _model->nv) ? _model->nv - 1 : index, J);

            return J;
        }

        MultiBody& MultiBody::setBody(btMultiBody* body) // (remember to init state here)
        {
            _body = body;

            return *this;
        };

        MultiBody& MultiBody::setPosition(const double& x, const double& y, const double& z)
        {
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

        /* Add controllers */
        template <typename... Args>
        MultiBody& MultiBody::addControllers(std::unique_ptr<control::MultiBodyCtr> controller, Args... args)
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
        Eigen::VectorXd MultiBody::inverseKinematics(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const std::string& frame)
        {
            const int FRAME_ID = _model->existFrame(frame) ? _model->getFrameId(frame) : _model->nframes - 1;

            // Desired Pose
            const pinocchio::SE3 oMdes(orientation, position);

            // Jacobian
            pinocchio::Data::Matrix6x J(6, _model->nv);
            J.setZero();

            // Optim params
            const int IT_MAX = 1000;
            const double eps = 1e-4, DT = 1e-1, damp = 1e-6;

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
            // Get joint pose and vel
            for (size_t i = 0; i < _body->getNumDofs(); i++) {
                _q(i) = _body->getJointPos(i);
                _v(i) = _body->getJointVel(i);
            }

            // Command force
            Eigen::VectorXd tau = Eigen::VectorXd::Zero(_body->getNumDofs());

            // Gravity compensation
            tau += pinocchio::nonLinearEffects(*_model, *_data, _q, _v); // pinocchio::computeGeneralizedGravity(*_model, *_data, _q); //
            // std::cout << "Force before: " << tau.transpose() << std::endl;

            // Control
            for (auto& controller : _controllers) {
                if (controller->mode() == ControlMode::CONFIGURATIONSPACE)
                    tau += controller->action(*this);
                else if (controller->mode() == ControlMode::OPERATIONSPACE) {
                    // Goal
                    Eigen::Vector3d xDes(0.365308, -0.0810892, 1.13717);
                    Eigen::Matrix3d oDes;
                    oDes << 0.591427, -0.62603, 0.508233,
                        0.689044, 0.719749, 0.0847368,
                        -0.418848, 0.300079, 0.857041;

                    // Update kinematics and frames
                    pinocchio::framesForwardKinematics(*_model, *_data, _q);

                    // Poses
                    const int id = _model->getFrameId("lbr_iiwa_link_7");
                    pinocchio::SE3 poseCurr = _data->oMf[id],
                                   poseDes(oDes, xDes);

                    // Get Jacobian
                    pinocchio::Data::Matrix6x J(6, _model->nv);
                    J.setZero();
                    pinocchio::computeFrameJacobian(*_model, *_data, _q, id, J);

                    // Tangent space
                    Eigen::Matrix<double, 6, 1> v1_p = pinocchio::log6(poseCurr).toVector(),
                                                v2_p = pinocchio::log6(poseDes).toVector(),
                                                err_p = pinocchio::log6(poseDes.actInv(poseCurr)).toVector();

                    Eigen::Matrix<double, 6, 1> t1, t2, err_m;
                    t1.head(3) = poseCurr.translation();
                    t1.tail(3) = Eigen::AngleAxisd(poseCurr.rotation()).angle() * Eigen::AngleAxisd(poseCurr.rotation()).axis();
                    t2.head(3) = poseDes.translation();
                    t2.tail(3) = Eigen::AngleAxisd(poseDes.rotation()).angle() * Eigen::AngleAxisd(poseDes.rotation()).axis();
                    control_lib::utils::ControlState v1_m(6, control_lib::ControlSpace::LINEAR | control_lib::ControlSpace::ANGLEAXIS),
                        v2_m(6, control_lib::ControlSpace::LINEAR | control_lib::ControlSpace::ANGLEAXIS);
                    v1_m.setState(t1);
                    v2_m.setState(t2);
                    err_m = (v1_m - v2_m).getPos();

                    // Rotation matrix
                    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
                    R.block(0, 0, 3, 3) = poseCurr.rotation();
                    R.block(3, 3, 3, 3) = poseCurr.rotation();
                    // J = R * J;

                    // Dynamics
                    Eigen::MatrixXd A = -0.1 * Eigen::MatrixXd::Identity(6, 6);
                    Eigen::VectorXd x_dot = A * err_p;

                    // Controller
                    double k = 1;
                    tau -= k * J.transpose() * (J * _v - x_dot);
                    // tau += J.transpose() * controller->control(poseJoint(controller->controlRef()), J * _v);
                }
            }

            // Set gravity compensation
            for (size_t i = 0; i < _body->getNumDofs(); i++) {
                // Clip force
                clipForce(i, tau(i));

                // Apply force
                _body->addJointTorque(i, tau(i));
            }
        }
    } // namespace bodies
} // namespace beautiful_bullet