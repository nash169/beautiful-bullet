#include "beautiful_bullet/Agent.hpp"

// Pinocchio
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace beautiful_bullet {
    /* Constructor */
    Agent::Agent(const std::string& file, int flags)
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

    /* Move constructor */
    Agent::Agent(Agent&& other) noexcept
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

    /* Destroyer */
    Agent::~Agent() // (not needed beacause of smart pointer; try to move towards raw or unique_ptr)
    {
        _controllers.clear();
        _controllers.shrink_to_fit();

        // Delete pinocchio objects
        delete _data;
        delete _model;
    }

    /* Set agent (base) position */
    Agent& Agent::setPosition(const double& x, const double& y, const double& z)
    {
        _body->setBasePos(btVector3(x, y, z) + _rootFrame.getOrigin());

        btAlignedObjectArray<btQuaternion> scratch_q;
        btAlignedObjectArray<btVector3> scratch_m;
        _body->forwardKinematics(scratch_q, scratch_m);
        _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

        return *this;
    }

    /* Set agent (base) orientation */
    Agent& Agent::setOrientation(const double& roll, const double& pitch, const double& yaw)
    {
        _body->setWorldToBaseRot(btQuaternion(yaw, pitch, roll));

        btAlignedObjectArray<btQuaternion> scratch_q;
        btAlignedObjectArray<btVector3> scratch_m;
        _body->forwardKinematics(scratch_q, scratch_m);
        _body->updateCollisionObjectWorldTransforms(scratch_q, scratch_m);

        return *this;
    }

    /* Set agent state */
    Agent& Agent::setState(const Eigen::VectorXd& q)
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
    Agent& Agent::setVelocity(const Eigen::VectorXd& v)
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

    Eigen::MatrixXd Agent::jacobian(const int& index)
    {
        pinocchio::Data::Matrix6x J(6, _model->nv);
        J.setZero();
        pinocchio::computeJointJacobian(*_model, *_data, _q, (index == -1 || index >= _model->nv) ? _model->nv - 1 : index, J);

        return J;
    }

    /* This might be very slow (find a more efficient way) */
    Eigen::Matrix<double, 6, 1> Agent::poseJoint(const int& index)
    {
        // Update pinocchio state
        pinocchio::forwardKinematics(*_model, *_data, _q, _v);

        // Get joint pose
        pinocchio::SE3 oMi = _data->oMi[(index == -1 || index >= _model->nv) ? _model->nv - 1 : index];
        Eigen::AngleAxisd rot(oMi.rotation());

        // std::cout << oMi.translation().transpose() << std::endl;
        // std::cout << oMi.rotation() << std::endl;

        return (Eigen::Matrix<double, 6, 1>() << oMi.translation(), rot.angle() * rot.axis()).finished();
    }

    /* Inverse Kinematics */
    Eigen::VectorXd Agent::inverseKinematics(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const int& index)
    {
        const int JOINT_ID = (index == -1 || index >= _model->nv) ? _model->nv - 1 : index;

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
            // Compute the forward kinematics
            pinocchio::forwardKinematics(*_model, *_data, q);

            // Compute error between current and desired pose
            const pinocchio::SE3 dMi = oMdes.actInv(_data->oMi[JOINT_ID]);
            err = pinocchio::log6(dMi).toVector();

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
            pinocchio::computeJointJacobian(*_model, *_data, q, JOINT_ID, J);

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
    void Agent::update()
    {
        // Get joint pose and vel
        for (size_t i = 0; i < _body->getNumDofs(); i++) {
            _q(i) = _body->getJointPos(i);
            _v(i) = _body->getJointVel(i);
        }

        // Command force
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(_body->getNumDofs());

        // Gravity compensation
        tau += pinocchio::nonLinearEffects(*_model, *_data, _q, _v); // pinocchio::computeGeneralizedGravity(_model, _data, _q);

        // Control
        for (auto& controller : _controllers) {
            if (controller->controlMode() == ControlMode::CONFIGURATIONSPACE)
                tau += controller->control(_q, _v);
            else if (controller->controlMode() == ControlMode::OPERATIONSPACE) {
                Eigen::Vector3d xDes(0.324141, -0.0879529, 1.06775);

                Eigen::Matrix3d oDes;
                oDes << -0.650971, 0.508233, -0.563859,
                    -0.613746, 0.0847368, 0.784943,
                    0.446713, 0.857041, 0.256765;

                // pinocchio::forwardKinematics(*_model, *_data, _q, _v);
                pinocchio::computeJointJacobians(*_model, *_data, _q);
                pinocchio::Data::Matrix6x J(6, _model->nv);
                J.setZero();
                // pinocchio::computeJointJacobian(*_model, *_data, _q, 6, J);
                pinocchio::getJointJacobian(*_model, *_data, 6, pinocchio::WORLD, J);

                pinocchio::SE3 poseDes(oDes, xDes),
                    poseCurr = _data->oMi[6];

                double k = .01, dt = .01;
                Eigen::Matrix<double, 6, 1> err = pinocchio::log6(poseDes.actInv(poseCurr)),
                                            oldPose = pinocchio::log6(poseCurr);

                err.array().tail(3) = 0;
                oldPose.array().tail(3) = 0;

                Eigen::MatrixXd A = -0.1 * Eigen::MatrixXd::Identity(6, 6);

                Eigen::VectorXd x_dot = A * err;

                tau += J.transpose() * k * (oldPose + dt * x_dot);
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
} // namespace beautiful_bullet
