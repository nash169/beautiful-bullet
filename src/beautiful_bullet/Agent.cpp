#include "beautiful_bullet/Agent.hpp"

// Pinocchio
#ifdef USE_PINOCCHIO
#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#endif

namespace beautiful_bullet {
    /* Constructor */
    Agent::Agent(const std::string& file, int flags)
    {
        // Create loader
        // _loader = std::make_shared<utils::BulletLoader>();

        // Get multibody
        _body = _loader.parseMultiBody(file, flags);

// Pinocchio model
#ifdef USE_PINOCCHIO
        _model = new pinocchio::Model();
        pinocchio::urdf::buildModel(file, *_model);
        // std::cout << _model->nv << std::endl;
        _data = new pinocchio::Data(*_model); // _model.gravity.linear(Eigen::Vector3d(0, 0, -9.81));
#endif

        // Init agent internal state variable
        _q.setZero(_body->getNumDofs());
        _v.setZero(_body->getNumDofs());

        // Store inertia frame of the root node
        _rootFrame = _body->getBaseWorldTransform();
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
#ifdef USE_PINOCCHIO
        _data = other._data;
        other._data = nullptr;

        _model = other._model;
        other._model = nullptr;
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

    /* Destroyer */
    Agent::~Agent() // (not needed beacause of smart pointer; try to move towards raw or unique_ptr)
    {
        _controllers.clear();
        _controllers.shrink_to_fit();

#ifdef USE_PINOCCHIO
        delete _data;
        delete _model;
#endif
    }

#ifdef USE_PINOCCHIO
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> Agent::poseJoint(const size_t& index)
    {
        pinocchio::SE3 oMi = _data->oMi[(!index || index >= _model->nv) ? _model->nv - 1 : index];

        return std::make_pair(oMi.translation_impl(), oMi.rotation_impl());
    }
#endif

#ifdef USE_PINOCCHIO
    /* Inverse Kinematics */
    Eigen::VectorXd Agent::inverseKinematics(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const size_t& index)
    {
        const int JOINT_ID = (!index || index >= _model->njoints) ? _model->njoints - 1 : index;

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
            pinocchio::forwardKinematics(*_model, *_data, _q);

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
            pinocchio::computeJointJacobian(*_model, *_data, _q, JOINT_ID, J);

            // Calculate damped pseudo-inverse (here why not applying Mantegazza method?)
            pinocchio::Data::Matrix6 JJt;
            JJt.noalias() = J * J.transpose();
            JJt.diagonal().array() += damp;

            // Calculate velocity
            v.noalias() = -J.transpose() * JJt.ldlt().solve(err);

            // Calculate configuration
            q = pinocchio::integrate(*_model, _q, v * DT);

            if (!(i % 10))
                std::cout << i << ": error = " << err.transpose() << std::endl;
        }

        return q;
    }
#endif

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
#ifdef USE_PINOCCHIO
        tau += pinocchio::nonLinearEffects(*_model, *_data, _q, _v); // pinocchio::computeGeneralizedGravity(_model, _data, _q);
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
} // namespace beautiful_bullet
