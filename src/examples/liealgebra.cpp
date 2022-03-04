#include <iostream>

#include <Eigen/Core>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

int main(int argc, char const* argv[])
{
    // Model
    pinocchio::Model model;
    pinocchio::urdf::buildModel("models/iiwa_bullet/model.urdf", model);
    pinocchio::Data data(model);

    // Frame ID
    const std::string name = "lbr_iiwa_link_7";
    const int id = model.getFrameId(name);

    // Random joint position and velocity
    Eigen::Matrix<double, 7, 1> q = Eigen::VectorXd::Random(7),
                                v = Eigen::VectorXd::Random(7);

    // Forward Kinematics
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::framesForwardKinematics(model, data, q);

    // Jacobian
    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, id, J);

    // Pose
    pinocchio::SE3 pose = data.oMf[id];
    Eigen::Vector3d trans = pose.translation();
    Eigen::Matrix3d rot = pose.rotation();

    // Compare
    std::cout << "Jacobian derivation" << std::endl;
    std::cout << (J * v).transpose() << std::endl;

    std::cout << "Lie Algebra derivation" << std::endl;
    std::cout << pinocchio::log6(pose).toVector().transpose() << std::endl;

    return 0;
}
