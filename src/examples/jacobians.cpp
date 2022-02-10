#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <control_lib/controllers/Feedback.hpp>

using namespace control_lib;

int main(int argc, char const* argv[])
{
    pinocchio::Model model;
    pinocchio::urdf::buildModel("models/iiwa_bullet/model.urdf", model);
    pinocchio::Data data(model);
    pinocchio::Data::Matrix6x J1(6, model.nv), J2(6, model.nv), J3(6, model.nv), J4(6, model.nv);

    const std::string name = "lbr_iiwa_link_7";
    const int id = model.getFrameId(name);
    Eigen::VectorXd q(7);
    q << 0., 0.7, 0.4, 0.6, 0.3, 0.5, 0.1;

    pinocchio::framesForwardKinematics(model, data, Eigen::VectorXd::Zero(7));
    pinocchio::SE3 initPose = data.oMf[id];
    pinocchio::framesForwardKinematics(model, data, q);
    pinocchio::SE3 endPose = data.oMf[id];

    std::cout << data.oMf.size() << std::endl;

    // pinocchio::forwardKinematics(*_model, *_data, _q, _v);
    if (model.existFrame(name))
        std::cout << "Frame ID: " << model.getFrameId(name) << std::endl;
    if (model.existJointName(name))
        std::cout << "Joint ID: " << model.getJointId(name) << std::endl;
    if (model.existBodyName(name))
        std::cout << "Body ID: " << model.getBodyId(name) << std::endl;

    std::cout << "Initial Pose" << std::endl;
    std::cout << initPose.translation().transpose() << std::endl;
    std::cout << initPose.rotation() << std::endl;

    std::cout << "Final Pose" << std::endl;
    std::cout << endPose.translation().transpose() << std::endl;
    std::cout << endPose.rotation() << std::endl;

    // Joint Jacobian
    J1.setZero();
    pinocchio::computeJointJacobian(model, data, q, 7, J1);
    std::cout << "J1" << std::endl;
    std::cout << J1 << std::endl;

    // Full joint Jacobian
    J2.setZero();
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::getJointJacobian(model, data, 7, pinocchio::LOCAL, J2);
    std::cout << "J2" << std::endl;
    std::cout << J2 << std::endl;

    // Frame Jacobian
    J3.setZero();
    pinocchio::computeFrameJacobian(model, data, q, id, J3);
    std::cout << "J3" << std::endl;
    std::cout << J3 << std::endl;

    // Full frame Jacobians
    // pinocchio::getFrameJacobian(*_model, *_data, _model->getFrameId("lbr_iiwa_joint_7"), pinocchio::LOCAL, J);

    std::cout << "Jacobian pose: " << (J3 * (q - Eigen::VectorXd::Zero(7))).transpose() << std::endl;
    std::cout << "se3 pose: " << pinocchio::log6(endPose.actInv(initPose)).toVector().transpose() << std::endl;

    control_lib::utils::ControlState poseCurr(6, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS),
        poseDes(6, ControlSpace::LINEAR | ControlSpace::ANGLEAXIS);

    Eigen::Matrix<double, 6, 1> v1, v2;
    v1.head(3) = initPose.translation();
    v1.tail(3) = Eigen::AngleAxisd(initPose.rotation()).angle() * Eigen::AngleAxisd(initPose.rotation()).axis();
    v2.head(3) = endPose.translation();
    v2.tail(3) = Eigen::AngleAxisd(endPose.rotation()).angle() * Eigen::AngleAxisd(endPose.rotation()).axis();

    poseCurr.setState(v1);
    poseDes.setState(v2);
    std::cout << (poseCurr - poseDes).getPos().transpose() << std::endl;

    return 0;
}
