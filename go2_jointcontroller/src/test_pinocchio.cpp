#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <iostream>

int main() {
    // Load the model from a URDF file
    pinocchio::Model model;
    pinocchio::urdf::buildModel("path/to/your/robot.urdf", model);
    pinocchio::Data data(model);

    // Define a configuration (random for now, you can set specific values)
    Eigen::VectorXd q = Eigen::VectorXd::Random(model.nq);

    // Define the index of the link you want the CoM Jacobian for
    std::string link_name = "FL_hip_joint";
    pinocchio::FrameIndex frame_id = model.getFrameId(link_name);

    if (frame_id == pinocchio::Model::FrameIndex(-1)) {
        std::cerr << "Frame not found!" << std::endl;
        return -1;
    }

    // Compute forward kinematics
    pinocchio::forwardKinematics(model, data, q);
    
    // Allocate space for the Jacobian (6 x nq)
    Eigen::MatrixXd J(6, model.nv);
    
    // Compute the CoM Jacobian of the specific link
    pinocchio::computeJointJacobian(model, data, q, frame_id, pinocchio::LOCAL, J);

    // Print the Jacobian
    std::cout << "Jacobian of CoM of link " << link_name << ":\n" << J << std::endl;

    return 0;
}
