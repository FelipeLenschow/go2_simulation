#ifndef GO2_ACTUATOR__GO2_ACTUATOR_HPP_
#define GO2_ACTUATOR__GO2_ACTUATOR_HPP_

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "unitree_go/msg/low_cmd.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

// TODO: remove pinocchio pededencies an replace for Eigen
#include <filesystem>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/parsers/urdf.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace go2_actuator
{
    using lowCmd = unitree_go::msg::LowCmd;

    class Go2Actuator : public controller_interface::ControllerInterface
    {
    public:
        // GO2_ACTUATOR_PUBLIC
        Go2Actuator();

        // GO2_ACTUATOR_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        // GO2_ACTUATOR_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // GO2_ACTUATOR_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // GO2_ACTUATOR_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        // GO2_ACTUATOR_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_ACTUATOR_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_ACTUATOR_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

    //     pinocchio::Model model;
    //     std::shared_ptr<pinocchio::Data> data;

        Eigen::VectorXd q;
        Eigen::VectorXd dq;
        Eigen::VectorXd kp;
        Eigen::VectorXd kd;
        Eigen::VectorXd tau;
        Eigen::VectorXd qr;
        Eigen::VectorXd dqr;

        double Kp_gain;
        double Kd_gain;

        const std::vector<std::string> allowed_state_interface_types_ = {
            hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY,
            hardware_interface::HW_IF_EFFORT,
        };

        const std::vector<std::string> allowed_command_interface_types_ = {
            hardware_interface::HW_IF_EFFORT,
        };

        template <typename T>
        using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

        InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
        InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

        std::mutex mutex_actuator;

        rclcpp::Subscription<lowCmd>::SharedPtr joints_reference_subscriber_;
    //     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_control_publisher_;

    //     float _startPos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};


    };

}
#endif