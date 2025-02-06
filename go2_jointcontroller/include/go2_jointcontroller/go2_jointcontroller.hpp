#ifndef GO2_JOINTCONTROLLER__GO2_JOINTCONTROLLER_HPP_
#define GO2_JOINTCONTROLLER__GO2_JOINTCONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <string>
#include <mutex>

#include "std_msgs/msg/float64_multi_array.hpp"

#include "go2_interfaces/msg/low_state.hpp"
#include "go2_interfaces/msg/low_cmd.hpp"

#include <filesystem>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/parsers/urdf.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace go2_jointcontroller
{
    using lowCmd = go2_interfaces::msg::LowCmd;
    using lowStates = go2_interfaces::msg::LowState;

    class Go2JointController : public controller_interface::ControllerInterface
    {
    public:
        // GO2_JOINTCONTROLLER_PUBLIC
        Go2JointController();

        // GO2_JOINTCONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        // GO2_JOINTCONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // GO2_JOINTCONTROLLER_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // GO2_JOINTCONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        // GO2_JOINTCONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_JOINTCONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_JOINTCONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        void computeG();

        void computePD();

        uint32_t crc32_core(uint32_t *ptr, uint32_t len);

    protected:
        std::vector<std::string> joint_names_;

        pinocchio::Model model;
        std::shared_ptr<pinocchio::Data> data;

        Eigen::VectorXd gravidade;
        Eigen::VectorXd q;
        Eigen::VectorXd dq;
        Eigen::VectorXd kp;
        Eigen::VectorXd kd;
        Eigen::VectorXd tauG;
        Eigen::VectorXd tau;
        Eigen::VectorXd q_e;
        Eigen::VectorXd dq_e;
        Eigen::VectorXd qr;
        Eigen::VectorXd dqr;
        Eigen::VectorXd effort;
        Eigen::VectorXd commanded_effort;

        Eigen::VectorXd v = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(12);

        lowCmd lowCmd_msg;

        rclcpp::Publisher<lowCmd>::SharedPtr joints_cmd_publisher_;

        rclcpp::Subscription<lowCmd>::SharedPtr controller_reference_subscriber_;
        rclcpp::Subscription<lowStates>::SharedPtr lowstate_subscriber_;

        uint32_t control_mode = 0;

        std::mutex mutex_controller;
    };

}
#endif