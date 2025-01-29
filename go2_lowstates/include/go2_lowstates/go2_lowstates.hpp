#ifndef GO2_LOWSTATES__GO2_LOWSTATES_HPP_
#define GO2_LOWSTATES__GO2_LOWSTATES_HPP_

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "go2_interfaces/msg/low_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace go2_lowstates
{
    using lowStates = go2_interfaces::msg::LowState;
    using imuStates = sensor_msgs::msg::Imu;
    using effortstates = std_msgs::msg::Float64MultiArray;
    using footforce =

        class Go2Lowstates : public controller_interface::ControllerInterface
    {
    public:
        // GO2_LOWSTATES_PUBLIC
        Go2Lowstates();

        // JOINT_STATE_BROADCASTER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        // GO2_LOWSTATES_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // GO2_LOWSTATES_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // GO2_LOWSTATES_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        // GO2_LOWSTATES_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_LOWSTATES_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        // GO2_LOWSTATES_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        // // GO2_CONTROLLER_PUBLIC
        // controller_interface::CallbackReturn on_cleanup(
        //     const rclcpp_lifecycle::State &previous_state) override;

        // // GO2_CONTROLLER_PUBLIC
        // controller_interface::CallbackReturn on_error(
        //     const rclcpp_lifecycle::State &previous_state) override;

        // // GO2_CONTROLLER_PUBLIC
        // controller_interface::CallbackReturn on_shutdown(
        //     const rclcpp_lifecycle::State &previous_state) override;

    private:
        void get_joints_info();

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        double q[12];
        double dq[12];
        double effort[12];
        double tau[12];

        double commanded_effort[12];
        double q_e[12];
        double dq_e[12];

        double qr[12];
        double dqr[12];

        const std::vector<std::string> allowed_state_interface_types_ = {
            hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY,
            hardware_interface::HW_IF_EFFORT,
        };

        // const std::vector<std::string> allowed_command_interface_types_ = {
        //     hardware_interface::HW_IF_EFFORT,
        // };

        template <typename T>
        using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

        InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
        InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

        std::mutex mutex_controller;

        lowStates lowStates_msg;

        rclcpp::Publisher<lowStates>::SharedPtr go2_lowstates_publisher;
        rclcpp::Subscription<imuStates>::SharedPtr imu_subscriber_;       // subscrevendo as informações do tipo imuStates no ponteiro imu_subscriber_
        rclcpp::Subscription<effortstates>::SharedPtr effort_subscriber_; // subscrevendo as informações do tipo std_msgs.... no ponteiro effort_subscriber_
    };

}
#endif