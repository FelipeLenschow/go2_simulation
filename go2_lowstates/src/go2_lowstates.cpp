#include "go2_lowstates/go2_lowstates.hpp"
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "rclcpp/qos.hpp"
// #include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
// #include "rclcpp_action/create_server.hpp"
// #include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace go2_lowstates
{

    Go2Lowstates::Go2Lowstates() : controller_interface::ControllerInterface(), joint_names_({})
    {
    }

    controller_interface::CallbackReturn Go2Lowstates::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", joint_names_);

            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    Go2Lowstates::command_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::InterfaceConfiguration
    Go2Lowstates::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        state_interfaces_config.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : state_interface_types_)
            {
                state_interfaces_config.names.push_back(joint_name + "/" + interface_type);
            }
        }
        return state_interfaces_config;
    }

    controller_interface::CallbackReturn
    Go2Lowstates::on_configure(
        const rclcpp_lifecycle::State &)
    {
        const auto logger = get_node()->get_logger();

        joint_names_ = get_node()->get_parameter("joints").as_string_array();

        if (joint_names_.empty())
        {
            RCLCPP_WARN(logger, "'joints' parameter is empty.");
        }

        // State interface checking
        state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();

        if (state_interface_types_.empty())
        {
            RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
            return CallbackReturn::FAILURE;
        }

        joint_state_interface_.resize(allowed_state_interface_types_.size());

        // Pritig format
        auto get_interface_list = [](const std::vector<std::string> &interface_types)
        {
            std::stringstream ss_interfaces;
            for (size_t index = 0; index < interface_types.size(); ++index)
            {
                if (index != 0)
                {
                    ss_interfaces << " ";
                }
                ss_interfaces << interface_types[index];
            }
            return ss_interfaces.str();
        };

        RCLCPP_INFO(
            logger, "State interfaces are [%s].",
            get_interface_list(state_interface_types_).c_str());

        imu_subscriber_ = get_node()->create_subscription<imuStates>(
            "imu", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<imuStates> msg) -> void
            {
                std::lock_guard<std::mutex> lock(this->mutex_controller);

                lowStates_msg.imu_state.quaternion[0] = msg->orientation.x;
                lowStates_msg.imu_state.quaternion[1] = msg->orientation.y;
                lowStates_msg.imu_state.quaternion[2] = msg->orientation.z;
                lowStates_msg.imu_state.quaternion[3] = msg->orientation.w;
                lowStates_msg.imu_state.gyroscope[0] = msg->angular_velocity.x;
                lowStates_msg.imu_state.gyroscope[1] = msg->angular_velocity.y;
                lowStates_msg.imu_state.gyroscope[2] = msg->angular_velocity.z;
                lowStates_msg.imu_state.accelerometer[0] = msg->angular_velocity.x;
                lowStates_msg.imu_state.accelerometer[1] = msg->angular_velocity.y;
                lowStates_msg.imu_state.accelerometer[2] = msg->angular_velocity.z;

                tf2::Quaternion q(
                    lowStates_msg.imu_state.quaternion[0],
                    lowStates_msg.imu_state.quaternion[1],
                    lowStates_msg.imu_state.quaternion[2],
                    lowStates_msg.imu_state.quaternion[3]);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                lowStates_msg.imu_state.rpy[0] = roll;
                lowStates_msg.imu_state.rpy[1] = pitch;
                lowStates_msg.imu_state.rpy[2] = yaw;
            });

        // effort_subscriber_ = get_node()->create_subscription<effortstates>(
        //     "/go2_controller/LowCommands", rclcpp::SystemDefaultsQoS(),
        //     [this](const std::shared_ptr<effortstates> msg) -> void
        //     {
        //         std::lock_guard<std::mutex> lock(this->mutex_controller);
        //         for (auto index{0}; index < 12; index++)
        //         {
        //             tau[index] = msg->data[index];
        //         }
        //     });

        go2_lowstates_publisher = get_node()->create_publisher<lowStates>("go2_lowstates/LowStates", 10);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2Lowstates::on_activate(const rclcpp_lifecycle::State &)
    {
        const auto logger = get_node()->get_logger();

        RCLCPP_INFO(logger, "Activing Low State node");

        for (const auto &interface : state_interface_types_)
        {
            auto it =
                std::find(allowed_state_interface_types_.begin(), allowed_state_interface_types_.end(), interface);

            auto index = std::distance(allowed_state_interface_types_.begin(), it);

            if (!controller_interface::get_ordered_interfaces(
                    state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", joint_names_.size(),
                    interface.c_str(), joint_state_interface_[index].size());
                return CallbackReturn::ERROR;
            }
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2Lowstates::on_deactivate(const rclcpp_lifecycle::State &)
    {
        const auto logger = get_node()->get_logger();
        RCLCPP_INFO(logger, "Deactiveting Low State node");

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type Go2Lowstates::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        const auto logger = get_node()->get_logger();
        std::lock_guard<std::mutex> lock(this->mutex_controller);
        Go2Lowstates::get_joints_info();

        // publish_joint_control_signal();
        go2_lowstates_publisher->publish(lowStates_msg);

        return controller_interface::return_type::OK;
    }

    void Go2Lowstates::get_joints_info()
    {
        // std::lock_guard<std::mutex> lock(this->mutex_controller);

        for (auto index{0}; index < 12; index++)
        {
            // get the joint position

            lowStates_msg.motor_state[index].q = joint_state_interface_[0][index].get().get_value();

            // get the joint velocity
            lowStates_msg.motor_state[index].dq = joint_state_interface_[1][index].get().get_value();

            // get the joint acceleration
            // lowStates_msg.motor_state[index].ddq = joint_state_interface_[2][index].get().get_value();

            // get the joint effort
            lowStates_msg.motor_state[index].tau_est = tau[index];
        }
    }
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_lowstates::Go2Lowstates,
    controller_interface::ControllerInterface)