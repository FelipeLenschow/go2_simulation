#include "go2_actuator/go2_actuator.hpp"
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>

namespace go2_actuator
{

    Go2Actuator::Go2Actuator()
        : controller_interface::ControllerInterface(),
          joint_names_({})
    {
        q.resize(12);
        dq.resize(12);
        kp.resize(12);
        kd.resize(12);
        tau.resize(12);
        q_e.resize(12);
        dq_e.resize(12);
        qr.resize(12);
        dqr.resize(12);
    }

    controller_interface::CallbackReturn Go2Actuator::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", joint_names_);
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            auto_declare<double>("gain.Kp", 60.0);
            auto_declare<double>("gain.Kd", 5.0);
            auto_declare<std::vector<double>>("joints_references", {});
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    Go2Actuator::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        command_interfaces_config.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint : joint_names_)
        {
            for (const auto &interface_type : command_interface_types_)
            {
                command_interfaces_config.names.push_back(joint + "/" + interface_type);
            }
        }
        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration
    Go2Actuator::state_interface_configuration() const
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
    Go2Actuator::on_configure(
        const rclcpp_lifecycle::State &)
    {
        const auto logger = get_node()->get_logger();

        joint_names_ = get_node()->get_parameter("joints").as_string_array();

        if (joint_names_.empty())
        {
            RCLCPP_WARN(logger, "'joints' parameter is empty.");
        }

        // Command interface checking
        // Specialized, child actuatorss set interfaces before calling configure function.
        if (command_interface_types_.empty())
        {
            command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
        }

        if (command_interface_types_.empty())
        {
            RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
            return CallbackReturn::FAILURE;
        }

        for (const auto &interface : command_interface_types_)
        {
            auto it =
                std::find(allowed_command_interface_types_.begin(), allowed_command_interface_types_.end(), interface);
            if (it == allowed_command_interface_types_.end())
            {
                RCLCPP_ERROR(logger, "Command interface type '%s' not allowed! Only effort type is allowed!", interface.c_str());
                return CallbackReturn::FAILURE;
            }
        }

        joint_command_interface_.resize(allowed_command_interface_types_.size());

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
            logger, "Command interfaces are [%s] and and state interfaces are [%s].",
            get_interface_list(command_interface_types_).c_str(),
            get_interface_list(state_interface_types_).c_str());

        // Gains update //
        Kp_gain = get_node()->get_parameter("gain.Kp").get_value<double>();
        Kd_gain = get_node()->get_parameter("gain.Kd").get_value<double>();

        std::vector<double> joits_references = get_node()->get_parameter("joints_references").get_value<std::vector<double>>();

        for (int index = 0; index < 12; index++)
        {

            kp[index] = Kp_gain;
            kd[index] = Kd_gain;
            tau[index] = 0;
            qr[index] = joits_references[index];
            dqr[index] = 0;
        }

        joints_reference_subscriber_ = get_node()->create_subscription<lowCmd>(
            "~/LowCommands", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<lowCmd> msg) -> void
            {
                std::lock_guard<std::mutex> lock(this->mutex_actuator);
                for (int index = 0; index < 12; index++)
                {
                    qr[index] = msg->motor_cmd[index].q;
                    dqr[index] = msg->motor_cmd[index].dq;
                    kd[index] = msg->motor_cmd[index].kd;
                    kp[index] = msg->motor_cmd[index].kp;
                    tau[index] = msg->motor_cmd[index].tau;
                }
            });

        // joints_control_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/LowCommands", 10);

        RCLCPP_INFO(logger, "Impedance actuator gains update");

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2Actuator::on_activate(const rclcpp_lifecycle::State &)
    {

        const auto logger = get_node()->get_logger();
        RCLCPP_INFO(logger, "Activing impedance actuator");

        // order all joints in the storage
        for (const auto &interface : command_interface_types_)
        {
            auto it =
                std::find(allowed_command_interface_types_.begin(), allowed_command_interface_types_.end(), interface);
            auto index = std::distance(allowed_command_interface_types_.begin(), it);
            if (!controller_interface::get_ordered_interfaces(
                    command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", joint_names_.size(),
                    interface.c_str(), joint_command_interface_[index].size());
                return CallbackReturn::ERROR;
            }
        }

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

    controller_interface::CallbackReturn Go2Actuator::on_deactivate(const rclcpp_lifecycle::State &)
    {
        const auto logger = get_node()->get_logger();
        RCLCPP_INFO(logger, "Deactiveting impedance actuator");

        for (auto index = 0ul; index < joint_names_.size(); ++index)
        {
            (void)joint_command_interface_[0][index].get().set_value(
                joint_command_interface_[2][index].get().get_value());
        }

        for (auto index = 0ul; index < allowed_state_interface_types_.size(); ++index)
        {
            joint_command_interface_[index].clear();
            joint_state_interface_[index].clear();
        }
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type Go2Actuator::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        const auto logger = get_node()->get_logger();

        std::lock_guard<std::mutex> lock(this->mutex_actuator);
        {

            if (tau[0] != 0)
            {
                for (auto index{0}; index < 12; index++)
                {
                    (void)joint_command_interface_[0][index].get().set_value(tau[index]);
                }
            }
            else if (kp[0] != 0)
            {
                for (auto index{0}; index < 12; index++)
                {
                    // get the joint position
                    q[index] = joint_state_interface_[0][index].get().get_value();
                    // get the joint velocity
                    dq[index] = joint_state_interface_[1][index].get().get_value();

                    // compute the error position
                    q_e[index] = qr[index] - q[index];
                    dq_e[index] = dqr[index] - dq[index];

                    double tau_ = kp[index] * q_e[index] + kd[index] * dq_e[index];

                    (void)joint_command_interface_[0][index].get().set_value(tau_);
                }
            }
            else
            {
                std::cout << "ERROR: Kp, Kd and Tau are zero" << std::endl;
            }
        }
        return controller_interface::return_type::OK;
    }

}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_actuator::Go2Actuator,
    controller_interface::ControllerInterface)
