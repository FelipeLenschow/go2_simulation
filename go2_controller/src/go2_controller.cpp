#include "go2_controller/go2_controller.hpp"
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

namespace go2_controller
{

    Go2Controller::Go2Controller() 
    : controller_interface::ControllerInterface(), 
    joint_names_({}),
    model()
    {
        std::cout<<"Name from URDF"<< std::endl;
       
        const auto package_share_path = ament_index_cpp::get_package_share_directory("go2_description");
        const auto urdf_path = std::filesystem::path(package_share_path) / "urdf" / "go2.xacro.urdf";

        // Create a set of Pinocchio models and data.
        // pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);
        
        std::cout<<model.name<<std::endl;
        std::cout<<model.nq<<std::endl;
    }

    controller_interface::CallbackReturn Go2Controller::on_init()
    {
        // std::cout << "hey2" << std::endl;
        try
        {
            auto_declare<std::vector<std::string>>("joints", joint_names_);
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            auto_declare<double>("gain.Kp", 100.0);
            auto_declare<double>("gain.Kd", 10.0);
            auto_declare<std::vector<double>>("joints_references", {});

            auto_declare<double>("robot_states_feedabck_rate", 100.0);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    Go2Controller::command_interface_configuration() const
    {
        // std::cout << "hey3" << std::endl;
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
    Go2Controller::state_interface_configuration() const
    {
        // std::cout << "hey4" << std::endl;
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
    Go2Controller::on_configure(
        const rclcpp_lifecycle::State &)
    {
        // std::cout << "hey5" << std::endl;
        const auto logger = get_node()->get_logger();

        joint_names_ = get_node()->get_parameter("joints").as_string_array();

        if (joint_names_.empty())
        {
            RCLCPP_WARN(logger, "'joints' parameter is empty.");
        }

        // Command interface checking
        // Specialized, child controllers set interfaces before calling configure function.
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
        //         return CallbackReturn::SUCCESS;
        //     }
        // }

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
            "~/LowReferences", rclcpp::SystemDefaultsQoS(), 
            [this](const std::shared_ptr<lowCmd> msg) -> void
            {
                std::lock_guard<std::mutex> lock(this->mutex_controller);
                for (int index = 0; index < 12; index++)
                {
                    qr[index] = msg->motor_cmd[index].q;
                    dqr[index] = msg->motor_cmd[index].dq;
                    kd[index] = msg->motor_cmd[index].kd;
                    kp[index] = msg->motor_cmd[index].kp;
                    tau[index] = msg->motor_cmd[index].tau;
                }
            });

        joints_control_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/LowCommands", 10);

        RCLCPP_INFO(logger, "Impedance controller gains update");

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2Controller::on_activate(const rclcpp_lifecycle::State &)
    {

        const auto logger = get_node()->get_logger();
        RCLCPP_INFO(logger, "Activing impedance controller");

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

    controller_interface::CallbackReturn Go2Controller::on_deactivate(const rclcpp_lifecycle::State &)
    {
        const auto logger = get_node()->get_logger();
        RCLCPP_INFO(logger, "Deactiveting impedance controller");

        // TODO(anyone): How to halt when using effort commands?
        for (auto index = 0ul; index < joint_names_.size(); ++index)
        {
            joint_command_interface_[0][index].get().set_value(
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

    controller_interface::return_type Go2Controller::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        const auto logger = get_node()->get_logger();

        // At joint_state_interface_ are all the joints interfaces.
        // They separate by the name and type:
        //   0 - Position
        //   1 - Velocity
        //   2 - Effort

        std::lock_guard<std::mutex> lock(this->mutex_controller);

        for (auto index{0}; index < 12; index++)
        {

            q[index] = joint_state_interface_[0][index].get().get_value();

            // get the joint velocity
            dq[index] = joint_state_interface_[1][index].get().get_value();

            // calcule the position and velocity errors

            // std::cout << qr[index] << std::endl;
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];

            commanded_effort[index] = kp[index] * q_e[index] + kd[index] * dq_e[index];
            joint_command_interface_[0][index].get().set_value(commanded_effort[index]);

        }

        publish_joint_control_signal();

        return controller_interface::return_type::OK;
    }
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_controller::Go2Controller,
    controller_interface::ControllerInterface)
