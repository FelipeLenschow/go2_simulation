#include "go2_jointcontroller/go2_jointcontroller.hpp"
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

namespace go2_jointcontroller
{

    Go2JointController::Go2JointController()
        : controller_interface::ControllerInterface(),
          joint_names_({}),
          model()
    {
        const auto package_share_path = ament_index_cpp::get_package_share_directory("go2_description");
        const auto urdf_path = std::filesystem::path(package_share_path) / "urdf" / "go2.xacro.urdf";

        std::cout << urdf_path << std::endl;

        // Create a set of Pinocchio models and data.
        pinocchio::urdf::buildModel(urdf_path, model);

        data = std::make_shared<pinocchio::Data>(model);

        std::cout << model.name << std::endl;

        for (int i = 0; i < model.nframes; i++)
        {
            std::cout << model.names[i] << std::endl;
            std::cout << "---" << std::endl;
        }
    }

    controller_interface::CallbackReturn Go2JointController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", joint_names_);
            auto_declare<double>("gain.Kp", 100.0);
            auto_declare<double>("gain.Kd", 10.0);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }
        gravidade.resize(3);
        q.resize(12);
        dq.resize(12);
        kp.resize(12);
        kd.resize(12);
        tau.resize(12);
        tauG.resize(12);
        q_e.resize(12);
        dq_e.resize(12);
        qr.resize(12);
        dqr.resize(12);
        effort.resize(12);
        commanded_effort.resize(12);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    Go2JointController::command_interface_configuration() const
    {
        return {controller_interface::interface_configuration_type::NONE};
    }
    controller_interface::InterfaceConfiguration
    Go2JointController::state_interface_configuration() const
    {
        return {controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::CallbackReturn
    Go2JointController::on_configure(
        const rclcpp_lifecycle::State &)
    {

        auto logger = get_node()->get_logger();

        // Read parameters from ROS parameter server

        joint_names_ = get_node()->get_parameter("joints").as_string_array();
        if (joint_names_.empty())
        {
            RCLCPP_ERROR(logger, "No joint names found in parameters.");
            return CallbackReturn::FAILURE;
        }
        auto kp_gain_ = get_node()->get_parameter("gain.Kp").get_value<double>();
        auto kd_gain_ = get_node()->get_parameter("gain.Kd").get_value<double>();

        // Initialize values
        for (size_t i = 0; i < 12; i++)
        {
            kp[i] = kp_gain_;
            kd[i] = kd_gain_;
            tau[i] = 0.0;
        }

        lowstate_subscriber_ = get_node()->create_subscription<lowStates>(
            "go2_lowstates/LowStates", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<lowStates> msg) -> void
            {
                std::lock_guard<std::mutex> lock(this->mutex_controller);

                for (auto index{0}; index < 12; index++)
                {
                    q[index] = msg->motor_state[index].q;

                    dq[index] = msg->motor_state[index].dq;

                    tau[index] = msg->motor_state[index].tau_est;
                }
            });

        controller_reference_subscriber_ = get_node()->create_subscription<lowCmd>(
            "~/JointControllerReferences", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<lowCmd> msg) -> void

            {
                std::lock_guard<std::mutex> lock(this->mutex_controller);
                control_mode = msg->reserve;
                for (int index = 0; index < 12; index++)
                {
                    qr[index] = msg->motor_cmd[index].q;
                    dqr[index] = msg->motor_cmd[index].dq;
                    kd[index] = msg->motor_cmd[index].kd;
                    kp[index] = msg->motor_cmd[index].kp;
                    tau[index] = msg->motor_cmd[index].tau;
                }
            });

        joints_cmd_publisher_ = get_node()->create_publisher<lowCmd>("~/LowCommands", 10);

        lowCmd_msg.head[0] = 0xFE;
        lowCmd_msg.head[1] = 0xEF;
        lowCmd_msg.level_flag = 0xFF;
        lowCmd_msg.gpio = 0;

        for (int i = 0; i < 12; i++)
        {
            lowCmd_msg.motor_cmd[i].mode = 0x01; // Modo servo (PMSM)
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2JointController::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating Go2JointController...");

        // Reset torques and errors
        for (auto &t : tau)
            t = 0.0;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2JointController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating Go2JointController...");

        // Stop sending commands
        for (auto &t : tau)
            t = 0.0;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type Go2JointController::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        std::lock_guard<std::mutex> lock(this->mutex_controller);
        {

            try
            {
                switch (control_mode)
                {
                case 1: // PD only
                    for (int j = 0; j < 12; j++)
                    {
                        lowCmd_msg.motor_cmd[j].q = qr[j];
                        lowCmd_msg.motor_cmd[j].dq = dqr[j];
                        lowCmd_msg.motor_cmd[j].kp = kp[j];
                        lowCmd_msg.motor_cmd[j].kd = kd[j];
                        lowCmd_msg.motor_cmd[j].tau = 0;
                    }
                    break;

                case 2: // PD + Gravity Compensation
                    computePD();
                    computeG();
                    for (int j = 0; j < 12; j++)
                    {
                        lowCmd_msg.motor_cmd[j].q = 0;
                        lowCmd_msg.motor_cmd[j].dq = 0;
                        lowCmd_msg.motor_cmd[j].kp = 0;
                        lowCmd_msg.motor_cmd[j].kd = 0;
                        lowCmd_msg.motor_cmd[j].tau = commanded_effort[j] + tauG[j];
                    }
                    break;

                default:
                    RCLCPP_ERROR(get_node()->get_logger(), "Invalid control mode: %d", control_mode);
                    return controller_interface::return_type::ERROR;
                }

                // Publish the control message
                lowCmd_msg.crc = crc32_core((uint32_t *)&lowCmd_msg, (sizeof(lowCmd) >> 2) - 1);
                joints_cmd_publisher_->publish(lowCmd_msg);

                return controller_interface::return_type::OK;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Exception in update(): %s", e.what());
                return controller_interface::return_type::ERROR;
            }
        }
    }

    void Go2JointController::computeG()
    {

        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd tau = pinocchio::rnea(model, *data, q, v, a);
        tauG = data->tau.transpose();
    }

    void Go2JointController::computePD()
    {
        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];
            commanded_effort[index] = kp[index] * q_e[index] + kd[index] * dq_e[index];
        }
    }

    uint32_t Go2JointController::crc32_core(uint32_t *ptr, uint32_t len)
    {
        unsigned int xbit = 0;
        unsigned int data = 0;
        unsigned int CRC32 = 0xFFFFFFFF;
        const unsigned int dwPolynomial = 0x04c11db7;

        for (unsigned int i = 0; i < len; i++)
        {
            xbit = 1 << 31;
            data = ptr[i];
            for (unsigned int bits = 0; bits < 32; bits++)
            {
                if (CRC32 & 0x80000000)
                {
                    CRC32 <<= 1;
                    CRC32 ^= dwPolynomial;
                }
                else
                {
                    CRC32 <<= 1;
                }

                if (data & xbit)
                    CRC32 ^= dwPolynomial;
                xbit >>= 1;
            }
        }

        return CRC32;
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_jointcontroller::Go2JointController,
    controller_interface::ControllerInterface)