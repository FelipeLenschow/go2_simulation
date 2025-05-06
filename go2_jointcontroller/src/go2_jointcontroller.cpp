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

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace go2_jointcontroller
{

    Go2JointController::Go2JointController()
        : controller_interface::ControllerInterface(),
          joint_names_({}),
          model(),
          gravidade(3),
          q(12),
          dq(12),
          kp(12),
          kd(12),
          ki(12),
          tau(12),
          tauG(12),
          tau_(12),
          q_e(12),
          qi_e(12),
          dq_e(12),
          qr(12),
          dqr(12),
          mass(12),
          effort(12),
          commanded_effort(12),
          vetor_pinocchio(12),
          v(Eigen::VectorXd::Zero(12)),
          a(Eigen::VectorXd::Zero(12)),
          pinocchio_frames(12)
    {
        const auto package_share_path = ament_index_cpp::get_package_share_directory("go2_description");
        const auto xacro_path = std::filesystem::path(package_share_path) / "urdf" / "go2.xacro.urdf";
        const auto urdf_path = std::filesystem::temp_directory_path() / "go2.urdf";

        // Convert Xacro to URDF using ROS 2 xacro CLI
        std::string command = "ros2 run xacro xacro " + xacro_path.string() + " -o " + urdf_path.string();
        int result = std::system(command.c_str());

        if (result != 0)
        {
            std::cerr << "Error: Failed to convert Xacro to URDF!" << std::endl;
            return;
        }

        std::cout << "Converted Xacro to URDF: " << urdf_path << std::endl;

        // Create a set of Pinocchio models and data.
        pinocchio::urdf::buildModel(urdf_path, model);

        for (int i = 1; i <= model.nq; i++)
            std::cout << model.names[i] << std::endl;

        data = std::make_shared<pinocchio::Data>(model);

        for (int i = 0; i < model.nq - 1; i++)
            pinocchio_frames[i] = model.getFrameId(joint_names_sequence[i]);

        control_mode = 1;

        gravidade[0] = 0;
        gravidade[1] = 0;
        gravidade[2] = -9.80665;

        mass[9] = mass[6] = mass[3] = mass[0] = 0.678;
        mass[10] = mass[7] = mass[4] = mass[1] = 1.152;
        mass[11] = mass[8] = mass[5] = mass[2] = 0.154;

        ki[0] = ki[3] = ki[6] = ki[9] = 50;
        ki[1] = ki[4] = ki[7] = ki[10] = 25;
        ki[2] = ki[5] = ki[8] = ki[11] = 0;
    }

    controller_interface::CallbackReturn Go2JointController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", joint_names_);
            // auto_declare<double>("gain.Kp", 60.0);
            // auto_declare<double>("gain.Kd", 5.0);
            auto_declare<double>("up_rate", 200.0);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

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
        // auto kp_gain_ = get_node()->get_parameter("gain.Kp").get_value<double>();
        // auto kd_gain_ = get_node()->get_parameter("gain.Kd").get_value<double>();
        auto _update_rate = get_node()->get_parameter("up_rate").get_value<double>();
        sample_time = 1.0 / _update_rate;

        for (size_t i = 0; i < 12; i++)
        {
            // kp[i] = kp_gain_;
            // kd[i] = kd_gain_;
            tau[i] = 0.0;
        }
        // TODO: use the name of topic from the YAML file
        lowstate_subscriber_ = get_node()->create_subscription<lowStates>(
            "/lowstate", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<lowStates> msg) -> void
            {
                std::lock_guard<std::mutex> lock(this->mutex_controller);

                for (auto index{0}; index < 12; index++)
                {
                    q[index] = msg->motor_state[index].q;

                    dq[index] = msg->motor_state[index].dq;
                }
            });

        controller_reference_subscriber_ = get_node()->create_subscription<lowCmd>(
            "go2_jointcontroller/JointControllerReferences", rclcpp::SystemDefaultsQoS(),
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

        joints_cmd_publisher_ = get_node()->create_publisher<lowCmd>("/lowcmd", 10);

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
        const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {

        if (last_update_time_ == 0)
        {
            last_update_time_ = time.nanoseconds();
        }

        // Compute time difference since last update
        elapsed_time = (time.nanoseconds() - last_update_time_) * 1e-9; // Convert ns to seconds

        const auto logger = get_node()->get_logger();
        if (elapsed_time >= sample_time)
        {
            std::lock_guard<std::mutex> lock(this->mutex_controller);
            {
                try
                {
                    switch (control_mode)
                    {
                    case 1: // PD only
                        computePD();
                        for (int j = 0; j < 12; j++)
                        {
                            lowCmd_msg.motor_cmd[j].q = 0;
                            lowCmd_msg.motor_cmd[j].dq = 0;
                            lowCmd_msg.motor_cmd[j].kp = 0;
                            lowCmd_msg.motor_cmd[j].kd = 0;
                            lowCmd_msg.motor_cmd[j].tau = commanded_effort[j];
                        }
                        break;

                    case 2: // PD + Gravity Compensation
                        computePD_COMPG();
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

                    case 3: // PID
                        computePID();
                        for (int j = 0; j < 12; j++)
                        {
                            lowCmd_msg.motor_cmd[j].q = 0;
                            lowCmd_msg.motor_cmd[j].dq = 0;
                            lowCmd_msg.motor_cmd[j].kp = 0;
                            lowCmd_msg.motor_cmd[j].kd = 0;
                            lowCmd_msg.motor_cmd[j].tau = commanded_effort[j];
                        }
                        break;

                    case 4: // PID + Gravity Compensation

                        computePID_COMPG();
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
                        RCLCPP_ERROR(logger, "Invalid control mode: %d", control_mode);
                        return controller_interface::return_type::ERROR;
                    }

                    // Publish the control message
                    lowCmd_msg.crc = crc32_core((uint32_t *)&lowCmd_msg, (sizeof(lowCmd) >> 2) - 1);
                    joints_cmd_publisher_->publish(lowCmd_msg);

                    return controller_interface::return_type::OK;
                }
                catch (const std::exception &e)
                {

                    RCLCPP_ERROR(logger, "Exception in update(): %s", e.what());
                    return controller_interface::return_type::ERROR;
                }
            }

            last_update_time_ = time.nanoseconds(); // Reset timer
        }
        return controller_interface::return_type::OK;
    }

    // void Go2JointController::computeG()
    // {
    //     // Atualiza a cinemática direta
    //     pinocchio::forwardKinematics(model, *data, q);

    //     // Zera o vetor de torques para garantir acúmulo correto
    //     tauG = Eigen::VectorXd::Zero(model.nv);

    //     // Para cada perna (4 no total)
    //     for (int leg = 0; leg < 4; ++leg)
    //     {
    //         // Para cada elo da perna (ombro, coxa, panturrilha)
    //         for (int elo = 0; elo < 3; ++elo)
    //         {
    //             int frame_id = leg * 3 + elo; // índice do frame do elo

    //             // Jacobiano do frame no mundo
    //             Eigen::MatrixXd J(6, model.nv);
    //             pinocchio::computeFrameJacobian(model, *data, q, pinocchio_frames[frame_id], pinocchio::WORLD, J);

    //             // Parte linear do Jacobiano
    //             Eigen::MatrixXd J_linear = J.topRows(3);

    //             // Força gravitacional aplicada ao centro de massa do elo
    //             Eigen::Vector3d F_grav = mass[frame_id] * gravidade;

    //             // Torque resultante nas juntas causado por essa força
    //             Eigen::VectorXd tau_contrib = J_linear.transpose() * F_grav;

    //             // Acumula o torque em cada junta
    //             for (int j = 0; j < model.nv; ++j)
    //             {
    //                 tauG[j] += tau_contrib[j];
    //                 // std::cout << tauG << std::endl;
    //                 // std::cout << "--- - - - - -- - - ----  - - - - --  - - - - -- - - - - ------------------------------ " << std::endl;
    //             }
    //         }
    //     }
    // }

    void Go2JointController::computeG()
    {
        pinocchio::forwardKinematics(model, *data, q); // atualiza valores
        tauG = Eigen::VectorXd::Zero(12);              // zera torque no início de cada cálculo

        for (int leg = 0; leg < 4; leg++) // organizando as juntas por perna
        {
            int j0 = leg * 3 + 0; // ombro
            int j1 = leg * 3 + 1; // joelho
            int j2 = leg * 3 + 2; // tornozelo

            for (int e = 0; e < 3; e++)
            {
                int i = leg * 3 + e; // índice do elo (frame) da perna

                // Calcula o jacobiano linear do centro de massa do elo i
                Eigen::MatrixXd J(6, model.nv);
                pinocchio::computeFrameJacobian(model, *data, q, pinocchio_frames[i], pinocchio::LOCAL, J); // cálculo do jacobiano
                Eigen::MatrixXd J_linear = J.topRows(3);                                                    // Parte linear do Jacobiano

                Eigen::Vector3d F_grav = mass[i] * gravidade;                   // Força gravitacional no elo
                Eigen::VectorXd torque_contrib = J_linear.transpose() * F_grav; // torque do compensador gravitacional daquela junta

                // O torque do elo afeta a si mesmo e todas as juntas anteriores (superiores)
                tauG[j2] += torque_contrib[j2]; // contribuição dos elos distais
                tauG[j1] += torque_contrib[j1];
                tauG[j0] += torque_contrib[j0];

                // std::cout << torque_contrib << std::endl;
                // std::cout << "--- - - - - -- - - ----  - - - - --  - - - - -- - - - - ------------------------------ " << std::endl;
            }
            // std::cout << tauG << std::endl;
            // std::cout << "--- - - - - -- - - ----  - - - - --  - - - - -- - - - - ------------------------------ " << std::endl;
        }
    }

    void Go2JointController::computePD()
    {
        auto kp = get_node()->get_parameter("gain.PD.Kp").get_value<std::vector<double>>();
        auto kd = get_node()->get_parameter("gain.PD.Kd").get_value<std::vector<double>>();

        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];
            commanded_effort[index] = kp[index] * q_e[index] + kd[index] * dq_e[index];
        }
    }

    void Go2JointController::computePD_COMPG()
    {
        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];
            commanded_effort[index] = 33 * q_e[index] + 0.5 * dq_e[index];
        }
        commanded_effort[0] = commanded_effort[0] + 0.3 * dq_e[0];
        commanded_effort[1] = commanded_effort[1] + 0.3 * dq_e[1];
        commanded_effort[3] = commanded_effort[3] + 0.3 * dq_e[3];
        commanded_effort[4] = commanded_effort[4] + 0.3 * dq_e[4];
        commanded_effort[6] = commanded_effort[6] + 0.3 * dq_e[6];
        commanded_effort[7] = commanded_effort[7] + 0.3 * dq_e[7];
        commanded_effort[9] = commanded_effort[9] + 0.3 * dq_e[9];
        commanded_effort[10] = commanded_effort[10] + 0.3 * dq_e[10];
    }

    void Go2JointController::computePID()
    {
        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];
            qi_e[index] += (q_e[index] / 1000);

            commanded_effort[index] = 50 * q_e[index] + 0.5 * dq_e[index] + ki[index] * qi_e[index];
        }
    }

    void Go2JointController::computePID_COMPG()
    {
        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];
            qi_e[index] += (q_e[index] / 1000);

            commanded_effort[index] = 25 * q_e[index] + 0.6 * dq_e[index];
        }
        commanded_effort[0] = commanded_effort[0] + 0.5 * dq_e[0] + 25 * qi_e[0];
        commanded_effort[1] = commanded_effort[1] + 0.5 * dq_e[0] + 12.5 * qi_e[1];
        commanded_effort[3] = commanded_effort[3] + 0.5 * dq_e[3] + 25 * qi_e[3];
        commanded_effort[4] = commanded_effort[4] + 0.5 * dq_e[4] + 12.5 * qi_e[4];
        commanded_effort[6] = commanded_effort[6] + 0.5 * dq_e[6] + 25 * qi_e[6];
        commanded_effort[7] = commanded_effort[7] + 0.5 * dq_e[7] + 12.5 * qi_e[7];
        commanded_effort[9] = commanded_effort[9] + 0.5 * dq_e[9] + 25 * qi_e[9];
        commanded_effort[10] = commanded_effort[10] + 0.5 * dq_e[10] + 12.5 * qi_e[10];
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