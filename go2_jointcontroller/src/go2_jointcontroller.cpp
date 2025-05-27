#include "go2_jointcontroller/go2_jointcontroller.hpp"
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

namespace go2_jointcontroller
{

    Go2JointController::Go2JointController()
        : controller_interface::ControllerInterface()
        , joint_names_({})
        , model()
        , gravidade(3)
        , q(12)
        , dq(12)
        , kp(12)
        , kd(12)
        , ki(12)
        , tau(12)
        , tauG(12)
        , tau_(12)
        , q_e(12)
        , qi_e(12)
        , dq_e(12)
        , qr(12)
        , dqr(12)
        , mass(12)
        , effort(12)
        , commanded_effort(12)
        , vetor_pinocchio(12)
        , v(Eigen::VectorXd::Zero(12))
        , a(Eigen::VectorXd::Zero(12))
        , _percent(0)
        , _duration(5000)
        , _started(false)
        , _startPos(12)
        , _targetPos(12)
        , _lowTick(0)
        , control_mode(1)
        , pinocchio_frames(12)
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

        data = std::make_shared<pinocchio::Data>(model);

        for (int i = 0; i < model.nq - 1; i++)
            pinocchio_frames[i] = model.getFrameId(joint_names_sequence[i]);

        for (const auto &joint_name : desired_order)
        {
            auto it = std::find(pinocchio_order.begin(), pinocchio_order.end(), joint_name);
            if (it != pinocchio_order.end())
            {
                int idx = std::distance(pinocchio_order.begin(), it);
                map_desired_to_pinocchio.push_back(idx);
            }
            else
            {
                std::cerr << "Joint name not found: " << joint_name << std::endl;
                map_desired_to_pinocchio.push_back(-1); // erro
            }
        }
        
        gravidade[0] = 0;
        gravidade[1] = 0;
        gravidade[2] = -9.80665;

        mass[9] = mass[6] = mass[3] = mass[0] = 0.678;
        mass[10] = mass[7] = mass[4] = mass[1] = 1.152;
        mass[11] = mass[8] = mass[5] = mass[2] = 0.154;

        ki[0] = ki[3] = ki[6] = ki[9] = 45;
        ki[1] = ki[4] = ki[7] = ki[10] = 25;
        ki[2] = ki[5] = ki[8] = ki[11] = 0;
    }

    controller_interface::CallbackReturn Go2JointController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints.names", joint_names_);
            auto_declare<std::vector<double>>("joints.initpos", _targetPos);
            auto_declare<std::vector<double>>("gain.PD.Kp", kp);
            auto_declare<std::vector<double>>("gain.PD.Kd", kd);
            auto_declare<int>("control_mode", control_mode);
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
        joint_names_ = get_node()->get_parameter("joints.names").as_string_array();
        if (joint_names_.empty())
        {
            RCLCPP_ERROR(logger, "No joint names found in parameters.");
            return CallbackReturn::FAILURE;
        }

        _targetPos = get_node()->get_parameter("joints.initpos").get_value<std::vector<double>>();
        if (_targetPos.size() != 12)
        {
            RCLCPP_ERROR(logger, "Initial joints postures 'initpos' vector does note have size 12, verify yaml file.");
            return CallbackReturn::FAILURE;
        }

        kp = get_node()->get_parameter("gain.PD.Kp").get_value<std::vector<double>>();
        if (kp.size() != 12)
        {
            RCLCPP_ERROR(logger, "Kp gains are not a vector of size 12, verify yaml file.");
            return CallbackReturn::FAILURE;
        }
        
        kd = get_node()->get_parameter("gain.PD.Kd").get_value<std::vector<double>>();
        if (kd.size() != 12)
        {
            RCLCPP_ERROR(logger, "Kd gains are not a vector of size 12, verify yaml file.");
            return CallbackReturn::FAILURE;
        }

        control_mode = get_node()->get_parameter("control_mode").get_value<int>();

        for (size_t i = 0; i < 12; i++)
        {
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
                _lowTick = msg->tick;
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

        for(int i=0; i<20; i++)
        {
            lowCmd_msg.motor_cmd[i].mode = 0x01;   // motor switch to servo (PMSM) mode
            lowCmd_msg.motor_cmd[i].q = PosStopF;
            lowCmd_msg.motor_cmd[i].kp = 0;
            lowCmd_msg.motor_cmd[i].dq = VelStopF;
            lowCmd_msg.motor_cmd[i].kd = 0;
            lowCmd_msg.motor_cmd[i].tau = 0;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2JointController::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating Go2JointController...");

        // Reset torques and errors
        for (auto &t : tau)
            t = 0.0;
        
        // Wait for a valid reading from robot low states
        while(_lowTick == 0);

        for(int i=0; i<12; i++)
        {
            _startPos[i] = q[i];
        }

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
        const auto logger = get_node()->get_logger();
        try
        {
            std::lock_guard<std::mutex> lock(this->mutex_controller);
            if(!_started)
            {
                _percent += 1.0 / _duration;
                _percent = _percent > 1 ? 1 : _percent;
                if (_percent < 1)
                {
                    for (int j = 0; j < 12; j++)
                    {
                        lowCmd_msg.motor_cmd[j].q = (1 - _percent) * _startPos[j] + _percent * _targetPos[j];
                        lowCmd_msg.motor_cmd[j].dq = 0;
                        lowCmd_msg.motor_cmd[j].kp = kp[j];
                        lowCmd_msg.motor_cmd[j].kd = kd[j];
                        lowCmd_msg.motor_cmd[j].tau = 0;
                    }
                }
                else
                {
                    _started = true;
                }
            }
            else
            {
                switch (control_mode)
                {
                    case 1: // PD only
                        computePD();
                        for (int j = 0; j < 12; j++)
                        {
                            lowCmd_msg.motor_cmd[j].q = qr[j];
                            lowCmd_msg.motor_cmd[j].dq = dqr[j];
                            lowCmd_msg.motor_cmd[j].kp = kp[j];
                            lowCmd_msg.motor_cmd[j].kd = kd[j];
                            lowCmd_msg.motor_cmd[j].tau = 0; // commanded_effort[j];
                        }
                        break;

                    case 2: // PD + Gravity Compensation
                        computePD_COMPG();
                        computeTotalGravityCompensation();
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
                        computeTotalGravityCompensation();
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
            }
            
            // std::cout << "## " << _percent << std::endl;
            // for (int i = 0; i < 12; i++)      
            // {                  
            //     std::cout << "## " << i << " #msg#" << lowCmd_msg.motor_cmd[i].q 
            //     << " #ini#" << _startPos[i] 
            //     << " #tag#" << _targetPos[i] << std::endl;
            // }

            // Publish the control message
            get_crc(lowCmd_msg);
            joints_cmd_publisher_->publish(lowCmd_msg);

            return controller_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {

            RCLCPP_ERROR(logger, "Exception in update(): %s", e.what());
            return controller_interface::return_type::ERROR;
        }
    }

    // Função de reordenação de entrada
    Eigen::VectorXd Go2JointController::reorder_to_pinocchio(const Eigen::VectorXd &vec_desired)
    {
        Eigen::VectorXd vec_pinocchio(vec_desired.size());
        for (int i = 0; i < vec_desired.size(); ++i)
        {
            vec_pinocchio[map_desired_to_pinocchio[i]] = vec_desired[i];
        }
        return vec_pinocchio;
    }

    // Função de reordenação de saída
    Eigen::VectorXd Go2JointController::reorder_to_desired(const Eigen::VectorXd &vec_pinocchio)
    {
        Eigen::VectorXd vec_desired(vec_pinocchio.size());
        for (int i = 0; i < vec_pinocchio.size(); ++i)
        {
            vec_desired[i] = vec_pinocchio[map_desired_to_pinocchio[i]];
        }
        return vec_desired;
    }
    void Go2JointController::computeG12()

    {

        // Supondo que qr, v, a estão na ordem desejada (FL, FR, RL, RR)

        Eigen::VectorXd q_pinocchio = reorder_to_pinocchio(qr);

        Eigen::VectorXd v_pinocchio = reorder_to_pinocchio(v);

        Eigen::VectorXd a_pinocchio = reorder_to_pinocchio(a);

        // Calcula torques na ordem do Pinocchio

        Eigen::VectorXd tau_pinocchio = pinocchio::rnea(model, *data, q_pinocchio, v_pinocchio, a_pinocchio);

        // Reordena para sua ordem desejada

        Eigen::VectorXd tau_temp = reorder_to_desired(tau_pinocchio);

        // Preenche apenas os índices 1, 2, 4, 5, 7, 8, 10 e 11

        std::vector<int> indices_to_fill = {1, 2, 4, 5, 7, 8, 10, 11};

        for (int i : indices_to_fill)
        {

            if (i >= 0 && i < tauG.size() && i < tau_temp.size())
            {

                tauG[i] = tau_temp[i];
            }
        }
    }

    void Go2JointController::computeG0()
    {

        // Atualiza cinemática direta para garantir que frames estejam atualizados

        pinocchio::forwardKinematics(model, *data, q);

        for (int leg = 0; leg < 4; leg++)
        {

            int shoulder_index = leg * 3 + 0; // Índice da junta de "ombro" (0, 3, 6, 9)

            Eigen::VectorXd torque_total = Eigen::VectorXd::Zero(model.nv);

            for (int e = 0; e < 3; e++)
            {

                int i = leg * 3 + e; // Índice global do elo

                // Calcula jacobiano do frame do elo no mundo

                Eigen::MatrixXd J(6, model.nv);

                pinocchio::computeFrameJacobian(model, *data, qr, pinocchio_frames[i], pinocchio::WORLD, J);

                // Parte linear do jacobiano

                Eigen::MatrixXd J_linear = J.topRows(3);

                // Força gravitacional no centro de massa

                Eigen::Vector3d F_grav = mass[i] * gravidade;

                // Torque gerado pela gravidade

                Eigen::VectorXd torque_contrib_global = J_linear.transpose() * F_grav;

                // Acumula toda a contribuição no vetor total

                torque_total += torque_contrib_global;
            }

            // Após somar as contribuições de todos os elos da perna, aplica apenas na junta do ombro

            tauG[shoulder_index] += torque_total[shoulder_index];
        }
    }

    void Go2JointController::computeTotalGravityCompensation()
    {
        // Zera todo o vetor de torques antes de preencher
        tauG = Eigen::VectorXd::Zero(12);

        // Preenche os torques apenas nos índices 0, 3, 6 e 9
        computeG0();

        // Preenche os torques apenas nos índices 1, 2, 4, 5, 7, 8, 10 e 11
        computeG12();
    }

    void Go2JointController::computePD()
    {
        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];
            commanded_effort[index] = kp[index] * q_e[index] + kd[index] * dq_e[index];
        }
        commanded_effort[0] = commanded_effort[0] + 25 * q_e[0];
        commanded_effort[1] = commanded_effort[1] + 25 * q_e[1];
        commanded_effort[3] = commanded_effort[3] + 25 * q_e[3];
        commanded_effort[4] = commanded_effort[4] + 25 * dq_e[4];
        commanded_effort[6] = commanded_effort[6] + 25 * dq_e[6];
        commanded_effort[7] = commanded_effort[7] + 25 * dq_e[7];
        commanded_effort[9] = commanded_effort[9] + 25 * dq_e[9];
        commanded_effort[10] = commanded_effort[10] + 25 * dq_e[10];
    }

    void Go2JointController::computePD_COMPG()
    {
        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];
            commanded_effort[index] = 25 * q_e[index] + 0.4 * dq_e[index];
        }
        commanded_effort[0] = commanded_effort[0] + 35 * q_e[0] + 0.3 * dq_e[0];
        commanded_effort[1] = commanded_effort[1] + 30 * q_e[1] + 0.3 * dq_e[1];
        commanded_effort[3] = commanded_effort[3] + 25 * q_e[3] + 0.3 * dq_e[3];
        commanded_effort[4] = commanded_effort[4] + 30 * q_e[4] + 0.3 * dq_e[4];
        commanded_effort[6] = commanded_effort[6] + 25 * q_e[6] + 0.3 * dq_e[6];
        commanded_effort[7] = commanded_effort[7] + 30 * q_e[7] + 0.3 * dq_e[7];
        commanded_effort[9] = commanded_effort[9] + 25 * q_e[9] + 0.3 * dq_e[9];
        commanded_effort[10] = commanded_effort[10] + 30 * q_e[10] + 0.3 * dq_e[10];
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
        commanded_effort[0] = commanded_effort[0] + 15 * q_e[0];
        commanded_effort[3] = commanded_effort[3] + 15 * q_e[3];
        commanded_effort[6] = commanded_effort[6] + 15 * q_e[6];
        commanded_effort[9] = commanded_effort[9] + 15 * q_e[9];
    }

    void Go2JointController::computePID_COMPG()
    {
        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - q[index];
            dq_e[index] = dqr[index] - dq[index];
            qi_e[index] += (q_e[index] / 1000);

            commanded_effort[index] = 30 * q_e[index] + 0.6 * dq_e[index];
        }
        commanded_effort[0] = commanded_effort[0] + 35 * q_e[0] + 30 * qi_e[0];
        commanded_effort[1] = commanded_effort[1] + 30 * q_e[1] + 20 * qi_e[1];
        commanded_effort[3] = commanded_effort[3] + 35 * q_e[3] + 30 * qi_e[3];
        commanded_effort[4] = commanded_effort[4] + 35 * q_e[4] + 20 * qi_e[4];
        commanded_effort[6] = commanded_effort[6] + 30 * q_e[6] + 20 * qi_e[6];
        commanded_effort[7] = commanded_effort[7] + 35 * q_e[7] + 20 * qi_e[7];
        commanded_effort[9] = commanded_effort[9] + 30 * q_e[9] + 30 * qi_e[9];
        commanded_effort[10] = commanded_effort[10] + 35 * q_e[10] + 20 * qi_e[10];
    }

    uint32_t Go2JointController::crc32_core(uint32_t *ptr, uint32_t len)
    {
        uint32_t xbit = 0;
        uint32_t data = 0;
        uint32_t CRC32 = 0xFFFFFFFF;
        const uint32_t dwPolynomial = 0x04c11db7;
        for (uint32_t i = 0; i < len; i++)
        {
            xbit = 1 << 31;
            data = ptr[i];
            for (uint32_t bits = 0; bits < 32; bits++)
            {
                if (CRC32 & 0x80000000)
                {
                    CRC32 <<= 1;
                    CRC32 ^= dwPolynomial;
                }
                else
                    CRC32 <<= 1;
                if (data & xbit)
                    CRC32 ^= dwPolynomial;
    
                xbit >>= 1;
            }
        }
        return CRC32;
    }


    void Go2JointController::get_crc(lowCmd& msg)
    {
        LowCmd raw{};
        memcpy(&raw.head[0], &msg.head[0], 2);

        raw.levelFlag=msg.level_flag;
        raw.frameReserve=msg.frame_reserve;

        memcpy(&raw.SN[0],&msg.sn[0], 8);
        memcpy(&raw.version[0], &msg.version[0], 8);

        raw.bandWidth=msg.bandwidth;


        for(int i = 0; i<20; i++)
        {
            raw.motorCmd[i].mode=msg.motor_cmd[i].mode;
            raw.motorCmd[i].q=msg.motor_cmd[i].q;
            raw.motorCmd[i].dq=msg.motor_cmd[i].dq;
            raw.motorCmd[i].tau=msg.motor_cmd[i].tau;
            raw.motorCmd[i].Kp=msg.motor_cmd[i].kp;
            raw.motorCmd[i].Kd=msg.motor_cmd[i].kd;

            memcpy(&raw.motorCmd[i].reserve[0], &msg.motor_cmd[i].reserve[0], 12);
        }

        raw.bms.off=msg.bms_cmd.off;
        memcpy(&raw.bms.reserve[0],&msg.bms_cmd.reserve[0],  3);


        memcpy(&raw.wirelessRemote[0], &msg.wireless_remote[0], 40);

        memcpy(&raw.led[0], &msg.led[0],  12);  // go2
        memcpy(&raw.fan[0], &msg.fan[0],  2);
        raw.gpio=msg.gpio;    // go2

        raw.reserve=msg.reserve;

        raw.crc=crc32_core((uint32_t *)&raw, (sizeof(LowCmd)>>2)-1);
        msg.crc=raw.crc;

        
    }
}



#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_jointcontroller::Go2JointController,
    controller_interface::ControllerInterface)
