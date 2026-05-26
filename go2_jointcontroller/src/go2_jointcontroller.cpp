#include "go2_jointcontroller/go2_jointcontroller.hpp"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

using namespace unitree::robot;

namespace go2_jointcontroller
{

    Go2JointController::Go2JointController()
        : controller_interface::ControllerInterface()
        , model()
        , _q(12)
        , _qd(12)
        , _tau(12)
        , _effort(12)
        , kp(12)
        , kd(12)
        , ki(12)
        , q_e(12)
        , qi_e(12)
        , dq_e(12)
        , qr(12)
        , dqr(12)
        , update_rate(0)
        , _percent(0)
        , _duration(1000)
        , _started(false)
        , _startPos(12)
        , _targetPos(12)
        , _lowTick(0)
        , control_mode(1)
    {
        const auto package_share_path = ament_index_cpp::get_package_share_directory("go2_description");
        const auto xacro_path = std::filesystem::path(package_share_path) / "urdf" / "go2.xacro";
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
        pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model);

        model.gravity.linear(Eigen::Vector3d(0, 0, -9.8));
        data = std::make_shared<pinocchio::Data>(model);
    }

    controller_interface::CallbackReturn Go2JointController::on_init()
    {
        try
        {
            auto_declare<std::vector<double>>("joints.initpos", _targetPos);
            auto_declare<std::string>("network_interface", "eth0");
            auto_declare<bool>("simulation", false);

            std::vector<double> zeros(12, 0.0);
            auto_declare<std::vector<double>>("gain.PD.Kp", zeros);
            auto_declare<std::vector<double>>("gain.PD.Kd", zeros);
            auto_declare<std::vector<double>>("gain.PDG.Kp", zeros);
            auto_declare<std::vector<double>>("gain.PDG.Kd", zeros);
            auto_declare<std::vector<double>>("gain.PID.Kp", zeros);
            auto_declare<std::vector<double>>("gain.PID.Kd", zeros);
            auto_declare<std::vector<double>>("gain.PID.Ki", zeros);
            auto_declare<std::vector<double>>("gain.PIDG.Kp", zeros);
            auto_declare<std::vector<double>>("gain.PIDG.Kd", zeros);
            auto_declare<std::vector<double>>("gain.PIDG.Ki", zeros);
            auto_declare<int>("control_mode", control_mode);
            
            auto_declare<int>("update_rate", update_rate);
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

        _targetPos = get_node()->get_parameter("joints.initpos").get_value<std::vector<double>>();
        if (_targetPos.size() != 12)
        {
            RCLCPP_ERROR(logger, "Initial joints postures 'initpos' vector does note have size 12, verify yaml file.");
            return CallbackReturn::FAILURE;
        }

        control_mode = get_node()->get_parameter("control_mode").get_value<int>();
        RCLCPP_INFO(logger, "==================================================");
        RCLCPP_INFO(logger, "   STARTING SIMULATION WITH CONTROL MODE: %d   ", control_mode);
        RCLCPP_INFO(logger, "==================================================");

        selectControlMode(control_mode);

        update_rate = get_node()->get_parameter("update_rate").get_value<int>();

        // TODO: use the name of topic from the YAML file
        lowstate_subscriber_ = get_node()->create_subscription<lowStates>(
            "/lowstate", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<lowStates> msg) -> void
            {
                std::lock_guard<std::mutex> lock(this->mutex_controller);

                for (auto index{0}; index < 12; index++)
                {
                    _q[index] = msg->motor_state[index].q;
                    _qd[index] = msg->motor_state[index].dq;
                }
                _lowTick = msg->tick;
            });

        controller_reference_subscriber_ = get_node()->create_subscription<lowCmd>(
            "go2_jointcontroller/JointControllerReferences", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<lowCmd> msg) -> void
            {
                std::lock_guard<std::mutex> lock(this->mutex_controller);

                for (int index = 0; index < 12; index++)
                {
                    qr[index] = msg->motor_cmd[index].q;
                    dqr[index] = msg->motor_cmd[index].dq;
                    // kd[index] = msg->motor_cmd[index].kd;
                    // kp[index] = msg->motor_cmd[index].kp;
                    // tau[index] = msg->motor_cmd[index].tau;
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
            lowCmd_msg.motor_cmd[i].dq = VelStopF;
            lowCmd_msg.motor_cmd[i].kp = 0;
            lowCmd_msg.motor_cmd[i].kd = 0;
            lowCmd_msg.motor_cmd[i].tau = 0;
        }

        if(!get_node()->get_parameter("simulation").get_value<bool>())
            ms_client_ = std::make_shared<unitree::robot::go2::MotionSwitchClient>(this->get_node().get());


        sleep(1);
        
        while (queryMotionStatus() != 0) {
            std::cout << "Try to deactivate the motion control-related service." << std::endl;
            int32_t ret = ms_client_->ReleaseMode();
            if (ret == 0) {
                std::cout << "ReleaseMode succeeded." << std::endl;
            } else {
                std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
            }
            sleep(2);
        }

        RCLCPP_INFO(this->get_node()->get_logger(), "Go2 Node Initialized");

        return CallbackReturn::SUCCESS;
    }

    void Go2JointController::selectControlMode(int mode)
    {
        if(mode == 1)
        {
            kp = get_node()->get_parameter("gain.PD.Kp").get_value<std::vector<double>>();
            kd = get_node()->get_parameter("gain.PD.Kd").get_value<std::vector<double>>();
            ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        else if(mode == 2)
        {
            kp = get_node()->get_parameter("gain.PDG.Kp").get_value<std::vector<double>>();
            kd = get_node()->get_parameter("gain.PDG.Kd").get_value<std::vector<double>>();
            ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        else if(mode == 3)
        {
            kp = get_node()->get_parameter("gain.PID.Kp").get_value<std::vector<double>>();
            kd = get_node()->get_parameter("gain.PID.Kd").get_value<std::vector<double>>();
            ki = get_node()->get_parameter("gain.PID.Ki").get_value<std::vector<double>>();
        }
        else if(mode == 4)
        {
            kp = get_node()->get_parameter("gain.PIDG.Kp").get_value<std::vector<double>>();
            kd = get_node()->get_parameter("gain.PIDG.Kd").get_value<std::vector<double>>();
            ki = get_node()->get_parameter("gain.PIDG.Ki").get_value<std::vector<double>>();
        }
        else
        {
            kd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            kp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
    }

    controller_interface::CallbackReturn Go2JointController::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Activating Go2JointController...");

        // Wait for a valid reading from robot low states
        while(_lowTick == 0);

        for(int i=0; i<12; i++)
        {
            _startPos[i] = _q[i];
            qr[i] = _targetPos[i];
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Go2JointController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Deactivating Go2JointController...");

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
                        lowCmd_msg.motor_cmd[j].dq = 0.0;
                        lowCmd_msg.motor_cmd[j].kp = 50.0;
                        lowCmd_msg.motor_cmd[j].kd = 0.7;
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
                _effort = computePID();
                switch (control_mode)
                {
                    case 1: // PD only
                    case 3: // PID
                        _tau = Eigen::VectorXd::Zero(12);
                        break;

                    case 2: // PD + Gravity Compensation
                    case 4: // PID + Gravity Compensation
                        _tau = computeTotalGravityCompensation(_q);
                        break;

                    default:
                        RCLCPP_ERROR(logger, "Invalid control mode: %d", control_mode);
                        return controller_interface::return_type::ERROR;
                }
                for (int j = 0; j < 12; j++)
                {
                    lowCmd_msg.motor_cmd[j].q = PosStopF;
                    lowCmd_msg.motor_cmd[j].dq = VelStopF;
                    lowCmd_msg.motor_cmd[j].kp = 0;
                    lowCmd_msg.motor_cmd[j].kd = 0;
                    auto cmd = std::max(-5.0, std::min(5.0, _effort[j] + _tau[j]));
                    if (std::isfinite(cmd))
                        lowCmd_msg.motor_cmd[j].tau = cmd;
                    else
                        lowCmd_msg.motor_cmd[j].tau = 0;
                }
            }
            
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

    Eigen::VectorXd Go2JointController::computeTotalGravityCompensation(Eigen::VectorXd q)
    {
        Eigen::VectorXd q_act = Eigen::VectorXd::Zero(19);

        q_act.segment(0, 6) = Eigen::VectorXd::Zero(6);
        q_act.segment(7, 12) = q;

        //Atualiza cinemática direta para garantir que frames estejam atualizados
        pinocchio::forwardKinematics(model, *data, q_act);

        //above function is the same of pinocchio::rnea(model, *data, qcerto, Eigen::VectorXd::Zero(model.nv), Eigen::VectorXd::Zero(model.nv));
        pinocchio::computeGeneralizedGravity(model, *data, q_act);

        return data->g.segment(6, 12);
    }

    Eigen::VectorXd Go2JointController::computePID()
    {
        Eigen::VectorXd effort(12);

        for (auto index{0}; index < 12; index++)
        {
            q_e[index] = qr[index] - _q[index];
            dq_e[index] = dqr[index] - _qd[index];
            qi_e[index] = std::max(-5.0, std::min(5.0, qi_e[index] + (q_e[index] / update_rate))); // I parcel with antiwindup

            effort[index] = kp[index] * q_e[index] + kd[index] * dq_e[index] + ki[index] * qi_e[index];
        }

        return effort;
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

    int Go2JointController::queryMotionStatus()
    {
        if(get_node()->get_parameter("simulation").get_value<bool>())
            return 0;

        std::string robotForm;
        std::string motionName;
        int motionStatus = 0;

        int32_t ret = ms_client_->CheckMode(robotForm, motionName);
        if (ret == 0) {
            std::cout << "CheckMode succeeded." << std::endl;
        } else {
            std::cout << "CheckMode failed. Error code: " << ret << std::endl;
        }
        
        if (motionName.empty()) {
            std::cout << "The motion control-related service is deactivated." << std::endl;
            motionStatus = 0;
        } else {
            std::string serviceName = queryServiceName(robotForm, motionName);
            std::cout << "Service: " << serviceName << " is activate" << std::endl;
            motionStatus = 1;
        }

        return motionStatus;
    }

    std::string Go2JointController::queryServiceName(std::string form,std::string name)
    {
        if (form == "0") {
            if (name == "normal") {
                return "sport_mode";
            }
            if (name == "ai") {
                return "ai_sport";
            }
            if (name == "advanced") {
                return "advanced_sport";
            }
            } else {
            if (name == "ai-w") {
                return "wheeled_sport(go2W)";
            }
            if (name == "normal-w") {
                return "wheeled_sport(b2W)";
            }
        }
        return "";
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    go2_jointcontroller::Go2JointController,
    controller_interface::ControllerInterface)
