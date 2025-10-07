#ifndef GO2_JOINTCONTROLLER__GO2_JOINTCONTROLLER_HPP_
#define GO2_JOINTCONTROLLER__GO2_JOINTCONTROLLER_HPP_

#include <vector>
#include <string>
#include <mutex>
#include <filesystem>

#include "controller_interface/controller_interface.hpp"

#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "unitree_ros2/include/go2_motion_switch_client.hpp"

// using namespace unitree::robot::go2;

namespace go2_jointcontroller
{
    using lowCmd = unitree_go::msg::LowCmd;
    using lowStates = unitree_go::msg::LowState;

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


        Eigen::VectorXd computeTotalGravityCompensation(Eigen::VectorXd q);

        Eigen::VectorXd computePID();

        void selectControlMode(int mode);

        uint32_t crc32_core(uint32_t *ptr, uint32_t len);
        void get_crc(lowCmd& msg);

        typedef struct
        {
            uint8_t off; // off 0xA5
            std::array<uint8_t, 3> reserve;
        } BmsCmd;

        typedef struct
        {
            uint8_t mode; // desired working mode
            float q;	  // desired angle (unit: radian)
            float dq;	  // desired velocity (unit: radian/second)
            float tau;	  // desired output torque (unit: N.m)
            float Kp;	  // desired position stiffness (unit: N.m/rad )
            float Kd;	  // desired velocity stiffness (unit: N.m/(rad/s) )
            std::array<uint32_t, 3> reserve;
        } MotorCmd; // motor control

        typedef struct
        {
            std::array<uint8_t, 2> head;
            uint8_t levelFlag;
            uint8_t frameReserve;
                
            std::array<uint32_t, 2> SN;
            std::array<uint32_t, 2> version;
            uint16_t bandWidth;
            std::array<MotorCmd, 20> motorCmd;
            BmsCmd bms;
            std::array<uint8_t, 40> wirelessRemote;
            std::array<uint8_t, 12> led;
            std::array<uint8_t, 2> fan;
            uint8_t gpio;
            uint32_t reserve;
            
            uint32_t crc;
        } LowCmd; 

    protected:
        pinocchio::Model model;
        std::shared_ptr<pinocchio::Data> data;

        Eigen::VectorXd _q;
        Eigen::VectorXd _qd;
        Eigen::VectorXd _tau;
        Eigen::VectorXd _effort;

        std::vector<double> kp;
        std::vector<double> kd;
        std::vector<double> ki;
        Eigen::VectorXd q_e;
        Eigen::VectorXd qi_e;
        Eigen::VectorXd dq_e;

        Eigen::VectorXd qr;
        Eigen::VectorXd dqr;

        int update_rate;

        lowCmd lowCmd_msg;

        double _percent;
        double _duration;
        bool _started;
        std::vector<double> _startPos;
        std::vector<double> _targetPos;
        uint32_t _lowTick;

        rclcpp::Publisher<lowCmd>::SharedPtr joints_cmd_publisher_;

        rclcpp::Subscription<lowCmd>::SharedPtr controller_reference_subscriber_;
        rclcpp::Subscription<lowStates>::SharedPtr lowstate_subscriber_;

        uint32_t control_mode;

        std::mutex mutex_controller;

        std::shared_ptr<unitree::robot::go2::MotionSwitchClient> ms_client_;
        int queryMotionStatus();
        std::string queryServiceName(std::string form, std::string name);
    };

}
#endif