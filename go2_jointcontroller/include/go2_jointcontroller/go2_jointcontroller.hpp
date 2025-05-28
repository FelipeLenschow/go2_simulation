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

#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"

#include <filesystem>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

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

        void computeG();

        void computeG12();

        void computeG0();

        Eigen::VectorXd computePD();

        Eigen::VectorXd computePD_COMPG();

        void computeTotalGravityCompensation();

        Eigen::VectorXd computePID();

        Eigen::VectorXd computePID_COMPG();

        void computeTauG();

        void selectControlMode(int mode);

        std::vector<int> map_desired_to_pinocchio;

        // std::vector<int> map_desired_to_pinocchio(const std::vector<std::string> &desired_order,
        //                                           const std::vector<std::string> &pinocchio_order);

        // Reordena um vetor da ordem desejada para a ordem do Pinocchio
        Eigen::VectorXd reorder_to_pinocchio(const Eigen::VectorXd &vec_desired);

        // Reordena um vetor da ordem do Pinocchio para a ordem desejada
        Eigen::VectorXd reorder_to_desired(const Eigen::VectorXd &vec_pinocchio);

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
        std::vector<std::string> joint_names_;

        pinocchio::Model model;
        std::shared_ptr<pinocchio::Data> data;

        Eigen::VectorXd gravidade;
        Eigen::VectorXd q;
        Eigen::VectorXd dq;
        std::vector<double> kp;
        std::vector<double> kd;
        std::vector<double> ki;
        Eigen::VectorXd tau;
        Eigen::VectorXd tauG;
        Eigen::VectorXd tauG_total;
        Eigen::VectorXd tauG_local;
        Eigen::VectorXd tau_pinocchio;
        Eigen::VectorXd torque_contrib;

        Eigen::VectorXd tau_;
        Eigen::VectorXd q_e;
        Eigen::VectorXd qi_e;
        Eigen::VectorXd dq_e;
        Eigen::VectorXd qr;
        Eigen::VectorXd dqr;
        Eigen::VectorXd mass;
        Eigen::VectorXd effort;
        Eigen::VectorXd commanded_effort;
        Eigen::VectorXd vetor_pinocchio;
        Eigen::VectorXd v;
        Eigen::VectorXd a;
        Eigen::MatrixXd J;
        Eigen::MatrixXd J_linear;
        Eigen::VectorXd tau_contrib;
        Eigen::Vector3d F_grav;

        Eigen::VectorXd q_pinocchio;
        Eigen::VectorXd v_pinocchio;
        Eigen::VectorXd a_pinocchio;

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

        //////////////////////////////////////////////////////////
        const std::vector<std::string> desired_order = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

        const std::vector<std::string> pinocchio_order = {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

        std::vector<pinocchio::FrameIndex> pinocchio_frames;

        const std::vector<std::string> joint_names_sequence = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    };

}
#endif