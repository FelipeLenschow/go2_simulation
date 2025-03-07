#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <unitree_go/msg/low_state.hpp>
#include "unitree_go/msg/low_cmd.hpp"

using namespace std::chrono_literals;
using lowCmd = unitree_go::msg::LowCmd;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("low_cmd"), motion_time(0), rate_count(0), toggle_pos(false)
    {
        publisher_ = this->create_publisher<lowCmd>("/go2_jointcontroller/JointControllerReferences", 1);
        // publisher_ = this->create_publisher<lowCmd>("/go2_actuator/LowCommands", 1);

        // Começa na posição inicial
        std::copy(std::begin(_targetPos_1), std::end(_targetPos_1), std::begin(_desPos));

        timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto low_cmd = lowCmd();
        motion_time++;

        // Normalização do tempo para interpolação (0 a 1)
        double rate = std::min(1.0, rate_count / 50.0);
        rate_count++;

        // Interpola entre as posições
        for (int j = 0; j < 12; j++)
        {
            low_cmd.motor_cmd[j].q = _desPos[j];
            low_cmd.motor_cmd[j].dq = 0;
            low_cmd.motor_cmd[j].kp = 30.0;
            low_cmd.motor_cmd[j].kd = 3;
            low_cmd.motor_cmd[j].tau = 0;
        }

        // Interpolação entre as posições (1 -> 2 ou 2 -> 1) com base no valor de `toggle_pos`
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd[i].q = jointLinearInterpolation(_startPos[i], _desPos[i], rate);
        }

        low_cmd.reserve = mode;
        publisher_->publish(low_cmd);

        // Alternância entre posições a cada 50 iterações
        if (rate >= 1.0)
        {
            toggle_pos = !toggle_pos;

            // Alterna entre _targetPos_1 e _targetPos_2 com base na direção da interpolação
            if (toggle_pos)
            {
                std::copy(std::begin(_targetPos_1), std::end(_targetPos_1), std::begin(_desPos));   // Interpolação de 1 para 2
                std::copy(std::begin(_targetPos_2), std::end(_targetPos_2), std::begin(_startPos)); // Inicia a interpolação da 2 para 1
            }
            else
            {
                std::copy(std::begin(_targetPos_2), std::end(_targetPos_2), std::begin(_desPos));   // Interpolação de 2 para 1
                std::copy(std::begin(_targetPos_1), std::end(_targetPos_1), std::begin(_startPos)); // Inicia a interpolação de 1 para 2
            }

            rate_count = 0; // Reinicia interpolação
            cout2_ += 1;
            if (cout2_ == 4)
            {
                mode = 1; // Muda o modo para 2
                cout2_ = 0;
            }
        }
    }

    double jointLinearInterpolation(double q0, double qf, double rate)
    {
        return q0 + rate * (qf - q0);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<lowCmd>::SharedPtr publisher_;
    bool toggle_pos = false;
    int motion_time, rate_count;
    float _desPos[12]; // Agora, _desPos é um array de 12 posições.

    int cout2_ = 0;
    uint32_t mode = 1;

    // float _startPos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
    // float _targetPos_1[12] = {0.0, 1.2, -2.05, 0.5, 0.8, -1.55, -0.2, 0.8, -1.55, 0.2, 1.0, -2.1};
    // float _targetPos_2[12] = {0.2, 1.5, -1.8, 0.3, 1.2, -1.25, -0.1, 1.0, -1.2, 0.3, 1.0, -1.6};

    float _startPos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
    float _targetPos_1[12] = {0.0, 1.36, -2.5, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
    float _targetPos_2[12] = {0.0, 1.36, -1.0, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}