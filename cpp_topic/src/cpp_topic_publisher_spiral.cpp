// #include <chrono>
// #include <memory>
// #include "rclcpp/rclcpp.hpp"
// #include "go2_interfaces/msg/low_cmd.hpp"

// using namespace std::chrono_literals;

// class MinimalPublisher : public rclcpp::Node
// {
// public:
//     MinimalPublisher()
//         : Node("low_cmd")
//     {
//         publisher_ = this->create_publisher<go2_interfaces::msg::LowCmd>("/go2_controller/LowReferences", 1);
//         i = 0.0;
//         //  toggle_pos = false;
//         //  _desPos = _startPos; // Inicializa com a posição inicial
//         timer_ = this->create_wall_timer(3s, std::bind(&MinimalPublisher::publish_message, this));
//     }

// private:
//     void publish_message()
//     {

//         auto low_cmd = go2_interfaces::msg::LowCmd();

//         if (toggle_pos)
//         {
//             _desPos = _startPos;
//             toggle_pos = false;
//         }
//         else
//         {
//             _desPos = _targetPos_1;
//             toggle_pos = true;
//         }

//         for (int j = 0; j < 12; j++)
//         {
//             // low_cmd.motor_cmd[j].q = (1 - i) * _startPos[j] + i * _targetPos_1[j];

//             low_cmd.motor_cmd[j].q = _desPos[j];

//             low_cmd.motor_cmd[j].dq = 0;
//             low_cmd.motor_cmd[j].kp = 60.0;
//             low_cmd.motor_cmd[j].kd = 5.0;
//             low_cmd.motor_cmd[j].tau = 0;
//         }

//         publisher_->publish(low_cmd);

//         if (i >= 1)
//             i = 1;
//         else
//             i += 0.0001;
//         // RCLCPP_INFO(this->get_logger(), "'%f'", i);
//     }

//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<go2_interfaces::msg::LowCmd>::SharedPtr publisher_;
//     float i;

//     bool toggle_pos = false;
//     float *_desPos;

//     // ThreadPtr lowCmdWriteThreadPtr;
//     // float _startPos[12] = {-0.05, 0.05, -1.18, -0.05, 0.05, -1.18, -0.05, -1.5, 1.18, -0.05, -1.5, 1.18};

//     float _startPos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

//     float _targetPos_1[12] = {0.0, 1.0, -1.05, 0.5, 0.8, -1.05, -0.2, 0.5, -1.05, 0.2, 0.5, -1.5};
//     // float _targetPos_1[12] = {0.0, 0.5, -1.05, 0.0, 0.5, -1.05, -0.2, 0.5, -1.05, 0.2, 0.5, -1.05};

//     //   Eigen::VectorXd _startPos, _targetPos_1;
//     //     _startPos.resize(12, 1);
//     //     targetPos_1.resize (12, 1);
//     //     _startPos << 0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65;
//     //     targetPos_1 << 0.0, 0.5, -1.05, 0.0, 0.5, -1.05, -0.2, 0.5, -1.05, 0.2, 0.5, -1.05;

//     // float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
//     //                           0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

//     // float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
//     //                           -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

//     // float _startPos[12];
//     // float _duration_1 = 500;
//     // float _duration_2 = 500;
//     // float _duration_3 = 1000;
//     // float _duration_4 = 900;
//     // float _percent_1 = 0;
//     // float _percent_2 = 0;
//     // float _percent_3 = 0;
//     // float _percent_4 = 0;

//     // bool firstRun = true;
//     // bool done = false;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MinimalPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "go2_interfaces/msg/low_cmd.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("low_cmd")
    {
        publisher_ = this->create_publisher<go2_interfaces::msg::LowCmd>("/go2_controller/LowReferences", 1);
        toggle_pos = false;
        _desPos = _startPos;                                                                          // Começa na posição inicial
        timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::publish_message, this)); // Publica a cada 100ms
    }

private:
    void publish_message()
    {
        auto low_cmd = go2_interfaces::msg::LowCmd();

        // Publica continuamente a posição alvo
        for (int j = 0; j < 12; j++)
        {
            low_cmd.motor_cmd[j].q = _desPos[j]; // Mantém publicando a posição desejada
            low_cmd.motor_cmd[j].dq = 0;
            low_cmd.motor_cmd[j].kp = 60.0;
            low_cmd.motor_cmd[j].kd = 5.0;
            low_cmd.motor_cmd[j].tau = 0;
        }

        publisher_->publish(low_cmd); // Publica a posição continuamente

        // Simulação de condição para troca de alvo (substituir por uma checagem real)
        static int count = 0;
        count++;
        if (count >= 50) // Simula mudança após algumas iterações
        {
            toggle_pos = !toggle_pos;
            _desPos = toggle_pos ? _targetPos_1 : _startPos;
            count = 0; // Reseta a contagem
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<go2_interfaces::msg::LowCmd>::SharedPtr publisher_;
    bool toggle_pos;
    float *_desPos;

    float _startPos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
    float _targetPos_1[12] = {0.0, 1.0, -1.05, 0.5, 0.8, -1.05, -0.2, 0.5, -1.05, 0.2, 0.5, -1.5};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
