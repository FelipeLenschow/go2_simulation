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
        : Node("low_cmd"), motion_time(0), rate_count(0)
    {
        publisher_ = this->create_publisher<lowCmd>("/go2_jointcontroller/JointControllerReferences", 1);

        // Inicializa posições
        float startPos[12]   = {0.0, 1.50, -2.65, 0.0, 1.50, -2.65, 0.0, 1.50, -2.65,  0.0, 1.50, -2.65}; //fold
        float targetPos1[12] = {0.5, 1.30, -1.36, 0.0, 1.50, -2.65, 0.0, 1.50, -2.65,  0.0, 1.50, -2.65}; //FL extend
        float targetPos2[12] = {0.5, 1.30, -1.36, 0.0, 1.50, -2.65, 0.0, 1.50, -2.65, -0.5, 1.30, -1.36}; //FL and RR extend 

        std::copy(std::begin(startPos), std::end(startPos), sequence[0]);
        std::copy(std::begin(targetPos1), std::end(targetPos1), sequence[1]);
        // std::copy(std::begin(startPos), std::end(startPos), sequence[2]);
        // std::copy(std::begin(targetPos2), std::end(targetPos2), sequence[3]);
        
        std::copy(std::begin(sequence[0]), std::end(sequence[0]), _startPos);
        std::copy(std::begin(sequence[1]), std::end(sequence[1]), _desPos);

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

        if (paused)
        {
            // Durante pausa, manter posição atual
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd[j].q = _desPos[j];
                low_cmd.motor_cmd[j].dq = 0;
            }

            publisher_->publish(low_cmd);

            pause_counter++;
            if (pause_counter >= pause_duration)
            {
                paused = false;
                pause_counter = 0;

                // Atualiza a nova interpolação
                current_step = (current_step + 1) % sequence_size;
                std::copy(std::begin(_desPos), std::end(_desPos), _startPos);
                std::copy(std::begin(sequence[current_step]), std::end(sequence[current_step]), _desPos);
                rate_count = 0;
            }
            return;
        }

        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd[i].q = _startPos[i] + rate * (_desPos[i] - _startPos[i]);
            low_cmd.motor_cmd[i].dq = 0;
        }

        publisher_->publish(low_cmd);

        // Alternância entre posições a cada 50 iterações
        if (rate >= 1.0)
        {
            // Chegou ao destino, inicia pausa se for targetPos_1
            if (current_step == 0) // 0 representa targetPos1
            {
                paused = true;
                pause_counter = 0;
            }
            else
            {
                // Continua normalmente para o próximo destino
                current_step = (current_step + 1) % sequence_size;
                std::copy(std::begin(_desPos), std::end(_desPos), _startPos);
                std::copy(std::begin(sequence[current_step]), std::end(sequence[current_step]), _desPos);
                rate_count = 0;
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<lowCmd>::SharedPtr publisher_;
    int motion_time, rate_count, pause_counter;
    bool paused = false;

    int current_step = 1;
    static const int sequence_size = 4;
    static const int pause_duration = 20; // número de ciclos de 100ms para pausar (2s)

    float _startPos[12];
    float _desPos[12];
    float sequence[sequence_size][12];
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
