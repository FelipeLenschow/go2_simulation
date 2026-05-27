#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "cmath"

#include <vector>
#include <array>

using namespace std::chrono_literals;

using lowCmd = unitree_go::msg::LowCmd;
using lowStates = unitree_go::msg::LowState;

struct TrajectoryPoint {
    std::array<float, 12> pos;
    double duration_ms;
};

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("low_cmd")
        , motion_time(0)
        , rate_count(0)
    {
        publisher_ = this->create_publisher<lowCmd>("/go2_jointcontroller/JointControllerReferences", 1);

        lowstate_subscriber_ = this->create_subscription<lowStates>(
            "/lowstate", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<lowStates> msg) -> void
            {
                if (!started)
                {
                    for (auto index{0}; index < 12; index++)
                    {
                        _q[index] = msg->motor_state[index].q;
                    }
                    std::copy(std::begin(_q), std::end(_q), _startPos);
                    started = true;
                }
            });

        // Inicializa posições (0 a 11)
        std::array<float, 12> foldPos = {0.0, 1.50, -2.65, 0.0, 1.50, -2.65, 0.0, 1.50, -2.65,  0.0, 1.50, -2.65};
        std::array<float, 12> extendFR = {0.0, -1.00, -1.00, 0.0, 1.50, -2.65, 0.0, 1.50, -2.65,  0.0, 1.50, -2.65};

        sequence.push_back({foldPos, 10000.0});   // Vai para fold em 10 segundos
        sequence.push_back({foldPos, 5000.0});   // Continua em fold por 3 segundos
        sequence.push_back({extendFR, 15000.0}); // Estende a perna em 15 segundos
        sequence.push_back({foldPos, 15000.0});  // Volta para fold em 15 segundos
        // sequence.push_back({extendFR, 1500.0});  // Degrau rápido na perna em 1.5 segundo
        // sequence.push_back({foldPos, 1500.0});   // Degrau rápido de volta em 1.5 segundo

        std::copy(sequence[0].pos.begin(), sequence[0].pos.end(), _desPos);

        timer_ = this->create_wall_timer(1ms, std::bind(&MinimalPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        if(!started) return;

        auto low_cmd = lowCmd();
        motion_time++;

        // Normalização do tempo para interpolação (0 a 1) baseada na duração do passo atual
        double duration = sequence[current_step].duration_ms;
        double rate = std::min(1.0, rate_count / duration);
        rate_count++;


        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd[i].q = _startPos[i] + rate * (_desPos[i] - _startPos[i]);
            low_cmd.motor_cmd[i].dq = 0;
        }

        publisher_->publish(low_cmd);

        // Alternância entre posições quando a interpolação terminar
        if (rate >= 1.0)
        {
            // Continua normalmente para o próximo destino
            current_step = (current_step + 1) % sequence.size();
            std::copy(std::begin(_desPos), std::end(_desPos), _startPos);
            std::copy(sequence[current_step].pos.begin(), sequence[current_step].pos.end(), _desPos);
            rate_count = 0;
        }

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<lowCmd>::SharedPtr publisher_;
    rclcpp::Subscription<lowStates>::SharedPtr lowstate_subscriber_;

    int motion_time, rate_count, pause_counter;
    bool paused = false;

    int current_step = 0;

    float _q[12];
    float _qd[12];

    float _startPos[12];
    float _desPos[12];
    std::vector<TrajectoryPoint> sequence;

    bool started = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
