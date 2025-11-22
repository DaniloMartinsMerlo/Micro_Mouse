#include <rclcpp/rclcpp.hpp>
#include "cg_interfaces/srv/move_cmd.hpp"

#include <memory>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class MoveClient : public rclcpp::Node {
public:
    MoveClient() : Node("move_client") {

        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

        // Lista de comandos
        comandos_ = {
            "right", "right", "down", "down", "right", "right", "down", "down", "left", "left", "left", "left", "down", "down", "right", "right", "right", "right", "right", "right", "right", "right", "up", "up", "up", "up", "left", "left", "up", "up", "right", "right", "right", "right", "right", "right", "down", "down", "right", "right", "up", "up", "right", "right", "down", "down", "right", "right", "right", "right", "down", "down", "right", "right", "up", "up", "right", "right", "right", "right", "down", "down", "down", "down", "left", "left", "down", "down", "left", "left", "left", "left", "up", "up", "left", "left", "left", "left", "up", "up", "left", "left", "down", "down", "left", "left", "up", "up", "left", "left", "down", "down", "down", "down", "down", "down", "right", "right", "down", "down", "down", "right"
        };

        // Timer para enviar os comandos um por um
        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&MoveClient::send_next_command, this)
        );
    }

private:

    void send_next_command() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Esperando serviço /move_command...");
            return;
        }

        if (index_ >= comandos_.size()) {
            RCLCPP_INFO(this->get_logger(), "Todos os comandos enviados!");
            rclcpp::shutdown();
            return;
        }

        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = comandos_[index_];

        RCLCPP_INFO(this->get_logger(), "Enviando comando: %s", request->direction.c_str());

        client_->async_send_request(request);

        index_++;
    }

    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> comandos_;
    size_t index_ = 0;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveClient>());
    rclcpp::shutdown();
    return 0;
}