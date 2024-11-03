#include <chrono>
#include <deque>
#include <vector>
#include <numeric>
#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include "package_2/srv/get_filtered_data.hpp"

using namespace std::chrono_literals;

class SensorNode : public rclcpp::Node
{
public:
    SensorNode() : Node("sensor_node")
    {
        // Configuração do timer para simular leituras de sensor a cada 1 segundo
        timer_ = this->create_wall_timer(1s, std::bind(&SensorNode::sensor_callback, this));

        // Serviço para obter os últimos 64 valores filtrados
        get_filtered_data_service_ = this->create_service<package_2::srv::GetFilteredData>(
            "get_filtered_data", std::bind(&SensorNode::handle_get_filtered_data, this, std::placeholders::_1, std::placeholders::_2));

        // Serviço para zerar os dados do filtro
        reset_filtered_data_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset_filtered_data", std::bind(&SensorNode::handle_reset_filtered_data, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void sensor_callback()
    {
        // Simula uma leitura de sensor (exemplo: valor aleatório entre 0 e 100)
        double new_sensor_data = static_cast<double>(rand() % 100);

        // Adiciona o novo dado ao buffer e calcula a média móvel
        sensor_data_.push_back(new_sensor_data);
        if (sensor_data_.size() > 5)
        {
            sensor_data_.pop_front(); // Mantém apenas os últimos 5 valores
        }

        // Calcula a média dos últimos 5 valores
        double filtered_value = std::accumulate(sensor_data_.begin(), sensor_data_.end(), 0.0) / sensor_data_.size();

        // Adiciona o valor filtrado ao buffer dos últimos 64 valores
        filtered_data_.push_back(filtered_value);
        if (filtered_data_.size() > 64)
        {
            filtered_data_.pop_front(); // Remove o valor mais antigo se exceder 64
        }

        RCLCPP_INFO(this->get_logger(), "Novo dado do sensor: %.2f, Média Móvel: %.2f", new_sensor_data, filtered_value);
    }

    void handle_get_filtered_data(
        const std::shared_ptr<package_2::srv::GetFilteredData::Request> /*request*/,
        std::shared_ptr<package_2::srv::GetFilteredData::Response> response)
    {

        // Copia os últimos 64 valores filtrados para a resposta
        response->filtered_data = std::vector<double>(filtered_data_.begin(), filtered_data_.end());
        RCLCPP_INFO(this->get_logger(), "Enviando últimos 64 valores filtrados.");
    }

    void handle_reset_filtered_data(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
    {

        // Limpa o buffer de dados filtrados
        filtered_data_.clear();
        RCLCPP_INFO(this->get_logger(), "Dados filtrados zerados.");
    }

    // Atributos do nó
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<package_2::srv::GetFilteredData>::SharedPtr get_filtered_data_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_filtered_data_service_;

    std::deque<double> sensor_data_;   // Buffer para os últimos 5 valores do sensor (para média móvel)
    std::deque<double> filtered_data_; // Buffer para os últimos 64 valores filtrados
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}
