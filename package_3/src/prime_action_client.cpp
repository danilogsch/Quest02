#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "package_3/action/find_prime.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals; // For `5s` literal

class PrimeActionClient : public rclcpp::Node
{
public:
    using FindPrime = package_3::action::FindPrime;
    using GoalHandleFindPrime = rclcpp_action::ClientGoalHandle<FindPrime>;

    explicit PrimeActionClient(int target_prime)
        : Node("prime_action_client"), target_prime_(target_prime)
    {
        this->action_client_ = rclcpp_action::create_client<FindPrime>(this, "find_prime");
        this->send_goal();
    }

private:
    rclcpp_action::Client<FindPrime>::SharedPtr action_client_;
    int target_prime_;

    void send_goal()
    {
        if (!this->action_client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "Ação 'find_prime' não disponível.");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = FindPrime::Goal();
        goal_msg.target_prime = target_prime_;

        RCLCPP_INFO(this->get_logger(), "Enviando meta para encontrar o %dº número primo.", target_prime_);

        auto send_goal_options = rclcpp_action::Client<FindPrime>::SendGoalOptions();

        // Goal response callback using a lambda
        send_goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleFindPrime> goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Meta foi rejeitada pelo servidor.");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Meta aceita pelo servidor, aguardando resultado...");
            }
        };

        // Feedback callback using a lambda
        send_goal_options.feedback_callback = [this](GoalHandleFindPrime::SharedPtr,
                                                     const std::shared_ptr<const FindPrime::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Feedback recebido - Primos encontrados: %d, Último primo: %d",
                        feedback->current_prime_count, feedback->latest_prime);
        };

        // Result callback using a lambda
        send_goal_options.result_callback = [this](const GoalHandleFindPrime::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Resultado final: %dº número primo é %d",
                            target_prime_, result.result->result);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Ação foi cancelada.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Ação foi abortada.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Resultado desconhecido.");
                break;
            }
            rclcpp::shutdown();
        };

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Uso: prime_action_client <target_prime>");
        return 1;
    }

    int target_prime = std::stoi(argv[1]);
    auto node = std::make_shared<PrimeActionClient>(target_prime);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}