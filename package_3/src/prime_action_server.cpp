#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "package_3/action/find_prime.hpp"

using namespace std::placeholders;

class PrimeActionServer : public rclcpp::Node {
public:
    using FindPrime = package_3::action::FindPrime;
    using GoalHandleFindPrime = rclcpp_action::ServerGoalHandle<FindPrime>;

    PrimeActionServer() : Node("prime_action_server") {
        action_server_ = rclcpp_action::create_server<FindPrime>(
            this,
            "find_prime",
            std::bind(&PrimeActionServer::handle_goal, this, _1, _2),
            std::bind(&PrimeActionServer::handle_cancel, this, _1),
            std::bind(&PrimeActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<FindPrime>::SharedPtr action_server_;

    // Verificação de número primo
    bool is_prime(int num) {
        if (num <= 1) return false;
        for (int i = 2; i * i <= num; ++i) {
            if (num % i == 0) return false;
        }
        return true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FindPrime::Goal> goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Recebendo solicitação para encontrar o %dº número primo", goal->target_prime);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFindPrime> goal_handle) {
            (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancelando solicitação");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFindPrime> goal_handle) {
        std::thread{std::bind(&PrimeActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFindPrime> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FindPrime::Feedback>();
        auto result = std::make_shared<FindPrime::Result>();

        int prime_count = 0;
        int current_number = 2;

        while (rclcpp::ok()) {
            if (is_prime(current_number)) {
                prime_count++;
                feedback->current_prime_count = prime_count;
                feedback->latest_prime = current_number;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Primo %d encontrado: %d", prime_count, current_number);

                if (prime_count == goal->target_prime) {
                    result->result = current_number;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Ação concluída: %dº número primo é %d", goal->target_prime, current_number);
                    return;
                }
            }
            current_number++;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PrimeActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}