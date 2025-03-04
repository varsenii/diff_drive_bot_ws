#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ScanAction : public plansys2::ActionExecutorClient {
public:
    ScanAction()
    : plansys2::ActionExecutorClient("scan", 250ms) {
        progress_ = 0;
    }

private: 
    void do_work() {
        if (progress_ < 1.0) {
            progress_ += 0.2;
            send_feedback(progress_, "Scan running");
        } else {
            finish(true, 1.0, "Scan completed");
            RCLCPP_INFO(get_logger(), "Scan completed");
        }
    }

    float progress_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ScanAction>();
    node->set_parameter(rclcpp::Parameter("action_name", "scan"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
}