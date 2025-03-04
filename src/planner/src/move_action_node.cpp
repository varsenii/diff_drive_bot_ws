#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
    MoveAction()
    : plansys2::ActionExecutorClient("move", 250ms)
    {
        progress_ = 0;
    }

private:
    void do_work()
    {
        if (progress_ < 1.0) {
            progress_ += 0.2;
            send_feedback(progress_, "Move running");
        } else {
            finish(true, 1.0, "Move completed");
            RCLCPP_INFO(get_logger(), "Move completed");
        }
    }

    float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}