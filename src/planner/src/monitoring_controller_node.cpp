#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <fmt/core.h>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std;

class MonitoringController : public rclcpp::Node {
public:
    MonitoringController()
    : rclcpp::Node("monitoring_controller") {}

    void init() {
        domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
        problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        
        init_knowledge();
        // execute_plan();
    }

    void set_goal() {
        problem_client_->setGoal(plansys2::Goal("(and(scanned wp1)(scanned wp2))"));
        RCLCPP_INFO(logger_, "The goal has been set");
    }

    void execute_plan() {
        auto domain = domain_client_->getDomain();
        auto problem = problem_client_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
            RCLCPP_ERROR(logger_, "Coudn't find plan to reach goal");
            return;
        }
        
        RCLCPP_INFO(logger_, "Executing plan...");
        executor_client_->start_plan_execution(plan.value());
    }

    void init_knowledge() {
        problem_client_->addInstance(plansys2::Instance{"diff_drive_bot", "robot"});
        problem_client_->addInstance(plansys2::Instance{"wp1", "waypoint"});
        problem_client_->addInstance(plansys2::Instance{"wp2", "waypoint"});

        problem_client_->addPredicate(plansys2::Predicate("(connected wp1 wp2)"));
        problem_client_->addPredicate(plansys2::Predicate("(connected wp2 wp1)"));

        problem_client_->addPredicate(plansys2::Predicate("(robot_at diff_drive_bot wp1)"));

        RCLCPP_INFO(logger_, "The knowledge has been initialized");
    }

private:
      std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
      std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
      std::shared_ptr<plansys2::PlannerClient> planner_client_;
      std::shared_ptr<plansys2::ExecutorClient> executor_client_;
      rclcpp::Logger logger_ = this->get_logger();
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = make_shared<MonitoringController>();
    node->init();
    node->set_goal();
    node->execute_plan();

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}