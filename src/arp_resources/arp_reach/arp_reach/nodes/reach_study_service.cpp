#include <reach/reach_study.h>

#include "arp_msgs/srv/run_reach_study.hpp"
#include "rclcpp/rclcpp.hpp"


#include <chrono>
#include <thread>
#include <memory>

#include <yaml-cpp/yaml.h>

void run_reach(const std::shared_ptr<arp_msgs::srv::RunReachStudy::Request> request,
        std::shared_ptr<arp_msgs::srv::RunReachStudy::Response> response) {

    try
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "testing try");
        const YAML::Node config = YAML::LoadFile(request->yaml_filepath.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "testing yaml");
        const std::string config_name = request->config_name.c_str();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "testing config name");
        const boost::filesystem::path results_dir(request->results_dir.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service called, about to ruin reach node");
        reach::runReachStudy(config, config_name, results_dir, false);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reach node called! check the directory!");

        boost::filesystem::path subdir(config_name);
        boost::filesystem::path file("reach.db.xml");
        boost::filesystem::path full_path = results_dir / subdir / file;

        response->results_filepath = full_path.generic_string();
        response->sucess = true;
        response->message = "Sucessfully called reach study!";

    }
    catch (const std::exception &ex) {

        boost::filesystem::path results_dir(request->results_dir);
        boost::filesystem::path subdir(request->config_name);
        boost::filesystem::path file("reach.db.xml");
        boost::filesystem::path full_path = results_dir / subdir / file;

        response->results_filepath = full_path.generic_string();
        response->sucess = false;
        response->message = ex.what();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arp_reach");
    rclcpp::Service<arp_msgs::srv::RunReachStudy>::SharedPtr service = 
        node->create_service<arp_msgs::srv::RunReachStudy>("run_reach_study", &run_reach);
        
    rclcpp::spin(node);
    rclcpp::shutdown();
}