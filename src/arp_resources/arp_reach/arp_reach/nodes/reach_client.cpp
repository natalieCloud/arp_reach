#include <rclcpp/rclcpp.hpp>
#include "arp_msgs/srv/run_reach_study.hpp"

#include <chrono>
#include <memory>

#include <string>
#include <stdlib.h>
#include <unistd.h>

using namespace std::chrono_literals;

//TODO Change:
#include <filesystem>
#include <stdio.h>

namespace fs = std::filesystem;

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arp_reach_client");
    rclcpp::Client<arp_msgs::srv::RunReachStudy>::SharedPtr client =
        node->create_client<arp_msgs::srv::RunReachStudy>("run_reach_study");

    auto request = std::make_shared<arp_msgs::srv::RunReachStudy::Request>();

    //std::string yaml = "/arpaint_project/arp_reach_ws/arp_reach/src/arp_resources/arp_reach/arp_reach/config/study_config.yaml";
    //auto cwd = getcwd();
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), cwd);
    auto curr = fs::current_path();
    //std::cout << curr.c_str() << std::endl;
    const fs::path sub_yaml = "/arp_reach/src/arp_resources/arp_reach/arp_reach/config/study_config.yaml";
    fs::path curr_yaml = curr / sub_yaml; 

    std::cout << curr_yaml.c_str() << std::endl;

    //std::string yamldir = getenv("HOME") + yaml;

    request->yaml.yaml_filepath = curr_yaml.c_str();

    request->config_name = "study_config";

    std::string results = "arpaint_project/arp_reach_ws/install/reach_ros/share/reach_ros";
    std::string resultsdir = getenv("HOME") + results;
    request->results_dir = resultsdir;

    request->signal = true;
    
    while(!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting forservice. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not active, waiting for activity.....");
    }

    auto result = client->async_send_request(request);

    if(rclcpp::spin_until_future_complete(node, result) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sucessfully called arp_reach! :)");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call arp_reach! :(");
    }

    rclcpp::shutdown();
    return 0;
}