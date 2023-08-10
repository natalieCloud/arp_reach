/**
 * @author Natalie Chmura
 *
 * This file runs a reach_study based node that functions as a client calling the launch file launched service node! 
 * The client's request is comprised of a  config_file, config_name, reach results directory, and a boolean
 * that indicates when the study can be run, along with the other parameters given by the launch file (which is how this
 * node is spun in the first place as opposed to the other ones that can be launched singularly, the launch parameters
 * help with the reach_ros interfaces that make up the reach study!) It is then sent to the service node, and recives 
 * A filepath to the reach results file, as a well as a boolean indicating that the reach study ran sucessfully!
 */

#include <rclcpp/rclcpp.hpp>
#include "arp_msgs/srv/run_reach_study.hpp"

#include <chrono>
#include <memory>

#include <string>
#include <stdlib.h>
#include <unistd.h>

using namespace std::chrono_literals;

#include <filesystem>
#include <stdio.h>

namespace fs = std::filesystem;

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arp_reach_client");
    rclcpp::Client<arp_msgs::srv::RunReachStudy>::SharedPtr client =
        node->create_client<arp_msgs::srv::RunReachStudy>("run_reach_study");

    auto request = std::make_shared<arp_msgs::srv::RunReachStudy::Request>();
    auto curr = fs::current_path();
    const fs::path sub_yaml = "/install/arp_reach_launch/share/arp_reach/study_config.yaml";
    fs::path curr_yaml = curr / sub_yaml; 

    std::cout << curr_yaml.c_str() << std::endl;

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