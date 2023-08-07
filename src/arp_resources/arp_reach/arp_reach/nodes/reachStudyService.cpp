/**
 * @author Natalie Chmura
 * 
 * @brief This file runs a reach_study based node that functions as a service rather than a free spinning node! 
 * The node takes in the client's request with the config_file, config_name, reach results directory, and a boolean
 * that indicates when the study can be run, along with the other parameters given by the launch file (which is how this
 * node is spun in the first place as opposed to the other ones that can be launched singularly, the launch parameters
 * help with the reach_ros interfaces that make up the reach study!)
*/

#include <reach/reach_study.h>
#include <reach_ros/utils.h>

#include "arp_msgs/srv/run_reach_study.hpp"

#include <stdlib.h>

#include <chrono>
#include <thread>
#include <memory>

#include <yaml-cpp/yaml.h>

// Global **Used rn cause they are not pased as args to the service call! (yet)
YAML::Node config;
std::string config_name;
boost::filesystem::path results_dir;

// **
// Following block of code was developed by Southwest Research Institute, 2019
//
// Accessible at: https://github.com/ros-industrial/reach_ros2
//
template<typename T>
T get(const std::shared_ptr<rclcpp::Node> node, const std::string &key)
{
    T val;
    if (!node->get_parameter(key, val))
        throw std::runtime_error("Failed to get '" + key + "' parameter");
    return val;
}
// **

int run_reach(const std::shared_ptr<arp_msgs::srv::RunReachStudy::Request> request,
        std::shared_ptr<arp_msgs::srv::RunReachStudy::Response> response) {

    try {
        if (!request->signal)
            throw std::runtime_error("Reach study cannot be run. Signal not recived! :(");

        // TODO: In refactor pass in request params over the launch params! 
        
        reach::runReachStudy(config, config_name, results_dir, false);

    } catch (const std::exception &ex) {

        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    config = YAML::LoadFile(get<std::string>(reach_ros::utils::getNodeInstance(), "config_file"));
    config_name = get<std::string>(reach_ros::utils::getNodeInstance(), "config_name");
    boost::filesystem::path results(get<std::string>(reach_ros::utils::getNodeInstance(), "results_dir"));
    results_dir = results;

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arp_reach");
    rclcpp::Service<arp_msgs::srv::RunReachStudy>::SharedPtr service = 
        node->create_service<arp_msgs::srv::RunReachStudy>("run_reach_study", &run_reach);
        
    rclcpp::spin(node);
    rclcpp::shutdown();
}