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

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include "boost/filesystem.hpp"
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <stdlib.h>

#include <chrono>
#include <thread>
#include <memory>

#include <yaml-cpp/yaml.h>


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

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sucessfully called arp_reach service!");

    if (!request->signal) {
            
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "State not ready to call service, aborting.");
        response->sucess = false;
        return 1;
    }

    try {

        YAML::Node config = YAML::LoadFile(request->yaml.yaml_filepath);
        std::string config_name = request->config_name;
        boost::filesystem::path results_dir(request->results_dir);

        reach::runReachStudy(config, config_name, results_dir, false);

        response->sucess = true;
        response->message = "Sucessfully ran reach study!";

        //Get full path to reults dir
        boost::filesystem::path results_path = absolute(results_dir, boost::filesystem::initial_path());

        response->results.xml_filepath = results_dir.c_str();

    } catch (const std::exception &ex) {

        std::cerr << ex.what() << std::endl;
        response->sucess = false;
        response->message = ex.what();
        return 1;
    }
    return 0;
}


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arp_reach");
    rclcpp::Service<arp_msgs::srv::RunReachStudy>::SharedPtr service = 
        node->create_service<arp_msgs::srv::RunReachStudy>("run_reach_study", &run_reach);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to run reach study!");
        
    rclcpp::spin(node);
    rclcpp::shutdown();
}