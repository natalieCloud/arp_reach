#include <rclcpp/rclcpp.hpp>
#include "../parsing_lib/include/interfaces/xml_parser.hpp"
#include "arp_msgs/srv/format_poses_from_xml.hpp"

#include <memory>

/**Tests*/
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>

// /**
//  * @class XMLServiceNode
//  * 
//  * This class represents a service node that will take in a poseArray and its respective
//  * database, and then returns the original PoseArray with a corresponding array of results! :)
//  */
// class XMLServiceNode : public rclcpp::Node
// {
// public:
//     XMLServiceNode()
//     : Node("xml_service_node")
//     {
//             std::string homedir = getenv("HOME");
//             std::cout << homedir << std::endl;
//             std::cout << homedir + "../../tmp/reach" << std::endl;
//     }
// }; //XMLServiceNode

// int main (int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<XMLServiceNode>());
//     rclcpp::shutdown();
//     return 0;
// }

void process_xml(const std::shared_ptr<arp_msgs::srv::FormatPosesFromXML::Request> request,
        std::shared_ptr<arp_msgs::srv::FormatPosesFromXML::Response> response) {

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "InComing request recieved.");
}

int main (int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("process_xml_server");

    rclcpp::Service<arp_msgs::srv::FormatPosesFromXML>::SharedPtr service =
        node->create_service<arp_msgs::srv::FormatPosesFromXML>("format_poses_from_xml", &process_xml);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to format poses.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}