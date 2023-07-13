#include <rclcpp/rclcpp.hpp>
//#include "arp_msgs/srv/FormatPosesFromXML.hpp"

#include <memory>

/**Tests*/
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>

/**
 * @class XMLServiceNode
 * 
 * This class represents a service node that will take in a poseArray and its respective
 * database, and then returns the original PoseArray with a corresponding array of results! :)
 */
class XMLServiceNode : public rclcpp::Node
{
public:
    XMLServiceNode()
    : Node("xml_service_node")
    {
            std::string homedir = getenv("HOME");
            std::cout << homedir << std::endl;
            std::cout << homedir + "../../" << std::endl;
    }
}; //XMLServiceNode

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XMLServiceNode>());
    rclcpp::shutdown();
    return 0;
}