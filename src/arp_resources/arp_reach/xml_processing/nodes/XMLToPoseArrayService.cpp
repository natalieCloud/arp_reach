#include <rclcpp/rclcpp.hpp>
#include "../parsing_lib/include/interfaces/xml_parser.hpp"
#include "../parsing_lib/include/interfaces/result_parsing.hpp"
#include "arp_msgs/srv/format_poses_from_xml.hpp"

#include <memory>

#include <stdlib.h>
#include <vector>

/**
 * @author Natalie Chmura
 * 
 * @brief This process instantiates a node that handles the service from FormatPosesFromXml.srv. It first takes the request 
 * of the XML filepath and PoseArray from the client and then pases those off, first to the xml_parser, which 
 * reads in the data from the file provided and outputs a data struct, which is then parsed by the result_parser, 
 * which uses a multi-threaded parser to contruct a score array, which is then passed back to the client!
*/

/**
 * The process_xml function takes in the client requests and returns the score array, orignal pose and boolean sucess!
*/
void process_xml(const std::shared_ptr<arp_msgs::srv::FormatPosesFromXML::Request> request,
        std::shared_ptr<arp_msgs::srv::FormatPosesFromXML::Response> response) {

    if (!request->signal) {
            
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "State not ready to call service, aborting.");
        response->sucess = false;
        return;
    }
        
    try {

        std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> poseMap = XML_PARSING_XML_PARSER_H::ReachXML::XMLParser::parseMap(request->xml.xml_filepath);
        std::vector<double> results = XML_PROCESSING_RESULT_THREADING_H::Scorter::Retriever::getScoreData(request->waypoints, poseMap, request->waypoints.poses.size());
        response->scores = results;
        response->sucess = true;

    } catch (const std::exception &ex) {

        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), ex.what());
        response->sucess = false;

        return;
    }
    
    response->waypoints = request->waypoints;
}

/**
 * Main! Instantiates the node!
*/
int main (int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("process_xml_server");

    rclcpp::Service<arp_msgs::srv::FormatPosesFromXML>::SharedPtr service =
        node->create_service<arp_msgs::srv::FormatPosesFromXML>("format_poses_from_xml", &process_xml);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to format poses.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
