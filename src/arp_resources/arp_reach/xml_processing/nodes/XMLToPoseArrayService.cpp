#include <rclcpp/rclcpp.hpp>
#include "../parsing_lib/include/interfaces/xml_parser.hpp"
#include "../parsing_lib/include/interfaces/result_parsing.hpp"
#include "arp_msgs/srv/format_poses_from_xml.hpp"

#include <memory>

#include <stdlib.h>
#include <vector>

void process_xml(const std::shared_ptr<arp_msgs::srv::FormatPosesFromXML::Request> request,
        std::shared_ptr<arp_msgs::srv::FormatPosesFromXML::Response> response) {

    std::map<XML_PROCESSING_POSTRUCTS_H::Postructs::PoseData, XML_PROCESSING_POSTRUCTS_H::Postructs::ResultData> poseMap = XML_PARSING_XML_PARSER_H::ReachXML::XMLParser::parseMap(request->xml_filepath.c_str());

    std::vector<_Float64> results = XML_PROCESSING_RESULT_THREADING_H::Scorter::Retriever::getScoreData(request->waypoints, poseMap, request->waypoints.poses.size());
    _Float64 scoarr[request->waypoints.poses.size()];

    //Assign response values!
    response->scores = std::copy(results.begin(), results.end(), scoarr);
    response->waypoints = request->waypoints;
    //response->scores = results.data();
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Results: %f", results[0]);
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