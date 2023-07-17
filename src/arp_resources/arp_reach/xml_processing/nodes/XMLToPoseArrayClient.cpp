#include <rclcpp/rclcpp.hpp>
#include "arp_msgs/srv/format_poses_from_xml.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

/** TEST INCLUDES */
#include <string>
#include <stdlib.h>

using namespace std::chrono_literals;

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("process_xml_client");
    rclcpp::Client<arp_msgs::srv::FormatPosesFromXML>::SharedPtr client =
        node->create_client<arp_msgs::srv::FormatPosesFromXML>("format_poses_from_xml");
    
    auto request = std::make_shared<arp_msgs::srv::FormatPosesFromXML::Request>();

    std::string extension = "/../../tmp/reach.db.xml";
    std::string homedir = getenv("HOME") + extension;

    request->xml_filepath = homedir;

    auto poseArr = geometry_msgs::msg::PoseArray();
    poseArr.header.frame_id = "world";
    /** Include initilizing a few poses here from the issue 6 folder
     * I dont feel like doing this rnnnnnnnn
    */
    auto pose1 = geometry_msgs::msg::Pose();
    pose1.position.x = 2.4223447;
    pose1.position.y = -0.36079797;
    pose1.position.z = 1.2388842;
    pose1.orientation.x = -0.48066208;
    pose1.orientation.y = -0.38725615;
    pose1.orientation.z = -0.78676349;
    pose1.orientation.w = 0.0;

    poseArr.poses.push_back(pose1);

    request->waypoints = poseArr;

    while (!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not active, waiting for activity...");
    }

    auto result = client->async_send_request(request);

    if(rclcpp::spin_until_future_complete(node, result) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sucessfully called process_xml! :)");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call process_xml! :(");
    }

    rclcpp::shutdown();
    return 0;
}