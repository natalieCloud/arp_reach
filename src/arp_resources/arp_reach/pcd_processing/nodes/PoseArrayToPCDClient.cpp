#include <rclcpp/rclcpp.hpp>
#include "arp_msgs/srv/format_poses_to_pcd.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <chrono>
#include <memory>

#include <string>

using namespace std::chrono_literals; //For timer! Unessesary otherwise!

/**
 * @author Natalie Chmura
 * 
 * @brief This class represents a type of client node from the FormatPosesToPCD.srv found 
 * in the arp_msgs package! The client sends a request that is comprised of a PoseArray to 
 * the service, which then processes it into a pcd file, whose filepath is then included 
 * into a yaml file (Used as the configuration file for arp_reach) The path to the configuration
 * file is then handed back to the client, along with a boolean indicating sucess!
*/

/**
 * Main! Instantiates the node and passes the request of waypoints (of type PoseArray) along to the server node,
 * returns a filepath to a generated YAML file (which contians a path to the genrated pcd file)
 * as well as a boolean indicationg sucess!
*/
int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("process_pcd_client");
    rclcpp::Client<arp_msgs::srv::FormatPosesToPCD>::SharedPtr client =
        node->create_client<arp_msgs::srv::FormatPosesToPCD>("format_poses_to_pcd");

    auto request = std::make_shared<arp_msgs::srv::FormatPosesToPCD::Request>();

    auto poseArr = geometry_msgs::msg::PoseArray();
    poseArr.header.frame_id = "world";
    /** 
     * Test pose to ensure that the request-response is valid!
    */
    auto pose1 = geometry_msgs::msg::Pose();
    pose1.position.x = 2.4766912;
    pose1.position.y = 0.22562733;
    pose1.position.z = 1.244387;
    pose1.orientation.x = -0.093562581;
    pose1.orientation.y = 0.0089845313;
    pose1.orientation.z = -0.99557281;
    pose1.orientation.w = 0.0;

    auto pose2 = geometry_msgs::msg::Pose();
    pose2.position.x = 0.5;
    pose2.position.y = 0.1;
    pose2.position.z = 2.0;
    pose2.orientation.x = 0;
    pose2.orientation.y = 0;
    pose2.orientation.z = 0;
    pose2.orientation.w = 0;

    poseArr.poses.push_back(pose1);
    poseArr.poses.push_back(pose2);

    request->waypoints = poseArr;

    while (!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrrupted while waitin for the service... exiting :(");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not active, waiting for activity... :/");
    }

    auto result = client->async_send_request(request);

    if(rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sucessfully called process pcd! :)");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to call process pcd! :(");
    }

    rclcpp::shutdown();
    return 0;
}