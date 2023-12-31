#include <rclcpp/rclcpp.hpp>
#include "arp_msgs/srv/format_poses_from_xml.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include "boost/filesystem.hpp"
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <chrono>
#include <cstdlib>
#include <memory>

/** TERMINAL TEST INCLUDES */
#include <string>
#include <stdlib.h>

using namespace std::chrono_literals; //For timer! Unessesary otherwise!
namespace bf = boost::filesystem;


/**
 * @author Natalie Chmura
 * 
 * @brief This class instantiates a node that handles the request from FormatPosesFromXml.srv. It first pases 
 * in a string representing the filepath of the reach_ros reach study results (Formatted in XMl), as well 
 * as a PoseArray of all the points that we wish to extract the reach results for. Then it pases these to 
 * the service which will return the original PoseArray as well as a corresponding array of reach scores 
 * back to the client!
*/

/**
 * Main! Instantiates the node and passes the request of waypoints along to the server node, which then
 * returns those points along with an array of reach scores and a sucess boolean!
*/
int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("process_xml_client");
    rclcpp::Client<arp_msgs::srv::FormatPosesFromXML>::SharedPtr client =
        node->create_client<arp_msgs::srv::FormatPosesFromXML>("format_poses_from_xml");
    
    auto request = std::make_shared<arp_msgs::srv::FormatPosesFromXML::Request>();

    bf::path cwd = bf::initial_path();
    bf::path results = cwd / "install" / "reach_ros" / "share" / "reach_ros" / "study_config" / "reach.db.xml";

    request->xml.xml_filepath = results.c_str();

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
    request->signal = true;

    while (!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not active, waiting for activity...");
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