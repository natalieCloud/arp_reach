#include <rclcpp/rclcpp.hpp>
#include "arp_msgs/srv/format_poses_to_pcd.hpp"

#include "../file_makers_lib/include/interfaces/PoseArrayToPCD.hpp"
#include "../file_makers_lib/include/interfaces/YAMLFileWriter.hpp"

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include "boost/filesystem.hpp"
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <memory>
#include <stdlib.h>
#include <string>

namespace bf = boost::filesystem;

/**
 * @author Natalie Chmura
 * 
 * @brief This node represents a type of service from the FormatPosesToPCD.srv found in the arp_msgs package!
 *  It takes in a request from the client which is comprised of a PoseArray, and then tries to run that 
 *  PoseArray into the pcd processor, and then the resulting pcd filepath into the yaml file creator!
 *  It then returns the yaml filepath and a boolean indicating sucess to the client, indicating whether
 *  or not to turn to phase 2 of the node series! :)
 **/

/**
 * The process_pcd funtion takes in the client requests and returns the yaml filepath and a sucess signal!
*/
void process_pcd(const std::shared_ptr<arp_msgs::srv::FormatPosesToPCD::Request> request,
        std::shared_ptr<arp_msgs::srv::FormatPosesToPCD::Response> response) {
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Called process_pcd");

    bf::path cwd = bf::initial_path();
    bf::path pcd = cwd / "install" / "arp_reach_launch" / "share" / "arp_reach" / "poseArray.pcd";
    bf::path yaml = cwd / "install" / "arp_reach_launch" / "share" / "arp_reach" / "study_config.yaml";

    int pcd_sucess = PCD_PROCESSING_WRITE_PCD::ProcessPCD::PCDFileProcessor::writeFile(request->waypoints, pcd.c_str());

    if(pcd_sucess) {

        std::string fname = "package://arp_reach/poseArray.pcd";

        PCD_PROCESSING_WRITE_YAML::ProcessYAML::YAMLFileWriter::writeYAML(fname, yaml.c_str());
    }

   response->yaml.yaml_filepath = yaml.c_str();
   response->sucess = true;
}

/**
 * Main! Instantiates the node!
*/
int main (int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("process_pcd_service");

    rclcpp::Service<arp_msgs::srv::FormatPosesToPCD>::SharedPtr service =
        node->create_service<arp_msgs::srv::FormatPosesToPCD>("format_poses_to_pcd", &process_pcd);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to format poses.");


    rclcpp::spin(node);
    rclcpp::shutdown();
}