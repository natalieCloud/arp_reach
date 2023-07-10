#include "rclcpp/rclcpp.hpp"
#include "arp_msgs/srv/FormatPosesToPCD.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main (int argc, char **argv) {

    rclcpp::init(argc, argv);
    if(argc != 1) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: pcd_processor_client PoseArray[]");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pcd_processor_client");
    rclcpp::Client<arp_msgs::srv::FormatPosesToPCD>::SharedPtr client = 
        node->create_client<arp_msgs::srv::FormatPosesToPCD>("process_pcd");

    auto request = std::make_shared<arp_msgs::srv::FormatPosesToPCD::Request>();
    request->a = atoll(argv[1]);

    while(!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interuppted while waiting for the service. Exiting execution");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not avaliable, waiting again...");
        }
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sucessfully called service process_pcd! :)\n
        .pcd file: %s", result.get()->pcd_file);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service process_pcd! :(");
    }

    rclcpp::shutdown();
    return 0;
}