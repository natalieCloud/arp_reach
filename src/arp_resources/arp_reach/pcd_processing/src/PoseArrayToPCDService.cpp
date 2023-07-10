#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "arp_msgs/srv/FormatPosesToPCD.hpp"

namespace pcd_processing
{
class PCDProcessClient : public rclcpp::Node
{
public:
    PCDProcessClient()
    : Node("pcd_process_client_node")
    {

    }
};

} // namespace pcd_processing

int main (int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcd_processing::PCDProcessClient>());
    rclcpp::shutdown();
    return 0;
}