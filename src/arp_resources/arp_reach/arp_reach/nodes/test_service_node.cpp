/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach/reach_study.h>
#include <reach_ros/utils.h>

#include "arp_msgs/srv/run_reach_study.hpp"

#include <stdlib.h>

#include <chrono>
#include <thread>
#include <memory>
#include <yaml-cpp/yaml.h>

YAML::Node config;
std::string config_name;
boost::filesystem::path results_dir;

template <typename T>
T get(const std::shared_ptr<rclcpp::Node> node, const std::string &key)
{
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

void run_reach(const std::shared_ptr<arp_msgs::srv::RunReachStudy::Request> request,
        std::shared_ptr<arp_msgs::srv::RunReachStudy::Response> response) {
  reach::runReachStudy(config, config_name, results_dir, true);
        }

int main(int argc, char **argv)
{
  try
  {

    // Initialize ROS
    rclcpp::init(argc, argv);

    // Initilize enviornment variables!
    // int err = setenv("REACH_PLUGINS", "reach_ros_plugins", 1);
    // if (err != 0)
    // {
    //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Plugin environment failed to set. Aborting...");
    //   return 1; // If the plugin enviornment has not been set than the reach study is inviable!
    // }

    config = YAML::LoadFile(get<std::string>(reach_ros::utils::getNodeInstance(), "config_file"));
    config_name = get<std::string>(reach_ros::utils::getNodeInstance(), "config_name");
    boost::filesystem::path results(get<std::string>(reach_ros::utils::getNodeInstance(), "results_dir"));
    results_dir = results;

    // Run the reach study
    // reach::runReachStudy(config, config_name, results_dir, true);
    // Initilize the service node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arp_reach");
    rclcpp::Service<arp_msgs::srv::RunReachStudy>::SharedPtr service = 
        node->create_service<arp_msgs::srv::RunReachStudy>("run_reach_study", &run_reach);

    rclcpp::spin(node);
    rclcpp::shutdown();

  }
  catch (const std::exception &ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
