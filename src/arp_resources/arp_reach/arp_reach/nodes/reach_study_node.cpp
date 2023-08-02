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

#include <reach/plugin_utils.h>
#include <reach/interfaces/ik_solver.h>
#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost/algorithm/string.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>

template <typename T>
T get(const std::shared_ptr<rclcpp::Node> node, const std::string& key)
{
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

int main(int argc, char** argv)
{
  try
  {
    // Initialize ROS
    rclcpp::init(argc, argv);

    const YAML::Node config = YAML::LoadFile(get<std::string>(reach_ros::utils::getNodeInstance(), "config_file"));
    const std::string config_name = get<std::string>(reach_ros::utils::getNodeInstance(), "config_name");
    const boost::filesystem::path results_dir(get<std::string>(reach_ros::utils::getNodeInstance(), "results_dir"));

    //sample_request
    //const YAML::Node config = YAML::LoadFile(sample_request->yaml_filepath);
    //const std::string config_name = std::string(sample_request->yaml_name);
    //const boost::filesystem::path results_dir(sample_request->results_dir);
    //
    //Run the reach Study
    //reach::runReachStudy(config, config_name, results_dir, false);

    // Run the reach study
    reach::runReachStudy(config, config_name, results_dir, true);
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
