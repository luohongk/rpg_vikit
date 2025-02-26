/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Author: cforster
 *  Update on: Feb 01, 2025
 *      Author: StrangeFly
 *
 * from libpointmatcher_ros
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <future>
#include <cstdio>
#include <stdexcept>
#include <sstream>

namespace vk {

template <typename T>
T getParam(const std::string& node, const std::string& name, const T& defaultValue) {
    try {
        std::promise<std::string> resultPromise;
        std::future<std::string> resultFuture = resultPromise.get_future();

        std::thread([node, name, promise = std::move(resultPromise)]() mutable {
            try {
                std::string command = "ros2 param get /" + node + " " + name;
                std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
                if (!pipe) {
                    throw std::runtime_error("popen() failed!");
                }
                std::ostringstream resultStream;
                char buffer[128];
                while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
                    resultStream << buffer;
                }
                std::string result = resultStream.str();

                // parse the result
                size_t pos = result.find(": ");
                if (pos != std::string::npos) {
                    promise.set_value(result.substr(pos + 2)); // return the substring after ":"
                } else {
                    promise.set_value(""); // if not found ":", return empty string
                }
            } catch (const std::exception& e) {
                promise.set_exception(std::make_exception_ptr(e));
            }
        }).join();

        std::string valueStr = resultFuture.get();

        // size check
        if (!valueStr.empty()) {
            // type covert
            std::stringstream ss(valueStr);
            T value;
            if (ss >> value) {
                return value;
            } else {
                return defaultValue; // convert failed, return default value
            }
        } else {
            return defaultValue; // size 0 , return default value
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return defaultValue; // some exception, return default value
    }
}

template <typename T>
T getParam(const std::string& node, const std::string& name) {
    // override function could has no default value
    if constexpr (std::is_same_v<T, std::string>) {
        return getParam<T>(node, name, ""); // if std::string, default value is ""
    } else if constexpr (std::is_integral_v<T>) {
        return getParam<T>(node, name, 0); // if int. defalt 0
    } else if constexpr (std::is_floating_point_v<T>) {
        return getParam<T>(node, name, 0.0); // if float，default 0.0
    } else {
        throw std::runtime_error("Unsupported type for getParam without default value.");
    }
}

inline
bool hasParam(const rclcpp::Node::SharedPtr &nh, const std::string& name)
{
  return nh->has_parameter(name);
}

template<typename T>
T getParam(const rclcpp::Node::SharedPtr &nh, const std::string& name, const T& defaultValue)
{
  T v;
  if(nh->has_parameter(name))
  {
    v = nh->get_parameter(name).get_value<T>();
    std::ostringstream oss;
    oss << v;
    RCLCPP_INFO(nh->get_logger(), "Found parameter: %s, value: %s", name.c_str(), oss.str().c_str());
    return v;
  }
  else
  {
    std::ostringstream oss;
    oss << defaultValue;
    RCLCPP_WARN(nh->get_logger(), "Cannot find value for parameter: %s, assigning default: %s", name.c_str(), oss.str().c_str());
  }
	  nh->declare_parameter(name, defaultValue);
  return defaultValue;
}

template<typename T>
T getParam(const rclcpp::Node::SharedPtr &nh, const std::string& name)
{
  T v;
  int i = 0;
  while(!nh->get_parameter(name, v))
  {
    RCLCPP_ERROR(nh->get_logger(), "Cannot find value for parameter: %s, will try again.", name.c_str());
    if ((i ++) >= 5) return T();
  }

  std::ostringstream oss;
  oss << v;
  
  RCLCPP_INFO(nh->get_logger(), "Found parameter: %s, value: %s", name.c_str(), oss.str().c_str());
  return v;
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
