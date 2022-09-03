

// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <prometheus/counter.h>
#include <prometheus/exposer.h>
#include <prometheus/registry.h>
#include <prometheus/gateway.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
     // create a metrics registry
    // @note it's the users responsibility to keep the object alive
    registry_ = std::make_shared<prometheus::Registry>();


    const auto labels = prometheus::Gateway::GetInstanceLabel("test_ros_integration_node");

   gateway_ = std::make_shared<prometheus::Gateway>("127.0.0.1", "9091", "sample_client", labels);

  // ask the pusher to push the metrics to the pushgateway
  gateway_->RegisterCollectable(registry_);
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    // I'd like to persist this but only returning the reference makes that very hard.
    // Requerying it seems to work.
    auto & counter = prometheus::BuildCounter().Name("loops_total")
                             .Help("loops packets")
                             .Register(*registry_);

    const auto random_value = std::rand();

    // add and remember dimensional data, incrementing those is very cheap
    auto& timer_counter =
        counter.Add({{"callback_type", "timer"}});
    auto& alt_counter =
        counter.Add({{"callback_type", "alt"}});


    if (random_value & 1) timer_counter.Increment();
    if (random_value & 4) alt_counter.Increment();


    // add a counter whose dimensional data is not known at compile time
    // nevertheless dimensional values should only occur in low cardinality:
    // https://prometheus.io/docs/practices/naming/#labels
    auto& http_requests_counter = prometheus::BuildCounter()
                                        .Name("http_requests_total")
                                        .Help("Number of HTTP requests")
                                        .Register(*registry_);

    const std::array<std::string, 4> methods = {"GET", "PUT", "POST", "HEAD"};
    auto method = methods.at(random_value % methods.size());
    // dynamically calling Family<T>.Add() works but is slow and should be
    // avoided
    http_requests_counter.Add({{"method", method}}).Increment();

    // push metrics
    auto returnCode = gateway_->Push();
    std::cout << "returnCode is " << returnCode << std::endl;

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::shared_ptr<prometheus::Registry> registry_;
  std::shared_ptr<prometheus::Gateway> gateway_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}