#include "kunwei_ft_driver/client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <thread>
#include <iostream>
#include <algorithm>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code=0;
  try {
    auto node = std::make_shared<rclcpp::Node>("kunwei_ft_driver");
    const auto host=node->declare_parameter<std::string>("host", "192.168.1.88");
    const auto port=node->declare_parameter<int>("port",5152);
    const auto frame=node->declare_parameter<std::string>("frame_id","ft_sensor");
    const auto scale=node->declare_parameter<double>("unit_scale",9.80665);
    if (port<1 || port>65535) throw std::invalid_argument("invalid TCP port");
    kunwei_ft_driver::Client client(scale);
    auto pub=node->create_publisher<geometry_msgs::msg::WrenchStamped>("ft_sensor/wrench",rclcpp::SensorDataQoS());
    while(rclcpp::ok()) {
      try {
        client.connect(host,static_cast<std::uint16_t>(port));
        RCLCPP_INFO(node->get_logger(),"Connected to %s:%d",host.c_str(),static_cast<int>(port));
        auto last=std::chrono::steady_clock::now();
        const auto started=last;
        std::uint64_t total=0, max_batch=0;
        while(rclcpp::ok()) {
          rclcpp::spin_some(node);
          std::uint64_t batch=0;
          client.read([&](const kunwei_ft_driver::Sample& sample){
            ++batch; ++total;
            last=sample.received_at;
            geometry_msgs::msg::WrenchStamped msg;
            msg.header.stamp=node->now(); msg.header.frame_id=frame;
            msg.wrench.force.x=sample.wrench[0]; msg.wrench.force.y=sample.wrench[1];
            msg.wrench.force.z=sample.wrench[2]; msg.wrench.torque.x=sample.wrench[3];
            msg.wrench.torque.y=sample.wrench[4]; msg.wrench.torque.z=sample.wrench[5];
            pub->publish(msg);
          });
          max_batch=std::max(max_batch,batch);
          if(std::chrono::steady_clock::now()-last>std::chrono::seconds(3))
            throw std::runtime_error("no valid frames for 3 seconds");
        }
        const auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        RCLCPP_INFO(node->get_logger(),"Published %llu frames in %.3f s (%.2f Hz); max receive batch %llu",
                    static_cast<unsigned long long>(total),elapsed,total/elapsed,
                    static_cast<unsigned long long>(max_batch));
      } catch(const std::exception& error) {
        client.close();
        if(rclcpp::ok()) {
          RCLCPP_WARN(node->get_logger(),"%s; retrying in 2 seconds",error.what());
          const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
          while(rclcpp::ok() && std::chrono::steady_clock::now()<deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
      client.close();
    }
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n'; exit_code=1;
  }
  rclcpp::shutdown();
  return exit_code;
}
