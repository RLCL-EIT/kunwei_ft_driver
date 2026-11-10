#include "kunwei_ft_driver/client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <regex>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code=0;
  try {
    auto node = std::make_shared<rclcpp::Node>("kunwei_ft_driver");
    const auto host=node->declare_parameter<std::string>("host", "192.168.1.88");
    const auto port=node->declare_parameter<int>("port",5152);
    const auto prefix=node->declare_parameter<std::string>("sensor_prefix","ft_sensor");
    if (!std::regex_match(prefix, std::regex("[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*")))
      throw std::invalid_argument("sensor_prefix must be a nonempty relative ROS name without leading/trailing slashes");
    const auto frame=node->declare_parameter<std::string>("frame_id",prefix);
    const auto scale=node->declare_parameter<double>("unit_scale",9.80665);
    const auto publish_visualization=node->declare_parameter<bool>("publish_visualization",false);
    const auto visualization_rate=node->declare_parameter<double>("visualization_rate",30.0);
    if (port<1 || port>65535) throw std::invalid_argument("invalid TCP port");
    if (publish_visualization && (!std::isfinite(visualization_rate) || visualization_rate<=0 || visualization_rate>1000))
      throw std::invalid_argument("visualization_rate must be in (0, 1000] Hz");
    kunwei_ft_driver::Client client(scale);
    auto pub=node->create_publisher<geometry_msgs::msg::WrenchStamped>(prefix+"/wrench",rclcpp::SensorDataQoS());
    // RViz creates wrench visuals for every received message. Keep acquisition
    // full-rate and send only the latest sample per display update to RViz.
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr visualization_pub;
    geometry_msgs::msg::WrenchStamped visualization_msg;
    bool visualization_pending=false;
    rclcpp::TimerBase::SharedPtr visualization_timer;
    if (publish_visualization) {
      visualization_pub=node->create_publisher<geometry_msgs::msg::WrenchStamped>(
          prefix+"/wrench_visualization",rclcpp::SensorDataQoS().keep_last(1));
      visualization_timer=node->create_wall_timer(
        std::chrono::duration<double>(1.0/visualization_rate), [&]() {
          if (visualization_pending) {
            visualization_pub->publish(visualization_msg);
            visualization_pending=false;
          }
        });
    }
    std::array<double, 6> zero_offset{};
    kunwei_ft_driver::Sample latest;
    bool have_sample=false;
    // Service callbacks and acquisition run on this thread, so the offset is
    // replaced atomically with respect to publishing all six components.
    auto zero_service=node->create_service<std_srvs::srv::Trigger>(
        prefix+"/zero",
        [&](const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response) {
          if (!have_sample || std::chrono::steady_clock::now()-latest.received_at>
                                  std::chrono::milliseconds(500)) {
            response->success=false;
            response->message="No fresh sensor sample available; software zero unchanged.";
            return;
          }
          // Always capture the uncorrected reading, including on repeated calls.
          zero_offset=latest.wrench;
          response->success=true;
          response->message="Software zero set from the latest sample for all six axes. "
                            "Offset lasts until publisher restart; sensor firmware unchanged.";
          RCLCPP_INFO(node->get_logger(), "Software force/torque zero updated");
        });
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
            latest=sample;
            have_sample=true;
            auto wrench=sample.wrench;
            for (std::size_t i=0; i<wrench.size(); ++i) wrench[i]-=zero_offset[i];
            geometry_msgs::msg::WrenchStamped msg;
            msg.header.stamp=node->now(); msg.header.frame_id=frame;
            msg.wrench.force.x=wrench[0]; msg.wrench.force.y=wrench[1];
            msg.wrench.force.z=wrench[2]; msg.wrench.torque.x=wrench[3];
            msg.wrench.torque.y=wrench[4]; msg.wrench.torque.z=wrench[5];
            pub->publish(msg);
            if (visualization_pub) {
              visualization_msg=msg;
              visualization_pending=true;
            }
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
        have_sample=false;
        visualization_pending=false;
        client.close();
        if(rclcpp::ok()) {
          RCLCPP_WARN(node->get_logger(),"%s; retrying in 2 seconds",error.what());
          const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
          while(rclcpp::ok() && std::chrono::steady_clock::now()<deadline) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
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
