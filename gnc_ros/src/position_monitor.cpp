// Subscribes to the PX4 local position and logs it once per second.
// First sanity check that our package talks to PX4 through the XRCE-DDS agent.

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

using px4_msgs::msg::VehicleLocalPosition;

class PositionMonitor : public rclcpp::Node {
 public:
  PositionMonitor() : Node("position_monitor") {
    // PX4 publishes with best-effort reliability; SensorDataQoS matches it.
    // A default (reliable) subscription would silently receive nothing.
    subscription_ = create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", rclcpp::SensorDataQoS(),
        [this](const VehicleLocalPosition& msg) { on_position(msg); });
  }

 private:
  void on_position(const VehicleLocalPosition& msg) {
    // PX4 uses NED: z points down, so altitude above home is -z.
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "NED x=%.2f y=%.2f z=%.2f | altitude=%.2f m | vz=%.2f m/s", msg.x, msg.y,
                         msg.z, -msg.z, msg.vz);
  }

  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionMonitor>());
  rclcpp::shutdown();
  return 0;
}
