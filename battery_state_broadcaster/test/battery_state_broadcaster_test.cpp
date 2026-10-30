#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "battery_state_broadcaster/BatteryStateBroadcaster.hpp"

namespace
{
using Broadcaster = battery_state_broadcaster::BatteryStateBroadcaster;
using CallbackReturn = controller_interface::CallbackReturn;
using Message = sensor_msgs::msg::BatteryState;

class BatteryStateBroadcasterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // Keep test discovery separate from the robot's normal ROS domain.
    rclcpp::InitOptions options;
    options.set_domain_id(232);
    rclcpp::init(0, nullptr, options);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    broadcaster_ = std::make_unique<Broadcaster>();
    ASSERT_EQ(broadcaster_->init("test_battery_broadcaster"), controller_interface::return_type::OK);
  }

  double voltage_ = 12.0;
  double current_ = 1.0;
  hardware_interface::StateInterface voltage_interface_{"battery_state", "voltage", &voltage_};
  hardware_interface::StateInterface current_interface_{"battery_state", "current", &current_};
  std::unique_ptr<Broadcaster> broadcaster_;
};

TEST_F(BatteryStateBroadcasterTest, EmptyDefaultCurveCanBeConfigured)
{
  EXPECT_TRUE(broadcaster_->get_node()->get_parameter("charging_voltages").as_double_array().empty());
  EXPECT_TRUE(broadcaster_->get_node()->get_parameter("charging_percentages").as_double_array().empty());
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(BatteryStateBroadcasterTest, InvalidCurveReturnsConfigurationError)
{
  broadcaster_->get_node()->set_parameter(rclcpp::Parameter("charging_voltages", std::vector<double>{14.0, 14.0}));
  broadcaster_->get_node()->set_parameter(rclcpp::Parameter("charging_percentages", std::vector<double>{100.0, 0.0}));
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(BatteryStateBroadcasterTest, ConfiguredCurveIsUsedInPublishedMessage)
{
  broadcaster_->get_node()->set_parameter(rclcpp::Parameter("charging_voltages", std::vector<double>{14.0, 12.0, 10.0}));
  broadcaster_->get_node()->set_parameter(rclcpp::Parameter("charging_percentages", std::vector<double>{100.0, 50.0, 0.0}));
  ASSERT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  std::vector<hardware_interface::LoanedStateInterface> interfaces;
  interfaces.emplace_back(voltage_interface_);
  interfaces.emplace_back(current_interface_);
  broadcaster_->assign_interfaces({}, std::move(interfaces));
  ASSERT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  Message::SharedPtr received;
  auto node = std::make_shared<rclcpp::Node>("battery_broadcaster_test_subscriber");
  auto subscription = node->create_subscription<Message>(
    "/test_battery_broadcaster/battery_state", rclcpp::SystemDefaultsQoS(),
    [&received](Message::SharedPtr message) { received = message; });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    broadcaster_->update(rclcpp::Time(1, 0), rclcpp::Duration::from_seconds(0.02));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(received, nullptr);
  EXPECT_FLOAT_EQ(received->voltage, 12.0f);
  EXPECT_FLOAT_EQ(received->current, 1.0f);
  EXPECT_EQ(received->power_supply_status, Message::POWER_SUPPLY_STATUS_CHARGING);
  EXPECT_FLOAT_EQ(received->percentage, 50.0f);
  EXPECT_TRUE(std::isnan(received->charge));
  broadcaster_->on_deactivate(rclcpp_lifecycle::State());
  broadcaster_->release_interfaces();
}
}  // namespace
