#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include "battery_state_broadcaster/BatterySensor.hpp"

namespace
{
using Message = sensor_msgs::msg::BatteryState;

class BatterySensorTest : public ::testing::Test
{
protected:
  BatterySensorTest()
  {
    interfaces_.emplace_back(voltage_interface_);
    interfaces_.emplace_back(current_interface_);
    sensor_.assign_loaned_state_interfaces(interfaces_);
  }

  Message read()
  {
    return read(sensor_);
  }

  Message read(battery_state_broadcaster::BatterySensor& sensor)
  {
    Message message;
    message.charge = std::numeric_limits<float>::quiet_NaN();
    sensor.get_values_as_message(message);
    return message;
  }

  double voltage_ = 11.8;
  double current_ = 1.0;
  hardware_interface::StateInterface voltage_interface_{"battery", "voltage", &voltage_};
  hardware_interface::StateInterface current_interface_{"battery", "current", &current_};
  std::vector<hardware_interface::LoanedStateInterface> interfaces_;
  battery_state_broadcaster::BatterySensor sensor_{"battery"};
};

TEST_F(BatterySensorTest, PositiveCurrentMeansChargingEvenAtLowVoltage)
{
  const auto message = read();
  EXPECT_EQ(message.power_supply_status, Message::POWER_SUPPLY_STATUS_CHARGING);
  EXPECT_FLOAT_EQ(message.current, 1.0f);
  EXPECT_TRUE(std::isnan(message.charge));
}

TEST_F(BatterySensorTest, NegativeCurrentMeansDischargingEvenAtHighVoltage)
{
  voltage_ = 13.75;
  current_ = -1.0;
  EXPECT_EQ(read().power_supply_status, Message::POWER_SUPPLY_STATUS_DISCHARGING);
}

TEST_F(BatterySensorTest, ZeroCurrentMeansNotCharging)
{
  current_ = 0.0;
  EXPECT_EQ(read().power_supply_status, Message::POWER_SUPPLY_STATUS_NOT_CHARGING);
}

TEST_F(BatterySensorTest, InvalidCurrentMeansUnknown)
{
  for (const auto current : {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
  {
    current_ = current;
    EXPECT_EQ(read().power_supply_status, Message::POWER_SUPPLY_STATUS_UNKNOWN);
  }
}

TEST_F(BatterySensorTest, MissingChargingCalibrationMeansUnknownPercentage)
{
  EXPECT_TRUE(std::isnan(read().percentage));
}

TEST_F(BatterySensorTest, DischargeCurveInterpolatesAndClamps)
{
  current_ = -1.0;
  const std::vector<std::pair<double, float>> samples = {
    {13.75, 100.0f}, {12.65, 100.0f}, {12.3, 100.0f}, {12.15, 80.0f},
    {12.0, 40.0f}, {10.95, 5.0f}, {10.7, 0.0f}, {9.0, 0.0f}};
  for (const auto& sample : samples) {
    voltage_ = sample.first;
    EXPECT_NEAR(read().percentage, sample.second, 0.001f) << "Voltage: " << voltage_;
  }
}

TEST_F(BatterySensorTest, InvalidMeasurementsMeanUnknownPercentage)
{
  current_ = -1.0;
  for (const auto voltage : {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
  {
    voltage_ = voltage;
    EXPECT_TRUE(std::isnan(read().percentage));
  }
  voltage_ = 12.0;
  current_ = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::isnan(read().percentage));
}

TEST_F(BatterySensorTest, ChargingCurveInterpolatesAndClamps)
{
  // Synthetic curve for testing only, not a calibration for the robot.
  battery_state_broadcaster::BatterySensor sensor("battery", {14.0, 12.0, 10.0}, {100.0, 50.0, 0.0});
  sensor.assign_loaned_state_interfaces(interfaces_);
  const std::vector<std::pair<double, float>> samples = {
    {15.0, 100.0f}, {14.0, 100.0f}, {13.0, 75.0f}, {12.0, 50.0f},
    {11.0, 25.0f}, {10.0, 0.0f}, {9.0, 0.0f}};
  for (const auto& sample : samples) {
    voltage_ = sample.first;
    EXPECT_FLOAT_EQ(read(sensor).percentage, sample.second) << "Voltage: " << voltage_;
  }
}

TEST_F(BatterySensorTest, CurrentChangesSelectTheCurveOnEveryUpdate)
{
  battery_state_broadcaster::BatterySensor sensor("battery", {14.0, 12.0, 10.0}, {100.0, 50.0, 0.0});
  sensor.assign_loaned_state_interfaces(interfaces_);
  voltage_ = 12.0;
  EXPECT_FLOAT_EQ(read(sensor).percentage, 50.0f);
  current_ = -1.0;
  EXPECT_FLOAT_EQ(read(sensor).percentage, 40.0f);
  current_ = 0.0;
  EXPECT_FLOAT_EQ(read(sensor).percentage, 40.0f);
  current_ = 1.0;
  EXPECT_FLOAT_EQ(read(sensor).percentage, 50.0f);
}

TEST_F(BatterySensorTest, PartialChargingCurveClampsToItsEndpointPercentages)
{
  battery_state_broadcaster::BatterySensor sensor("battery", {13.0, 12.0}, {80.0, 20.0});
  sensor.assign_loaned_state_interfaces(interfaces_);
  voltage_ = 14.0;
  EXPECT_FLOAT_EQ(read(sensor).percentage, 80.0f);
  voltage_ = 11.0;
  EXPECT_FLOAT_EQ(read(sensor).percentage, 20.0f);
}

struct InvalidCurve
{
  std::vector<double> voltages;
  std::vector<double> percentages;
};

class InvalidChargingCurveTest : public ::testing::TestWithParam<InvalidCurve> {};

TEST_P(InvalidChargingCurveTest, RejectsInvalidCalibration)
{
  const auto& curve = GetParam();
  EXPECT_THROW(
    battery_state_broadcaster::BatterySensor("battery", curve.voltages, curve.percentages),
    std::invalid_argument);
}

INSTANTIATE_TEST_SUITE_P(CalibrationValidation, InvalidChargingCurveTest, ::testing::Values(
  InvalidCurve{{14.0, 12.0}, {}},
  InvalidCurve{{}, {100.0, 0.0}},
  InvalidCurve{{14.0}, {100.0}},
  InvalidCurve{{14.0, 12.0}, {100.0}},
  InvalidCurve{{12.0, 14.0}, {100.0, 0.0}},
  InvalidCurve{{14.0, 14.0}, {100.0, 0.0}},
  InvalidCurve{{14.0, 12.0}, {0.0, 100.0}},
  InvalidCurve{{14.0, 12.0}, {101.0, 0.0}},
  InvalidCurve{{14.0, 12.0}, {100.0, -1.0}},
  InvalidCurve{{std::numeric_limits<double>::quiet_NaN(), 12.0}, {100.0, 0.0}},
  InvalidCurve{{std::numeric_limits<double>::infinity(), 12.0}, {100.0, 0.0}},
  InvalidCurve{{14.0, 12.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}},
  InvalidCurve{{14.0, 12.0}, {100.0, -std::numeric_limits<double>::infinity()}}
));
}  // namespace
