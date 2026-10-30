#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace battery_state_broadcaster
{
class BatterySensor : public semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>
{
public:
  explicit BatterySensor(const std::string& name,
    const std::vector<double>& charging_voltages = {},
    const std::vector<double>& charging_percentages = {})
    : semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>(name, 2)
  {
    interface_names_.emplace_back(name_ + "/" + "voltage");
    interface_names_.emplace_back(name_ + "/" + "current");
    if (charging_voltages.size() != charging_percentages.size() ||
      (!charging_voltages.empty() && charging_voltages.size() < 2))
    {
      throw std::invalid_argument(
        "charging_voltages and charging_percentages must both be empty or have the same size (at least 2)");
    }
    charging_table_.reserve(charging_voltages.size());
    for (size_t i = 0; i < charging_voltages.size(); ++i) {
      if (!std::isfinite(charging_voltages[i]) || !std::isfinite(charging_percentages[i]) ||
        charging_percentages[i] < 0.0 || charging_percentages[i] > 100.0 ||
        (i > 0 && (charging_voltages[i] >= charging_voltages[i - 1] ||
        charging_percentages[i] > charging_percentages[i - 1])))
      {
        throw std::invalid_argument(
          "Charging curve requires finite values, strictly descending voltages, and "
          "non-increasing percentages in [0, 100]");
      }
      charging_table_.push_back({charging_voltages[i], charging_percentages[i]});
    }
  }

  virtual ~BatterySensor() = default;

  double get_voltage()
  {
    voltage_ = state_interfaces_[0].get().get_value();
    return voltage_;
  }

  double get_current()
  {
    current_ = state_interfaces_[1].get().get_value();
    return current_;
  }

  bool get_values_as_message(sensor_msgs::msg::BatteryState& message)
  {
    get_voltage();
    get_current();
    
    message.voltage = static_cast<float>(voltage_);
    message.percentage = calculate_percentage();
    message.current = static_cast<float>(current_);
    if (!std::isfinite(current_)) {
      message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    } else if (get_charging_status()) {
      message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else if (current_ < 0.0) {
      message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
      message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    
    return true;
  }

  float calculate_percentage() const
  {
    if (!std::isfinite(voltage_) || !std::isfinite(current_)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    const auto& table = get_charging_status() ? charging_table_ : discharging_table_;
    if (table.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    if (voltage_ >= table.front().voltage) {
      return static_cast<float>(table.front().percentage);
    }
    if (voltage_ <= table.back().voltage) {
      return static_cast<float>(table.back().percentage);
    }

    for (size_t i = 0; i + 1 < table.size(); ++i) {
      if (voltage_ <= table[i].voltage && voltage_ >= table[i + 1].voltage) {
        const auto& high = table[i];
        const auto& low = table[i + 1];
        return static_cast<float>(low.percentage +
          (high.percentage - low.percentage) *
          (voltage_ - low.voltage) / (high.voltage - low.voltage));
      }
    }

    return std::numeric_limits<float>::quiet_NaN();
  }

  bool get_charging_status() const
  {
    return std::isfinite(current_) && current_ > 0.0;
  }

private:
  double voltage_ = 0.0;
  double current_ = 0.0;

  struct VoltageMap {
    double voltage;
    double percentage;
  };

  std::vector<VoltageMap> charging_table_;
  
  // Existing discharge calibration, kept separate from the charging curve.
  const std::vector<VoltageMap> discharging_table_ = {
      {13.06, 100.0},
      {12.2, 90.0},
      {12.1, 70.0},
      {12.0, 40.0},
      {11.8, 20.0},
      {11.2, 10.0},
      {10.7, 0.0}
  };
};
}  // namespace battery_state_broadcaster
