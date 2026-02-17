
#include <limits>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/detail/battery_state__struct.hpp>

namespace battery_state_broadcaster
{
class BatterySensor : public semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>
{
public:
  explicit BatterySensor(const std::string& name)
    : semantic_components::SemanticComponentInterface<sensor_msgs::msg::BatteryState>(name, 1)
  {
    interface_names_.emplace_back(name_ + "/" + "voltage");
    interface_names_.emplace_back(name_ + "/" + "current");
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
    message.power_supply_status = get_charging_status() ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                                                  : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    
    return true;
  }

  float calculate_percentage()
  {
    if (voltage_ >= batteryTable.front().voltage) {
      return 100.0f;
    }
    if (voltage_ <= batteryTable.back().voltage) {
      return 0.0f;
    }

    for (size_t i = 0; i < batteryTable.size() -1 ; ++i) {
      if (voltage_ <= batteryTable[i].voltage && voltage_ >= batteryTable[i + 1].voltage) {
        float vHigh = batteryTable[i].voltage;
        float pHigh = batteryTable[i].percentage;
        float vLow = batteryTable[i + 1].voltage;
        float pLow = batteryTable[i + 1].percentage;

        // Linear interpolation
        return pLow + (pHigh - pLow) * (voltage_ - vLow) / (vHigh - vLow);
      }
    }

    return 0.0f; // Default return value if no match found
  }

  bool get_charging_status()
  {
    for (const auto& entry : batteryTable) {
      if (voltage_ >= entry.voltage) {
        return entry.in_charging_range;
      }
    }
    return false; // Default to not charging if voltage is below the lowest entry
  }

  

private:
  double voltage_ = 0.0;
  double current_ = 0.0;
  double percentage_ = 0.0;

  struct VoltageMap {
    float voltage;
    float percentage;
    bool in_charging_range = false;
  };
  
  // Based on the 20°C - 30°C curve from your Haisic manual
  // at 13.85 the battery is fully charged, and at 10.7 it's empty. 
  // above 13.00 V the battery is charging
  const std::vector<VoltageMap> batteryTable = {
      {13.75f, 100.0f, true}, // Fully charged
      {13.00f, 90.0f, true},  // Charging range starts
      {12.3f, 100.0f, false},
      {12.2f, 90.0f, false},
      {12.1f, 70.0f, false}, // The start of the flat plateau
      {12.0f, 40.0f, false}, // Mid-plateau
      {11.8f, 20.0f, false}, // The "knee" where voltage drops fast
      {11.2f, 10.0f, false}, // Critical level
      {10.7f, 0.0f, false}   // Empty
  };
};
}  // namespace battery_state_broadcaster
