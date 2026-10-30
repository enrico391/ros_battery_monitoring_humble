# battery_state_broadcaster
This is a [ros2_control Broadcaster](https://control.ros.org/foxy/doc/ros2_controllers/doc/controllers_index.html#available-broadcasters) which publishes [sensor_msgs/BatteryState](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/BatteryState.msg) messages from appropriage state interfaces.

## Required State Interfaces
This broadcaster requires the robot to have a sensor component which contains the battery state interfaces:

```xml
<ros2_control type="system">

    <!-- ... -->

    <sensor name="battery_state">
        <state_interface name="voltage" />
        <state_interface name="current" />
    </sensor>
</ros2_control>
```

## Parameters
To use this broadcaster, declare it in the controller manager and set its parameters:

```yaml
controller_manager:
  ros__parameters:
    battery_state_broadcaster:
      type: battery_state_broadcaster/BatteryStateBroadcaster

battery_state_broadcaster:
  ros__parameters:
    sensor_name: "battery_state"
    design_capacity: 100.0
    # https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/BatteryState.msg
    power_supply_technology: 2
```

And spawn it in the launch file:

```python
battery_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["battery_state_broadcaster"],
)
```

## Topics
The battery state is published on `~/battery_state`.
Since it's a plugin within the controller manager, add a remapping of the form `("~/battery_state", "/my_battery_state") to the *controller manager*, not the spawner, to change the topic name.

## Charging status and percentage

The `current` state interface must use amperes, positive while charging and negative
while discharging. `power_supply_status` is updated on every published message:

| Current | Status |
| --- | --- |
| Positive | `POWER_SUPPLY_STATUS_CHARGING` (1) |
| Negative | `POWER_SUPPLY_STATUS_DISCHARGING` (2) |
| Zero | `POWER_SUPPLY_STATUS_NOT_CHARGING` (3) |
| NaN or infinity | `POWER_SUPPLY_STATUS_UNKNOWN` (0) |

`charge` remains NaN: that field represents remaining amp-hours, not a charging
indicator. See the [ROS BatteryState definition](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/BatteryState.msg).

During discharge or at zero current, percentage uses the existing discharge curve:
12.3 V = 100%, 12.2 V = 90%, 12.1 V = 70%, 12.0 V = 40%, 11.8 V = 20%,
11.2 V = 10%, and 10.7 V = 0%.

During charging, percentage uses the optional `charging_voltages` and
`charging_percentages` parameters. Both default to empty arrays; until a calibrated
charging curve is supplied, percentage is NaN while charging. Invalid voltage or
current measurements also produce NaN percentage.

Add your calibration to the broadcaster's `ros__parameters` block in the controller
YAML. This example demonstrates the format only; **these are synthetic test values,
not a calibration for your battery**:

```yaml
battery_state_broadcaster:
  ros__parameters:
    sensor_name: "battery_state"
    charging_voltages: [14.0, 12.0, 10.0]
    charging_percentages: [100.0, 50.0, 0.0]
```

Each voltage pairs with the percentage at the same index. Supply at least two
points, with finite, strictly descending voltages and finite, non-increasing
percentages in the range 0–100. Unequal lengths or invalid values cause controller
configuration to fail with an error. Values between points are linearly
interpolated; values outside the table clamp to the nearest endpoint's percentage.

For compatibility with existing consumers, this broadcaster continues to publish
percentage on a **0–100** scale, although ROS BatteryState specifies 0–1. Calibration
percentages also use 0–100.

Parameters are read when the controller is configured. After editing the YAML,
reload the controller with the updated parameters (or restart the robot bringup).
