# ROS 2 API

## `husarion_ugv_crsf_interfaces`

Contains custom message definitions for the `husarion_ugv_crsf_teleop` node. Currently, it only contains the `LinkStatus` message.

## `husarion_ugv_crsf_teleop`

The main package for controlling robots using a CRSF compatible remote control. A receiver should be connected to the robot's computer via USB-UART converter or be integrated as a hardware USB dongle. The CRSF protocol parser is implemented based on the following [specification](https://github.com/crsf-wg/crsf/wiki).

### Launch Files

- `teleop.launch.py`: Launches crsf_teleop_node node. Automatically respawns node if a remote controller is connected. A namespace of a node can be set using the `namespace` launch argument. A custom parameters file can be provided by setting the `params_file` launch argument.

#### Parameters

- `namespace` [*string*, default: **`""`**]: Namespace for the node.
- `params_file` [*string*, default: **`config/crsf_teleop_${ROBOT_MODEL}.yaml`**]: Path to the `crsf_teleop_node` parameters file.

### Configuration Files

- [`crsf_teleop_panther.yaml`](./husarion_ugv_crsf_teleop/config/crsf_teleop_panther.yaml): Sets default Panther robot parameter values for the crsf_teleop_node when `teleop.launch.py` is launched.
- [`crsf_teleop_lynx.yaml`](./husarion_ugv_crsf_teleop/config/crsf_teleop_lynx.yaml): Sets default Lynx robot parameter values for the crsf_teleop_node when `teleop.launch.py` is launched.

### ROS Nodes

#### crsf_teleop_node

Translates received CRSF commands to velocity commands for the robot.

The following channels are used for controlling the robot via the TX16S remote control:

- Channel 2 - Right gimbal up/down - forward/backward velocity
- Channel 4 - Left gimbal left/right - turning (angular) velocity
- Channel 5 - SF switch - emergency stop
- Channel 7 - SA switch (down position) - silence `cmd_vel` messages, allows other nodes to control the robot while enabling e_stop functionality
- Channel 11 - SG switch - tristate switch, selects robot speed

##### Publishes

- `cmd_vel` [*geometry_msgs/Twist*]: Publishes velocity commands to the robot.
- `link_status` [*panther_crsf_teleop_msgs/LinkStatus*]: Describes a radio link status between the remote control and the robot. Parameters are described in the [CRSF_FRAMETYPE_LINK_STATISTICS frame documentation](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_LINK_STATISTICS).

##### Service Clients

- `hardware/e_stop_trigger` [*std_srvs/Trigger*]: Triggers an emergency stop.
- `hardware/e_stop_reset` [*std_srvs/Trigger*]: Resets an emergency stop.

##### Parameters

- `serial_port` [*string*, default: **/dev/ttyUSB0**]: Serial port to which the CRSF receiver is connected.
- `baudrate` [*int*, default: **576000**]: Baudrate of the serial port.
- `e_stop_republish` [*bool*, default: **False**]: Rebroadcasts asserted emergency stop signal once per second. Will override other emergency stop sources.
- `enable_cmd_vel_silence_switch`[*bool*, default: **False**]: Enables remote to disable publishing `cmd_vel` messages on demand. Can be used as a remote emergency stop when using other nodes to control the robot.
- `linear_speed_presets` [*double_array*, default: **[0.5, 1.0, 2.0]**]: Selectable robot maximum linear speed for `cmd_vel` topic.
- `angular_speed_presets` [*double_array*, default: **[0.5, 1.0, 2.0]**]: Selectable robot maximum angular speed for the `cmd_vel` topic.
