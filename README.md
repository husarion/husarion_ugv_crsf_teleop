# husarion_ugv_crsf_teleop

## Getting started

To run the RadioMaster TX16S, you must meet certain requirements listed below:

### Software

- [**Docker Engine and Docker Compose**](https://docs.docker.com/engine/install/).
- Device with Linux based OS (e.g. robot, laptop).

### Hardware

- Hardware receiver dongle (ELRS receiver with an integrated USB to UART converter) connected to a USB port of the User Computer.

## Demo

[husarion_ugv_crsf_teleop](https://github.com/husarion/husarion_ugv_crsf_teleop) package creates a ROS 2 node, allowing control of the ROS-powered mobile and manipulation robots with a CRSF compatible RC remote such as RadioMaster TX16S. `crsf_teleop` node converts received CRSF packets from the receiver dongle to `geometry_msgs/Twist` message type in order to provide velocity commands for the robot.

The result of this demo will be the publication of a velocity messages on the ROS topic `/cmd_vel`. An example with a mobile robot will be described below.

### Start guide

#### 1. Setup udev rules

> [!WARNING] Husarion OS
> If you are using Husarion OS, you can skip this step as the udev rules are already set up.

To be able to distinguish between connected devices it is necessary to add a rule that aliases detected hardware dongle to a known symlink file.

Open (or create) the udev rules file:

```bash
sudo nano /etc/udev/rules.d/99-local.rules
```

Append the following line to the file:

```bash
ACTION=="add", ,ATTRS{interface}=="PAD02 Dongle", SYMLINK+="ttyUSBPAD"
```

Press <kbd>ctrl</kbd> + <kbd>o</kbd> and then <kbd>ctrl</kbd> + <kbd>x</kbd> to save and quit.

Reload rules by executing:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 2. Test receiver dongle

Connect receiver dongle to the USB port of the User Computer. You can verify that the device is detected by typing:

```bash
ls /dev | grep "ttyUSBPAD"
```

You should see a file named `ttyUSBPAD` in the output.

#### 3. Clone repository

```bash
git clone https://github.com/husarion/husarion_ugv_crsf_teleop.git
cd husarion_ugv_crsf_teleop
```

#### 4. Edit Parameters File

By default the node is set up to work with Panther robot. To load Lynx configuration, you need to change the `params_file` parameter in the `docker-compose.yml` file.

```yaml
command: >
      ros2 launch husarion_ugv_crsf_teleop teleop.launch.py params_file:=/config/crsf_teleop_lynx.yaml
```

Default parameter files for Panther and Lynx can be found in the `husarion_ugv_crsf_teleop/config` directory and can be changed directly.
Custom parameters file can be provided by setting the `params_file` launch argument.

#### 5. Start Docker service

```bash
docker compose up -d
```

You can view the logs of the running container by typing:

```bash
docker compose logs -f
```

After a successful connection to the RC remote, you should see the following message:

```bash
[panther.crsf_teleop]: Connected
[panther.crsf_teleop]: Link quality restored: 100%
```

### Result

Exec into running container:

```bash
docker compose exec -it husarion_ugv_crsf_teleop /bin/bash
```

Now you should see the relevant ROS 2 topic:

```bash
user@docker:~$ ros2 topic list
/panther/cmd_vel
/panther/link_status
/parameter_events
/rosout
```

Tilting sticks on the RC controller should result in `/cmd_vel` topic being updated with velocity commands. Namespace fields should be updated with used robot model name (e.g. `panther`).

```bash
ros2 topic echo /<namespace>/cmd_vel
```

Additionally, you can check the link status by echoing the `/link_status` topic:

```bash
ros2 topic echo /<namespace>/link_status
```

Sticks are mapped to the following axes:

- Left stick horizontal: angular velocity
- Right stick vertical: linear velocity

RC controller has some additional buttons that can be used for different functionalities:

- SG switch - tri-state switch, selects robot speed from values specified in the parameters yaml file (slow, normal, fast)
- SF switch - emergency stop
- SA switch (down position) - silence `cmd_vel` messages, allows other nodes to control the robot while enabling e_stop functionality

> [!WARNING] SF switch
> By default the SA switch has no functionality assigned. You can enable silence mode by adding the following line to the custom `params.yaml` file:
>
> ```yaml
> silence_cmd_vel: true
> ```

## ROS node API

### Publishes

- `cmd_vel` [*geometry_msgs/Twist* or *geometry_msgs/TwistStamped*]: Command velocity, the type of the message depends on `cmd_vel_stamped` parameter.
- `link_status` [*husarion_ugv_crsf_interfaces/LinkStatus*]: CRSF link status.

### Service clients

- `hardware/e_stop_trigger` [*std_srvs/Trigger]: Trigger robot emergency stop.
- `hardware/e_stop_release` [*std_srvs/Trigger]: Release robot emergency stop.

### Parameters

- `port` [*string*, default: **/dev/ttyUSB0**]: CRSF receiver serial port".
- `baud` [*int*, default: **576000**]: CRSF receiver baud rate".
- `cmd_vel_stamped` [*bool*, default: **False**]: Publish cmd_vel as TwistStamped instead of Twist.
- `e_stop_republish` [*bool*, default: **False**]:  Rebroadcast asserted e-stop signal once per second
- `enable_cmd_vel_silence_switch` [*bool*, default: **False**]: Enable cmd_vel silence switch allowing other nodes to take control.
- `linear_speed_presets` [*list[float]*, default: **[0.5, 1.0, 2.0]**]: Selectable robot maximum linear speed for cmd_vel in value range from 0.0 to 10.0.
- `angular_speed_presets` [*list[float]*, default: **[0.5, 1.0, 2.0]**]: Selectable robot maximum angular speed for cmd_vel in value range from 0.0 to 10.0.
