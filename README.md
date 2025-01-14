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

```
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

#### 4. (Optional) Edit Parameters File

> [!NOTE]
> When using default settings there is no need to add or edit parameters file as the model of your Husarion robot is deduced automatically from environment variables.

If you want to change the default parameters, you can do so by providing a parameters file. Detailed information about the parameters can be found in the ROS API.

Get a reference file from for Panther:

```bash
cp husarion_ugv_crsf_teleop/config/crsf_teleop_panther.yaml params.yaml
```

Or for Lynx:

```bash
cp husarion_ugv_crsf_teleop/config/crsf_teleop_lynx.yaml params.yaml
```

In the `compose.yaml` file, pass `params.yaml` under `volumes` section:

```yaml
    volumes:
      - /dev:/dev
      - ./params.yaml:/params.yaml
```

And add parameter file as launch argument:

```yaml
    command: ros2 launch husarion_ugv_crsf_teleop crsf_teleop.launch.py params_file:=/params.yaml
```

Edit entries in the `params.yaml` file to change the default parameters.

#### 5. Start Docker service

```bash
docker compose up -d
```

You can view the logs of the running container by typing:

```bash
docker compose logs -f
```

After a successful connection to the RC remote, you should see the following message:

```
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
