# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from husarion_ugv_crsf_interfaces.msg import LinkStatus

import serial
from .crsf.parser import CRSFParser, unpack_channels
from .crsf.message import CRSFMessage, PacketType


class CRSFInterface(Node):
    def __init__(self):
        super().__init__('crsf_interface')
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1
            )
        )
        
        self.link_status_publisher = self.create_publisher(
            LinkStatus,
            'link_status',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1
            )
        )
        
        self.e_stop_trigger = self.create_client(
            Trigger,
            'hardware/e_stop_trigger',
        )
        
        self.e_stop_reset = self.create_client(
            Trigger,
            'hardware/e_stop_reset',
        )   
        
        self.link_status = LinkStatus()
        
        self.declare_parameter('port', '/dev/ttyUSB0', ParameterDescriptor(description='CRSF receiver serial port'))
        self.declare_parameter('baud', 576000, ParameterDescriptor(description='CRSF receiver baud rate'))
        self.declare_parameter('e_stop_republish', False,
                               ParameterDescriptor(description='Rebroadcast asserted e-stop signal once per second'))
        self.declare_parameter('enable_cmd_vel_silence_switch', False,
                               ParameterDescriptor(description='Enable cmd_vel silence switch allowing other nodes to take control'))
        self.declare_parameter('linear_speed_presets', [0.5, 1.0, 2.0], ParameterDescriptor(description='Selectable robot maximum linear speed for cmd_vel', floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0)]))
        self.declare_parameter('angular_speed_presets', [0.5, 1.0, 2.0], ParameterDescriptor(description='Selectable robot maximum angular speed for cmd_vel', floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0)]))
        self.linear_speed_presets = self.get_parameter('linear_speed_presets').get_parameter_value()._double_array_value
        self.angular_speed_presets = self.get_parameter('angular_speed_presets').get_parameter_value()._double_array_value
        
        if len(self.linear_speed_presets) != 3 or len(self.angular_speed_presets) != 3:
            raise ValueError("Speed presets must be a list of 3 values")
        
        # Initialize serial port
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        
        self.serial = serial.Serial(port, baud, timeout=2)
        
        # Setup CRSF parser
        self.parser = CRSFParser()
        self.parser.on_message = lambda msg: self.handle_message(msg)
        
        self.timer = self.create_timer(0.01, self.run)
        
        self.rc_estop_state = True
        if self.get_parameter('e_stop_republish').get_parameter_value().bool_value:
            self.e_stop_republisher = self.create_timer(1, self.update_e_stop)
        
    def run(self):
        if self.serial.in_waiting > 0:
            self.parser.parse(self.serial.read(self.serial.in_waiting))
                
    def handle_message(self, msg: CRSFMessage):
        if msg.msg_type == PacketType.RC_CHANNELS_PACKED:
            channels = unpack_channels(msg.payload)
            
            # Map channels from CRSF range [172, 1812] to [-1, 1]
            channels = [(channel - 992)/ 820.0 for channel in channels]
            
            # Handle emergency stop from RC controller
            # Asserted e-stop is retransmitted once per second by republish timer
            # if enabled by the 'e_stop_republish' parameter
            # Deasserted e-stop is transmitted only once
            requested_e_stop = channels[4] < 0.5
            if requested_e_stop != self.rc_estop_state:
                self.rc_estop_state = requested_e_stop
                
                if self.rc_estop_state:
                    self.e_stop_trigger.call_async(Trigger.Request())
                else:
                    self.e_stop_reset.call_async(Trigger.Request())
            
            # Disable sending cmd_vel if override switch is asserted
            send_cmd_vel = channels[6] < -0.5
            silence_cmd_vel = self.get_parameter('enable_cmd_vel_silence_switch').get_parameter_value().bool_value
            
            if send_cmd_vel and not silence_cmd_vel:
                linear_speed_modifier = self.linear_speed_presets[round(channels[10]) + 1]
                angular_speed_modifier = self.angular_speed_presets[round(channels[10]) + 1]
                
                t = Twist()
                t.linear.x = channels[1] * linear_speed_modifier
                t.angular.z = -channels[3] * angular_speed_modifier
                
                self.cmd_vel_publisher.publish(t)
        
        elif msg.msg_type == PacketType.LINK_STATISTICS:
            last_lq = self.link_status.lq
            
            self.link_status.rssi_1 = -msg.payload[0]
            self.link_status.rssi_2 = -msg.payload[1]
            self.link_status.lq = msg.payload[2]
            self.link_status.uplink_snr = int.from_bytes(bytes(msg.payload[3]), byteorder='big', signed=True)
            self.link_status.used_antenna = msg.payload[4]
            self.link_status.mode = msg.payload[5]
            self.link_status.tx_power = msg.payload[6]
            self.link_status.downlink_rssi = -msg.payload[7]
            self.link_status.downlink_lq = msg.payload[8]
            self.link_status.downlink_snr = int.from_bytes(bytes(msg.payload[9]), byteorder='big', signed=True)
            
            # Detect connection status
            if last_lq != 0 and self.link_status.lq == 0:
                self.get_logger().error("Connection lost")
                
                # Publish empty cmd_vel to stop the robot
                self.cmd_vel_publisher.publish(Twist())
                
            if last_lq == 0 and self.link_status.lq > 0:
                self.get_logger().info("Connected")
            
            # Warn on low link quality
            if last_lq != self.link_status.lq:
                if self.link_status.lq < 15:
                    self.get_logger().warn(f"Very low link quality: {self.link_status.lq}%")
                elif self.link_status.lq < 30:
                    self.get_logger().warn(f"Low link quality: {self.link_status.lq}%")
                elif last_lq < 30 and self.link_status.lq >= 30:
                    self.get_logger().info(f"Link quality restored: {self.link_status.lq}%")
            
            self.link_status_publisher.publish(self.link_status)
            
        else:
            self.get_logger().warn(f"Unknown CRSF message (Type: {msg.msg_type.name}, Length: {msg.length})")
            
    def update_e_stop(self):
        if self.rc_estop_state:
            self.e_stop_trigger.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)

    crsf_interface = CRSFInterface()
    
    rclpy.spin(crsf_interface)
    
    crsf_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()