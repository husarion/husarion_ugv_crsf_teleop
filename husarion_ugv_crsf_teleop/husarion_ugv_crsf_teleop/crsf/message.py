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

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List

CRSF_SYNC = 0xC8
CRSF_SYNC_EDGETX = 0xEE
CRSF_MSG_EXTENDED = 0x28
CRSF_RC_CHANNELS_PACKED_LEN = 22
CRSF_RC_CHANNELS_LEN = 16


class PacketType(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALTITUDE = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    RC_CHANNELS_SUBSET = 0x17
    LINK_RX_ID = 0x1C
    LINK_TX_ID = 0x1D
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_PING = 0x28
    DEVICE_INFO = 0x29
    PARAM_ENTRY = 0x2B
    PARAM_READ = 0x2C
    PARAM_WRITE = 0x2D
    ELRS_STATUS = 0x2E
    COMMAND = 0x32
    RADIO_ID = 0x3A
    KISS_REQUEST = 0x78
    KISS_RESPONSE = 0x79
    MSP_REQUEST = 0x7A
    MSP_RESPONSE = 0x7B
    MSP_WRITE = 0x7C
    DISPLAYPORT_CMD = 0x7D
    CUSTOM_TELEMETRY = 0x80


@dataclass
class CRSFMessage:
    msg_type: PacketType = 0
    payload: bytearray = field(default_factory=bytearray)
    crc: int = 0

    destination: int = 0
    source: int = 0

    def is_extended(self) -> bool:
        if self.msg_type not in PacketType:
            raise ValueError("Invalid message type")

        # All extended messages begin from the 0x28 address
        if self.msg_type >= CRSF_MSG_EXTENDED:
            return True

    def calculate_crc(self, assign_to_self: bool = True) -> int:
        crc = self._crc8_dvb_s2(0, self.msg_type)

        if self.is_extended():
            crc = self._crc8_dvb_s2(crc, self.destination)
            crc = self._crc8_dvb_s2(crc, self.source)

        for byte in self.payload:
            crc = self._crc8_dvb_s2(crc, byte)

        if assign_to_self:
            self.crc = crc

        return crc

    def encode(self) -> bytearray:
        data = bytearray()

        if self.msg_type not in PacketType:
            raise ValueError("Invalid message type")

        data.append(CRSF_SYNC)
        data.append(len(self.payload))
        data.append(self.msg_type)

        if self.is_extended():
            data.append(self.destination)
            data.append(self.source)

        data.extend(self.payload)
        data.append(self.calculate_crc())

        return data

    def _crc8_dvb_s2(self, crc, a) -> int:
        crc = crc ^ a
        for ii in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc = crc << 1
        return crc & 0xFF


def unpack_channels(packed: bytearray) -> List[int]:
    if len(packed) != CRSF_RC_CHANNELS_PACKED_LEN:
        raise ValueError("Input data must be 22 bytes long")

    channels = []
    bit_buffer = int.from_bytes(packed, byteorder="little")

    for _ in range(CRSF_RC_CHANNELS_LEN):
        channels.append(bit_buffer & 0x7FF)
        bit_buffer >>= 11

    return channels


# Normalize channel values from CRSF range [172, 1812] to [-1, 1]
def normalize_channel_values(channels: List[int]) -> List[float]:
    if len(channels) != CRSF_RC_CHANNELS_LEN:
        raise ValueError("Input data must contain 16 channels")

    return [(channel - 992) / 820.0 for channel in channels]
