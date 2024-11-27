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
#
# Based on https://github.com/crsf-wg/crsf/wiki/Python-Parser by crsf-wg

from enum import Enum
from typing import Callable, Tuple

from .message import CRSF_SYNC, CRSF_SYNC_EDGETX, CRSFMessage, PacketType


class CRSFParser:
    class State(Enum):
        SEEK_SYNC = (0,)
        MSG_LEN = (1,)
        MSG_TYPE = (2,)
        MSG_EXT_DEST = (3,)
        MSG_EXT_SRC = (4,)
        MSG_PAYLOAD = (5,)
        MSG_CRC = 6

    class Result(Enum):
        PACKET_VALID = (0,)
        IN_PROGRESS = (1,)
        PACKET_INVALID = 2

    _msg: CRSFMessage
    _buffer: bytearray = bytearray()
    state: State = State.SEEK_SYNC
    on_message: Callable[[CRSFMessage], None]

    def __init__(self):
        pass

    def parse(self, data: bytearray):
        self._buffer.extend(data)

        i = 0
        while len(self._buffer) - i > 0:
            byte = self._buffer[i]
            (result, length) = self.parse_byte(byte)

            if result == self.Result.PACKET_VALID:
                # Process message
                if self.on_message:
                    self.on_message(self._msg)

                # Remove raw message from the buffer
                self._buffer = self._buffer[length:]
                i = 0

            elif result == self.Result.PACKET_INVALID:
                # Remove invalid message/byte from the buffer
                self._buffer = self._buffer[length:]
                i = 0

            elif result == self.Result.IN_PROGRESS:
                i += length

    # Parses a single byte of a CRSF message
    # Returns a tuple of the result and the number of bytes consumed
    #
    # To allow iterative parsing input data buffer first element/s should
    # only be discarded if the result is PACKET_VALID or PACKET_INVALID.
    # IN_PROGRESS indicates that the byte was read and parsed but the
    # message parsing could fail in te future.
    def parse_byte(self, byte: int) -> Tuple[Result, int]:
        INVALID_DISCARD_BYTE = (self.Result.PACKET_INVALID, 1)
        IN_PROGRESS = (self.Result.IN_PROGRESS, 1)

        if self.state == self.State.SEEK_SYNC:
            if byte == CRSF_SYNC or byte == CRSF_SYNC_EDGETX:
                self._msg = CRSFMessage()
                self._msg.payload.clear()

                self.state = self.State.MSG_LEN

                return IN_PROGRESS
            else:
                return INVALID_DISCARD_BYTE

        elif self.state == self.State.MSG_LEN:
            if byte > 64 - 2:
                self.state = self.State.SEEK_SYNC
                return INVALID_DISCARD_BYTE

            # Len includes 2 byte packet type and crc fields
            self._msg.length = byte - 2
            self.state = self.State.MSG_TYPE
            return IN_PROGRESS

        elif self.state == self.state.MSG_TYPE:
            try:
                self._msg.msg_type = PacketType(byte)
            except ValueError:
                self.state = self.State.SEEK_SYNC
                return INVALID_DISCARD_BYTE

            if self._msg.is_extended():
                self.state = self.State.MSG_EXT_DEST
                self._msg.length -= 2
            else:
                self.state = self.State.MSG_PAYLOAD
            return IN_PROGRESS

        elif self.state == self.State.MSG_EXT_DEST:
            self._msg.destination = byte
            self.state = self.State.MSG_EXT_SRC
            return IN_PROGRESS

        elif self.state == self.State.MSG_EXT_SRC:
            self._msg.source = byte
            self.state = self.State.MSG_PAYLOAD
            return IN_PROGRESS

        elif self.state == self.State.MSG_PAYLOAD:
            self._msg.payload.append(byte)
            if len(self._msg.payload) == self._msg.length:
                self.state = self.State.MSG_CRC
            return IN_PROGRESS

        elif self.state == self.State.MSG_CRC:
            if self._msg.calculate_crc() == byte:
                # Reset parser
                self.state = self.State.SEEK_SYNC

                # Packet len consists of payload length + 4 bytes for sync, len, type and crc
                # + 2 bytes if packet has an extended format
                return (
                    self.Result.PACKET_VALID,
                    len(self._msg.payload) + 4 + (2 if self._msg.is_extended() else 0),
                )
            else:
                return INVALID_DISCARD_BYTE
