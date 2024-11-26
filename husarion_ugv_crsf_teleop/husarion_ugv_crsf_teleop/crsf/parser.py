# Based on https://github.com/crsf-wg/crsf/wiki/Python-Parser by crsf-wg

from typing import Tuple, Callable
from enum import Enum
from .message import CRSFMessage, PacketType, CRSF_SYNC, CRSF_SYNC_EDGETX, unpack_channels

class CRSFParser:
    class State(Enum):
        SEEK_SYNC = 0,
        MSG_LEN = 1,
        MSG_TYPE = 2,
        MSG_EXT_DEST = 3,
        MSG_EXT_SRC = 4,
        MSG_PAYLOAD = 5,
        MSG_CRC = 6
        
    class Result(Enum):
        PACKET_VALID = 0,
        IN_PROGRESS = 1,
        PACKET_INVALID = 2
        
    _msg: CRSFMessage
    _buffer: bytearray = bytearray()
    state: State = State.SEEK_SYNC
    on_message: Callable[[CRSFMessage], None]
    
    def __init__(self):
        pass
    
    # Parses CRSF messages from entire buffer
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
                pass
            
            elif result == self.Result.PACKET_INVALID:
                # Remove invalid message/byte from the buffer
                self._buffer = self._buffer[length:]
                i = 0
                pass
            
            elif result == self.Result.IN_PROGRESS:
                i += length
                pass
    
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
            
            # Len includes the 2 bytes for type and crc
            self._msg.length = byte - 2
            self.state = self.State.MSG_TYPE
            return IN_PROGRESS
        
        elif self.state == self.state.MSG_TYPE:
            try:
                self._msg.msg_type = PacketType(byte)
            except ValueError:
                #todo: logging
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
                
                # Packet len consists of payload length + 4 bytes for sync, len, type and crc + 2 bytes for extended format
                return (
                    self.Result.PACKET_VALID,
                    len(self._msg.payload) + 4 + (2 if self._msg.is_extended() else 0)
                )
            else:
                return INVALID_DISCARD_BYTE
