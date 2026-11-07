"""Synthetic Velodyne HDL-32E data packets (1206 B, UDP port 2368).

Every field is little-endian. Layout: 12 blocks of (header 0xEEFF, azimuth in 1/100 degree,
32 channels of distance in 2 mm units and intensity), then a timestamp and 2 factory bytes.
One revolution at 600 rpm is 181 packets, so one revolution is 181 * 12 = 2172 firings.
"""

import struct
from collections.abc import Iterator

PACKET_SIZE = 1206
BLOCKS_PER_PACKET = 12
CHANNELS_PER_BLOCK = 32
PACKETS_PER_REVOLUTION = 181
FIRINGS_PER_REVOLUTION = PACKETS_PER_REVOLUTION * BLOCKS_PER_PACKET

BLOCK_HEADER = 0xEEFF
AZIMUTH_UNITS_PER_REVOLUTION = 36000
DISTANCE_UNIT_METERS = 0.002
DEFAULT_DISTANCE_METERS = 5.0
DEFAULT_INTENSITY = 100
# 0x37 = Strongest return, 0x21 = HDL-32E. A last byte of 161 would be read as VLS-128.
FACTORY_BYTES = bytes([0x37, 0x21])


def build_packet(
    packet_index: int,
    distance_meters: float = DEFAULT_DISTANCE_METERS,
    intensity: int = DEFAULT_INTENSITY,
) -> bytes:
    """Return packet number `packet_index`; the azimuth wraps once per 181 packets."""
    distance_raw = round(distance_meters / DISTANCE_UNIT_METERS)
    channels = struct.pack("<HB", distance_raw, intensity) * CHANNELS_PER_BLOCK
    blocks = []
    for block_index in range(BLOCKS_PER_PACKET):
        firing_index = packet_index * BLOCKS_PER_PACKET + block_index
        azimuth = (
            firing_index % FIRINGS_PER_REVOLUTION
        ) * AZIMUTH_UNITS_PER_REVOLUTION // FIRINGS_PER_REVOLUTION
        blocks.append(struct.pack("<HH", BLOCK_HEADER, azimuth) + channels)
    timestamp_microseconds = 0
    packet = b"".join(blocks) + struct.pack("<I", timestamp_microseconds) + FACTORY_BYTES
    assert len(packet) == PACKET_SIZE
    return packet


def build_revolutions(revolution_count: int) -> Iterator[bytes]:
    for packet_index in range(revolution_count * PACKETS_PER_REVOLUTION):
        yield build_packet(packet_index)
