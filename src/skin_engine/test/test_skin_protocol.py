import numpy as np

from skin_engine.protocol import (
    ADC_BYTES,
    COLS,
    LINE_FOOTER,
    PACKET_MAGIC,
    DemoSource,
    FrameAssembler,
)


def test_demo_source_assembles_display_frame_without_dtype_cast_error():
    source = DemoSource()
    assembler = FrameAssembler(reorder=True)
    frame = None

    for _ in range(8):
        for packet in source.read_packets():
            next_frame = assembler.push_packet(packet)
            if next_frame is not None:
                frame = next_frame
        if frame is not None:
            break

    assert frame is not None
    assert frame.shape == (32, 16)
    assert frame.dtype == np.float32
    assert np.max(frame) > 0


def test_short_hardware_packets_assemble_without_dashboard_padding():
    assembler = FrameAssembler(reorder=False)
    frame = None

    for band in range(4):
        packet = _short_packet_with_value(row=0, col=0, value=100 + band)
        next_frame = assembler.push_packet(packet)
        if next_frame is not None:
            frame = next_frame

    assert frame is not None
    assert frame.dtype == np.float32
    assert frame[0, 0] == 100
    assert frame[8, 0] == 101
    assert frame[16, 0] == 102
    assert frame[24, 0] == 103


def _short_packet_with_value(row: int, col: int, value: int) -> bytes:
    data = bytearray(PACKET_MAGIC + bytes(ADC_BYTES) + LINE_FOOTER)
    pair = col // 2
    off = 3 + 24 * row + 3 * pair
    if col % 2 == 0:
        other = 0
        data[off] = ((value >> 8) << 4) | (other >> 8)
        data[off + 1] = value & 0xFF
        data[off + 2] = other & 0xFF
    else:
        other = 0
        data[off] = ((other >> 8) << 4) | (value >> 8)
        data[off + 1] = other & 0xFF
        data[off + 2] = value & 0xFF
    return bytes(data)
