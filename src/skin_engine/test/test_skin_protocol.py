import numpy as np

from skin_engine.protocol import DemoSource, FrameAssembler


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
