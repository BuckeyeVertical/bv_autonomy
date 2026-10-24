"""H.264/fMP4 encoding for the dashboard's existing /video WebSocket."""

from fractions import Fraction

from bv_autonomy.types import Frame

PREVIEW_FPS = 15
_MAX_BOX_BYTES = 32 * 1024 * 1024


class _Mp4Output:
    """Non-seekable muxer output, drained as complete MP4 boxes."""

    def __init__(self):
        self.buffer = bytearray()

    def writable(self):
        return True

    def write(self, data):
        self.buffer.extend(data)
        if len(self.buffer) > _MAX_BOX_BYTES:
            raise ValueError("preview muxer output exceeded its buffer limit")
        return len(data)

    def boxes(self):
        while len(self.buffer) >= 8:
            size = int.from_bytes(self.buffer[:4], "big")
            if not 8 <= size <= _MAX_BOX_BYTES:
                raise ValueError("unsupported preview MP4 box size")
            if len(self.buffer) < size:
                return
            box = bytes(self.buffer[:size])
            del self.buffer[:size]
            yield box


class H264PreviewEncoder:
    """Owns one encoder session. All methods run on the same worker thread.

    Each emitted fragment begins with a keyframe, so a slow or newly connected
    viewer can start at the next fragment after receiving the cached header.
    """

    def __init__(self):
        # Import only when an operator requests video; the rest of the GCS can
        # still serve controls when the optional video dependency is missing.
        import av

        self._av = av
        self._output = _Mp4Output()
        self._container = None
        self._stream = None
        self._init = b""
        self._fragment = b""
        self._first_time = None
        self._pts = -1

    def _open(self, frame: Frame):
        height, width = frame.image.shape[:2]
        scale = min(640 / width, 480 / height, 1.0)
        self._container = self._av.open(self._output, mode="w", format="mp4", options={
            "movflags": "empty_moov+default_base_moof+frag_keyframe",
            "flush_packets": "1",
        })
        self._stream = self._container.add_stream("libx264", rate=PREVIEW_FPS)
        self._stream.width = max(2, int(width * scale) // 2 * 2)
        self._stream.height = max(2, int(height * scale) // 2 * 2)
        self._stream.pix_fmt = "yuv420p"
        self._stream.bit_rate = 400_000
        self._stream.codec_context.time_base = Fraction(1, PREVIEW_FPS)
        self._stream.codec_context.thread_count = 1
        self._stream.codec_context.gop_size = 3
        self._stream.codec_context.max_b_frames = 0
        self._stream.options = {
            "preset": "ultrafast", "tune": "zerolatency", "profile": "baseline", "level": "3.0",
            "x264-params": "keyint=3:min-keyint=3:scenecut=0",
        }

    def encode(self, frame: Frame) -> tuple[bytes, list[bytes]]:
        if self._container is None:
            self._open(frame)
        if self._first_time is None:
            self._first_time = frame.time
        self._pts = max(self._pts + 1, round((frame.time - self._first_time) * PREVIEW_FPS))
        image = self._av.VideoFrame.from_ndarray(frame.image, format="bgr24")
        image = image.reformat(width=self._stream.width, height=self._stream.height, format="yuv420p")
        image.pts = self._pts
        image.time_base = Fraction(1, PREVIEW_FPS)
        for packet in self._stream.encode(image):
            self._container.mux(packet)

        fragments = []
        for box in self._output.boxes():
            kind = box[4:8]
            if kind in (b"ftyp", b"moov"):
                self._init += box
            elif kind == b"moof":
                self._fragment = box
            elif kind == b"mdat" and self._fragment:
                fragments.append(self._fragment + box)
                self._fragment = b""
        return self._init, fragments

    def close(self):
        if self._container is not None:
            # The live session is ending; discard any final, unsent fragment.
            container, self._container = self._container, None
            container.close()
