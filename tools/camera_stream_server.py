#!/usr/bin/env python3
"""Lightweight MJPEG camera streamer for the SerBot Jetson."""

import argparse
import json
import signal
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
except Exception as exc:  # pragma: no cover - robot-side dependency
    cv2 = None
    CV2_IMPORT_ERROR = exc
else:
    CV2_IMPORT_ERROR = None


BOUNDARY = "serbot-frame"


class CameraState:
    def __init__(self, device, width, height, fps, quality):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.lock = threading.Lock()
        self.frame = None
        self.last_error = None
        self.last_frame_at = None
        self.running = False
        self.thread = None

    def start(self):
        if cv2 is None:
            raise RuntimeError("opencv import failed: %s" % CV2_IMPORT_ERROR)
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.5)

    def snapshot(self):
        with self.lock:
            return self.frame, self.last_error, self.last_frame_at

    def health(self):
        with self.lock:
            return {
                "ok": self.frame is not None,
                "device": self.device,
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
                "quality": self.quality,
                "last_error": self.last_error,
                "last_frame_at": self.last_frame_at,
            }

    def _capture_loop(self):
        interval = 1.0 / max(1.0, float(self.fps))
        while self.running:
            cap = self._open_capture()
            if cap is None:
                time.sleep(1.0)
                continue
            try:
                while self.running:
                    ok, image = cap.read()
                    if not ok or image is None:
                        self._set_error("camera read failed")
                        time.sleep(0.2)
                        break
                    ok, encoded = cv2.imencode(
                        ".jpg",
                        image,
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(self.quality)],
                    )
                    if not ok:
                        self._set_error("jpeg encode failed")
                        time.sleep(0.2)
                        continue
                    with self.lock:
                        self.frame = encoded.tobytes()
                        self.last_error = None
                        self.last_frame_at = time.time()
                    time.sleep(interval)
            finally:
                cap.release()

    def _open_capture(self):
        device = int(self.device) if str(self.device).isdigit() else self.device
        cap = cv2.VideoCapture(device, getattr(cv2, "CAP_V4L2", 200))
        if cap is None or not cap.isOpened():
            if cap is not None:
                cap.release()
            self._set_error("camera open failed: %s" % self.device)
            return None
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.height))
        cap.set(cv2.CAP_PROP_FPS, int(self.fps))
        return cap

    def _set_error(self, message):
        with self.lock:
            self.last_error = message


class CameraRequestHandler(BaseHTTPRequestHandler):
    server_version = "SerBotCamera/1.0"

    def do_GET(self):
        if self.path in ("/", "/health"):
            return self._serve_health()
        if self.path.startswith("/snapshot.jpg"):
            return self._serve_snapshot()
        if self.path.startswith("/stream.mjpg"):
            return self._serve_stream()
        self.send_error(HTTPStatus.NOT_FOUND)

    def _serve_health(self):
        payload = json.dumps(self.server.camera.health(), ensure_ascii=False).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _serve_snapshot(self):
        frame, error, _ = self.server.camera.snapshot()
        if not frame:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, error or "no camera frame yet")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def _serve_stream(self):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=%s" % BOUNDARY)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        last_frame = None
        while True:
            frame, _, _ = self.server.camera.snapshot()
            if not frame:
                time.sleep(0.05)
                continue
            if frame is last_frame:
                time.sleep(0.01)
                continue
            last_frame = frame
            try:
                self.wfile.write(("--%s\r\n" % BOUNDARY).encode("ascii"))
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(frame)).encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

    def log_message(self, fmt, *args):
        return


class CameraServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, address, camera):
        super().__init__(address, CameraRequestHandler)
        self.camera = camera


def build_parser():
    parser = argparse.ArgumentParser(description="SerBot MJPEG camera streamer")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--quality", type=int, default=75)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    camera = CameraState(args.device, args.width, args.height, args.fps, args.quality)
    camera.start()
    server = CameraServer((args.host, args.port), camera)

    def shutdown(signum, frame):
        camera.stop()
        server.shutdown()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print("SerBot camera stream listening on http://%s:%d/stream.mjpg" % (args.host, args.port))
    try:
        server.serve_forever()
    finally:
        camera.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
