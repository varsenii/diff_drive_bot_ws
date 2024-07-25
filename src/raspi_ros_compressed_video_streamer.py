# This code works with jpeg image streaming

import json
import websocket
import threading
import time
from picamera2 import Picamera2
import cv2
import base64
import argparse
import logging
import signal
import sys


picam2 = Picamera2()
logger = logging.getLogger(__name__)

def on_message(ws, message):
    logger.debug(f"Received: {message}")

def on_error(ws, error):
    logger.error(f"WebSocket Error: {error}")

def on_close(wsapp, close_status_code, close_msg):
    logger.info("### Connection closed ###")
    if close_status_code or close_msg:
        logger.info("close status code: " + str(close_status_code))
        logger.info("close message: " + str(close_msg))

def on_open(ws):
    def run(*args):
        while True:
            try:
                frame = picam2.capture_array()
                _, buffer = cv2.imencode('.jpg', frame)
                data = base64.b64encode(buffer).decode('utf-8')
                ros_compressed_msg = {
                    "header": {
                        "stamp": {"secs": int(time.time()), "nsecs": 0},
                        "frame_id": "camera",
                    },
                    "format": "jpeg",
                    "data": data,
                }
                message = {
                    "op": "publish",
                    "topic": "/camera/image_raw/compressed",
                    "msg": ros_compressed_msg,
                }
                ws.send(json.dumps(message))
                time.sleep(0.01)
            except Exception as e:
                logger.error(f"Error sending frame: {e}")
                break
        ws.close()
        logger.info("### Exiting WebSocket connection ###")

    t = threading.Thread(target=run)
    logger.info("Streaming video from Raspberry Pi to ROS server...")
    t.start()

def parse_arguments():
    parser = argparse.ArgumentParser(description='Stream video frames from Raspberry Pi to ROS using rosbridge.')
    parser.add_argument('--ip', type=str, default='192.168.2.11', help='IP address of the ROS server (default: 192.168.2.11)')
    parser.add_argument('--port', type=int, default=9090, help='Port of the ROS server (default: 9090)')
    parser.add_argument('--width', type=int, default=640, help='Width of the image (default: 640)')
    parser.add_argument('--height', type=int, default=480, help='Height of the image (default: 480)')
    parser.add_argument('--format', type=str, default='RGB888', help='Format of the image (default: RGB888)')
    return parser.parse_args()

def signal_handler(sig, frame):
    logger.info('Received SIGINT, shutting down...')
    picam2.stop()
    sys.exit(0)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    args = parse_arguments()
    video_config = picam2.create_video_configuration(main={"size": (args.width, args.height), "format": args.format})
    picam2.configure(video_config)
    picam2.start()

    # websocket.enableTrace(True)
    ws = websocket.WebSocketApp(f"ws://{args.ip}:{args.port}")
    ws.on_message = on_message
    ws.on_error = on_error
    ws.on_close = on_close
    ws.on_open = on_open

    signal.signal(signal.SIGINT, signal_handler)

    logger.info("Connecting to ROS server...")
    ws.run_forever()
