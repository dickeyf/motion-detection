import numpy as np
import cv2
import json
import base64
import os
import paho.mqtt.client as mqtt
import datetime as dt

last_2_pictures = []
sdThresh = 3
motion_debounce_seconds = 2
motion_detected_from = -1
last_motion_detected = -1

def insert_picture(picture):
    last_2_pictures.append(picture)
    # Don't keep more than 2 pictures in memory
    if len(last_2_pictures) > 2:
        last_2_pictures.pop(0)


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("dickeycloud/birdhouse/previews/v1/1")


def distMap(frame1, frame2):
    frame1_32 = np.float32(frame1)
    frame2_32 = np.float32(frame2)
    diff32 = frame1_32 - frame2_32
    norm32 = np.sqrt(diff32[:, :, 0] ** 2 + diff32[:, :, 1] ** 2 + diff32[:, :, 2] ** 2) / np.sqrt(
        255 ** 2 + 255 ** 2 + 255 ** 2)
    dist = np.uint8(norm32 * 255)
    return dist

def get_opencv_img_from_buffer(buffer, flags):
    bytes_as_np_array = np.frombuffer(buffer, dtype=np.uint8)
    return cv2.imdecode(bytes_as_np_array, flags)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    json_payload = msg.payload
    payload = json.loads(json_payload)
    jpeg = base64.b64decode(payload["picture"])
    insert_picture(jpeg)
    newFile = open("image.jpeg", "wb")
    newFile.write(jpeg)
    newFile.close()
    if len(last_2_pictures) == 2:
        img1 = get_opencv_img_from_buffer(last_2_pictures[0], cv2.IMREAD_COLOR)
        img2 = get_opencv_img_from_buffer(last_2_pictures[1], cv2.IMREAD_COLOR)
        rows, cols, _ = np.shape(img2)
        dist = distMap(img1, img2)
        # apply Gaussian smoothing
        mod = cv2.GaussianBlur(dist, (9,9), 0)
        # apply thresholding
        _, thresh = cv2.threshold(mod, 100, 255, 0)
        # calculate st dev test
        _, stDev = cv2.meanStdDev(mod)
        if stDev > sdThresh:
            current_time = dt.datetime.now().microsecond
            if motion_detected_from == -1:
                motion_detected_from = current_time

            motion = {
                "timestamp": payload["timestamp"],
                "stdDev": str(stDev)
            }
            client.publish("dickeycloud/birdhouse/motion/v1/1", json.dumps(motion))
            print("Motion detected! Timestamp: " + payload["timestamp"])

vmr_host = os.environ["VMR_HOST"]
mqtt_port = os.environ["MQTT_PORT"]
mqtt_username = os.environ["MQTT_USERNAME"]
mqtt_password = os.environ["MQTT_PASSWORD"]

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(mqtt_username, mqtt_password)
client.connect(vmr_host, int(mqtt_port), 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
