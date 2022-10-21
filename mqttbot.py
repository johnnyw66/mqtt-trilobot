#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

from trilobot import *

import uuid
import logging
import argparse
import os
import hashlib
import subprocess
import time
from datetime import datetime

import socket
import json

import io
import picamera
import cv2
import numpy


from enum import Enum

MQTT_BROKER_HOST = '127.0.0.1'

class RobotState(Enum):
    STATE_IDLE = 0
    STATE_FOLLOW = 1
    STATE_TRACK = 2,
    STATE_WATCH = 3


class NetworkException(Exception):
    def __init__(self,msg):
        self.msg = msg

    def __str__(self):
        return f'NetworkException: {self.msg}'


class PauseException(Exception):
    def __init__(self,msg):
        self.msg = msg

    def __str__(self):
        return f'PauseException: {self.msg}'



def write_file(objects, image, filename):
    # write file
    for (x,y,w,h) in objects:
        cv2.rectangle(image, (x,y), (x+w,y+h),(255,255,0),4)

    cv2.imwrite(filename, image)

def snapshot():
    stream = io.BytesIO()

    # capture an image
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)
        camera.capture(stream, format='jpeg')

    # create an image buffer
    buff = numpy.frombuffer(stream.getvalue(), dtype=numpy.uint8)
    return buff


def detect_object(cascade):
    #stream = io.BytesIO()

    ## capture an image
    #with picamera.PiCamera() as camera:
    #    camera.resolution = (320, 240)
    #    camera.capture(stream, format='jpeg')

    # create an image buffer
    #buff = numpy.frombuffer(stream.getvalue(), dtype=numpy.uint8)
    buff = snapshot()

    # assign the image from the cv2 buffer
    image = cv2.imdecode(buff, 1)


    # convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # detect faces
    objects = cascade.detectMultiScale(gray, 1.1, 5)
    return objects, image, buff



def on_connect(client, userdata, flags, rc):

    logging.info(f"on_connect: Connected to MQTT Broker - setting up callbacks for trilobot")

    client.subscribe(f"trilobot/initialise",qos=1)
    client.subscribe(f"trilobot/colour",qos=1)
    client.subscribe(f"trilobot/percentage",qos=1)
    client.subscribe(f"trilobot/switch",qos=1)
    client.subscribe(f"trilobot/track",qos=1)
    client.subscribe(f"trilobot/watcher",qos=1)
    client.subscribe(f"trilobot/follow",qos=1)

    logging.info("on_connect: **Completed**")


def on_message(client, userdata, msg):
    logging.info(f"{hostname}: on_message: MQTT TOPIC {msg.topic}")

def on_disconnect(client, userdata,rc=0):
    logging.debug("Disconnected result code "+str(rc))
    #client.loop_stop()
    raise NetworkException("MQTT DisConnected")

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.254.254.254', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

def initialise_callback(client, userdata, msg):
    logging.info(f"initialise_callback: MQTT TOPIC {msg.topic}")
    try:
        msg_payload = str(msg.payload,'utf-8')
        logging.info(f"MQTT initialise_callback topic, Payload: {msg_payload}")
    except Exception as e:
        logging.info(f"MQTT Exception on trying to decode payload : '{e}'")
        pass

def set_robot_lights(hsv_colour):
    global tbot
    h = hsv_colour['hue']/360
    s = hsv_colour['saturation']
    v = hsv_colour['brightness']

    tbot.set_underlight_hsv(LIGHT_FRONT_LEFT, h, s, v, show=False)
    tbot.set_underlight_hsv(LIGHT_MIDDLE_LEFT, h, s, v, show=False)
    tbot.set_underlight_hsv(LIGHT_REAR_LEFT, h, s, v, show=False)
    tbot.set_underlight_hsv(LIGHT_REAR_RIGHT, h, s, v, show=False)
    tbot.set_underlight_hsv(LIGHT_MIDDLE_RIGHT, h, s, v, show=False)
    tbot.set_underlight_hsv(LIGHT_FRONT_RIGHT, h, s, v, show=False)
    tbot.show_underlighting()

def switch_watcher(on=True):
    global tbot
    if (on):
        robot_state = RobotState.STATE_WATCH
    else:
        robot_state = RobotState.STATE_IDLE


def switch_robot_lights(on=True):
    global tbot
    if (on):
        tbot.show_underlighting()
    else:
        tbot.fill_underlighting(0, 0, 0)


def switch_trackface(on=True):
    global tbot, robot_state
    logging.info(f"switch_trackface {on}")
    if (on):
        robot_state = RobotState.STATE_TRACK
        pass
    else:
        tbot.set_motor_speeds(0.0, 0.0)
        robot_state = RobotState.STATE_IDLE

def switch_robot_follow(on=False):
    global tbot, robot_state
    if (on):
        robot_state = RobotState.STATE_FOLLOW
    else:
        tbot.set_motor_speeds(0.0, 0.0)
        robot_state = RobotState.STATE_IDLE


def switch_watcher(on=False):
    global tbot, robot_state
    if (on):
        robot_state = RobotState.STATE_WATCH
    else:
        robot_state = RobotState.STATE_IDLE




def colour_callback(client, userdata, msg):
    logging.info(f"colour_callback: MQTT TOPIC {msg.topic}")
    try:
        msg_payload = json.loads(str(msg.payload,'utf-8'))
        logging.info(f"MQTT colour_callback topic, Payload: {msg_payload}")
        set_robot_lights(msg_payload)


    except Exception as e:
        logging.info(f"MQTT Exception on trying to decode payload : '{e}'")
        pass


def percentage_callback(client, userdata, msg):
    logging.info(f"percentage_callback: MQTT TOPIC {msg.topic}")
    try:
        msg_payload = str(msg.payload,'utf-8')
        logging.info(f"MQTT percentage_callback topic, Payload: {msg_payload}")

    except Exception as e:
        logging.info(f"MQTT Exception on trying to decode payload : '{e}'")
        pass



# Switch (Toggle) states from on to off (IDLE)

def switch_lights_callback(client, userdata, msg):

    logging.info(f"switch_lights_callback: MQTT TOPIC {msg.topic}")
    try:
        msg_payload = str(msg.payload,'utf-8')
        switch_robot_lights("true" in msg_payload)
        logging.info(f"MQTT switch_lights_callback topic, Payload: {msg_payload}")

    except Exception as e:
        logging.info(f"MQTT Exception on trying to decode payload : '{e}'")
        pass


def switch_follow_callback(client, userdata, msg):

    logging.info(f"switch_follow_callback: MQTT TOPIC {msg.topic}")
    try:
        msg_payload = str(msg.payload,'utf-8')
        logging.info(f"MQTT switch_follow_callback topic, Payload: {msg_payload}")
        switch_robot_follow("true" in msg_payload)

    except Exception as e:
        logging.info(f"MQTT Exception on trying to decode payload : '{e}'")
        pass


def switch_trackface_callback(client, userdata, msg):

    logging.info(f"switch_trackface_callback: MQTT TOPIC {msg.topic}")
    try:
        msg_payload = str(msg.payload,'utf-8')
        switch_trackface("true" in msg_payload)
        logging.info(f"MQTT switch_trackface_callback topic, Payload: {msg_payload}")

    except Exception as e:
        logging.info(f"MQTT Exception on trying to decode payload : '{e}'")
        pass

def switch_watcher_callback(client, userdata, msg):

    logging.info(f"switch_watch_callback: MQTT TOPIC {msg.topic}")
    try:
        msg_payload = str(msg.payload,'utf-8')
        switch_watcher("true" in msg_payload)
        logging.info(f"MQTT switch_watch_callback topic, Payload: {msg_payload}")

    except Exception as e:
        logging.info(f"MQTT Exception on trying to decode payload : '{e}'")
        pass



# State Loop Routines
def state_robot_idle():
    global tbot
    logging.info(f"STATE ROBOT IDLE")
    pass

def state_robot_follow():
    global tbot
    logging.info(f"STATE ROBOT FOLLOW")

    TOP_SPEED = 0.7         # The top speed (between 0.0 and 1.0) that the robot will drive at to get an object in range
    GOAL_DISTANCE = 20.0    # The distance in cm the robot will keep an object in front of it
    SPEED_RANGE = 5.0       # The distance an object is from the goal that will have the robot drive at full speed
    distance = tbot.read_distance()
    if (distance >= 0.0):
        scale = (distance - GOAL_DISTANCE) / SPEED_RANGE
        speed = max(min(scale, 1.0), -1.0) * TOP_SPEED
        tbot.set_motor_speeds(speed, speed)
    else:
        tbot.set_motor_speeds(0.0, 0.0)

def state_robot_watch():
    logging.info(f"STATE ROBOT WATCH")
    buff = snapshot()
    publish.single('trilobot/watchimage', buff.tobytes(), qos=1, retain=False, hostname=MQTT_BROKER_HOST)

def state_track_face():

    global tbot
    logging.info(f"STATE ROBOT TRACK FACE")
    # import the cascade file - needs to be in the same folder
    cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    objects, image, buff = detect_object(cascade)
    logging.info(f"return from detect_object: Found {str(len(objects))} objects(s)")

    if (len(objects) == 0):
        tbot.curve_forward_right(0.75)  # Three quarters speed
        time.sleep(0.25)
        tbot.stop()  # Apply the brakes
        time.sleep(1.0)
        #publish.single('trilobot/trackedimage', buff.tobytes(), qos=1, retain=False, hostname=MQTT_BROKER_HOST)
        pass
    else:
        #client.send('topic', byteArr, qos=1, hostname='m2m.eclipse.org')
        publish.single('trilobot/trackedimage', buff.tobytes(), qos=1, retain=False, hostname=MQTT_BROKER_HOST)
        pass





global robot_state

robot_state = RobotState.STATE_IDLE

parser = argparse.ArgumentParser(description="MQTT Bot",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument("-po", "--poll-time", default=1440, help="Poll Time in mins")
parser.add_argument("-mu", "--mqtt-user", default="bot", help="MQTT User")
parser.add_argument("-ms", "--mqtt-secret", default="xyzzy", help="MQTT Secret")
parser.add_argument("-mh", "--mqtt-host", default=MQTT_BROKER_HOST, help="MQTT Host")
parser.add_argument("-mp", "--mqtt-port", default=1883, help="MQTT Port")
#parser.add_argument("-v", "--voice", action='store_true',default=False, help="Suppress Voice")
parser.add_argument("-w", "--watch-dog", action='store_true',default=False, help="Auto Reset Watchdog")

args = parser.parse_args()
arg_config = vars(args)

logging.basicConfig(format='%(name)s %(levelname)s: %(asctime)s: %(message)s', level=logging.DEBUG)

tbot = Trilobot()

try:
    # We now have everything - Start the MQTT Client
    watch_dog = arg_config['watch_dog']

    hostname = socket.gethostname()
    ipaddress= get_ip()



    mqtt_id = f'mqtt-{str(uuid.uuid4())}'
    logging.info(f"MQTT Client ID: {mqtt_id}  Hostname: {hostname} ip: {ipaddress}")
    client=mqtt.Client(mqtt_id)

    #client.username_pw_set(username=arg_config["mqtt_user"],password=arg_config["mqtt_secret"])
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect

    client.message_callback_add(f"trilobot/initialise",initialise_callback)
    client.message_callback_add(f"trilobot/colour",colour_callback)
    client.message_callback_add(f"trilobot/percentage",percentage_callback)
    client.message_callback_add(f"trilobot/switch",switch_lights_callback)
    client.message_callback_add(f"trilobot/track",switch_trackface_callback)
    client.message_callback_add(f"trilobot/follow",switch_follow_callback)
    client.message_callback_add(f"trilobot/watcher",switch_watcher_callback)

    client.connect(host=arg_config["mqtt_host"],port=int(arg_config["mqtt_port"]),keepalive=60)
    client.on_message = on_message

    poll_start = time.time()
    poll_timeout = 60*int(arg_config['poll_time'])

    while True:
        time_passed = int(time.time() - poll_start)
        if ( time_passed >= poll_timeout):
            break
        time.sleep(0.0625)
        client.loop(timeout=1.0)

        if (robot_state == RobotState.STATE_FOLLOW):
            state_robot_follow()

        if (robot_state == RobotState.STATE_WATCH):
            state_robot_watch()

        if (robot_state == RobotState.STATE_TRACK):
            state_track_face()

        if (robot_state == RobotState.STATE_IDLE):
            state_robot_idle()

    raise PauseException('Leaving')


except Exception as e:
    logging.info(f"????{e}")
finally:
    logging.info(f"Finishing")
    client.loop_stop()
    exit()
