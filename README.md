# mqtt-trilobot

First make sure your Pi is up to date.
```
sudo apt update
sudo apt upgrade

```


Install OpenCV

sudo apt install python3-opencv

Install PiCamera

sudo pip install picamera

Note:
If you're using VNC.

Uncommenting hdmi_force_hotplug=1 in /boot/config.txt then rebooting solved problems with enabling 'legacy camera'
and VNC server


#Installing and Upgrading Node-RED

https://nodered.org/docs/getting-started/raspberrypi

```
bash <(curl -sL https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered)
```

#Autostart on boot

To get Node-RED to run when the Pi is turned on, or re-booted, you can enable the service to autostart by running the command:
```
sudo systemctl enable nodered.service
```
To disable the service, run the command:
```
sudo systemctl disable nodered.service
```
#Mosquitto

```
sudo apt install mosquitto mosquitto-clients

sudo systemctl status mosquitto.


sudo nano /etc/mosquitto/mosquitto.conf
```

Move to the end of the file using the arrow keys and paste the following two lines:

```
listener 1883
allow_anonymous true
```


#Install Node Red Image and Alexa Tools
Manage Palette/Install

node-red-contrib-image-tools
node-red-contrib-alexa-home-skill
