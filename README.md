# mqtt-trilobot

Install OpenCV

sudo apt install python3-opencv

Install PiCamera

sudo pip install picamera



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
sudo apt update
sudo apt upgrade

sudo apt install mosquitto mosquitto-clients.

sudo systemctl status mosquitto.


sudo nano /etc/mosquitto/mosquitto.conf
```

Move to the end of the file using the arrow keys and paste the following two lines:
```
listener 1883
allow_anonymous true
```


#Install Node Red Image Tools

node-red-contrib-image-tools
