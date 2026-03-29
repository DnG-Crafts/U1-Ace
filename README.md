# U1 Ace
Adding the Anycubic ACE to the U1 printer


<br><br>

[![https://www.youtube.com/watch?v=YoNCkkrdzvg](https://img.youtube.com/vi/YoNCkkrdzvg/0.jpg)](https://www.youtube.com/watch?v=YoNCkkrdzvg)

https://www.youtube.com/watch?v=YoNCkkrdzvg<br>

<br><br>

# Extended Firmware

To use this you must install this version of the <a href=https://github.com/DnG-Crafts/SnapmakerU1-Extended-Firmware/releases>extended firmware</a><br>
The original version provided by <a href=https://github.com/paxx12>paxx12</a> does not have the edits required to run this script.<br>

The extended firmware adds many features and fixes which you can read about <a href=https://snapmakeru1-extended-firmware.pages.dev/>HERE</a>

<br><br>

# RFID Tags

You can use NTAG 213,215 and 216 rfid tags for this mod, in the app navigate to settings and enable `Ace Format` to create tags that the ace can read.
<br>
App to write rfid tags for the U1:  <a href=https://github.com/DnG-Crafts/U1-RFID>U1-RFID - Github</a>
<br>

The android app is available on google play<br>
<a href="https://play.google.com/store/apps/details?id=dngsoftware.u1rfid&hl=en"><img src=https://github.com/DnG-Crafts/U1-Ace/blob/main/images/gp.webp width="20%" height="20%"></a>

<br><br>

# Enable Ace Mod

To enable the ace mod use a web brower and navigate to `http://PRINTER-IP/firmware-config/` and you will find the Ace Mod options<br>
<img src=https://github.com/DnG-Crafts/U1-Ace/blob/main/images/ace_options.png width="70%" height="70%">

<br><br>

# Klipper Settings

These are the default config values found in '/extended/mods/ace_device.cfg', you do not need to modify these unless there is an issue.

```
[ace_device]


# these are the serial connection options and should not need to be modified

serial: /dev/serial/by-id/usb-ANYCUBIC_ACE_1-if00
baud: 115200


# speeds, 100 seems to be the maximum speed the ace will allow

feed_speed: 90
load_speed: 100
retract_speed: 25



# feeder mode allows you to leave the original U1 feeders on the printer
# and connect the ace to the U1 feeders on the sides of the printer.
# this allows you to still use the side spools if you needed.
# setting this to True will enable that functionality

enable_feeder_mode: False



# feed assist, setting this to False will disable feed assist

enable_feed_assist: True



# max temp for dryer

max_dryer_temperature: 55


# slot 1 2 3 4 are the individual numbered feeders/extruders
# this allows for different settings per slot


# the feed length is the distance from the ace to the u1
# the script detects the filament hitting the u1 sensor
# this only needs to be modified if the filament does not reach the u1 feeder

feed_length_slot1: 1000
feed_length_slot2: 1000
feed_length_slot3: 1000
feed_length_slot4: 1000


# the load length is the distance from the u1 feeder to the extruder
# these values should be fine unless you have extended the tubes

load_length_slot1: 850
load_length_slot2: 850
load_length_slot3: 850
load_length_slot4: 850


# the retract length is how far the ace retracts the filament when you
# unload the filament from the u1 touch screen menu
# if you do not like the loose filament on the spool set these values to 100 which
# is enough to clear the extruder but should not cause the loose filament issue shown in the video

retract_length_slot1: 100
retract_length_slot2: 100
retract_length_slot3: 100
retract_length_slot4: 100

```
<br><br>

# Filament Dryer

The filament dryer can be enabled or disabled from the fluidd or mainsail web ui using the following gcode commands.


Starts the dryer for 240 minutes at 55 degrees celsius.
```
ACE_START_DRYING TEMPERATURE=55 DURATION=240
```


Stops the dryer, there is a cooldown time before the dryer stops after running this command.
```
ACE_STOP_DRYING
```

<br><br>

# Cable and Extruder Setup

[![https://www.youtube.com/watch?v=NxCtS9ZYoLk](https://img.youtube.com/vi/NxCtS9ZYoLk/0.jpg)](https://www.youtube.com/watch?v=NxCtS9ZYoLk)

https://www.youtube.com/watch?v=NxCtS9ZYoLk<br>

<br><br>

### Ace Pinout

<img src=https://github.com/DnG-Crafts/U1-Ace/blob/main/images/ACE_PORT.jpg width=50% height=50%>

<br><br>

## Thankyou

Thankyou to the following people

<a href=https://github.com/paxx12>paxx12</a> for the extended firmware enabling the ability to mod the U1<br>
https://github.com/paxx12/SnapmakerU1-Extended-Firmware
<br><br>

<a href=https://github.com/Jookia>Jookia</a> for the raw data dumps from the ace<br>
https://github.com/printers-for-people/ACEResearch
<br><br>

<a href=https://github.com/utkabobr>utkabobr</a> for the python routines to keep the ace alive and responding<br>
https://github.com/utkabobr/DuckACE/
<br><br>

<a href=https://github.com/BlackFrogKok>BlackFrogKok</a> for the edit locations in the filament_feed.py to disable the factory feeders<br>
https://github.com/BlackFrogKok/SnapAce/
<br><br>