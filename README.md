# U1 Ace
Adding the Anycubic ACE to the U1 printer


<br><br>

[![https://www.youtube.com/watch?v=nXq-57eN3aU](https://img.youtube.com/vi/nXq-57eN3aU/0.jpg)](https://www.youtube.com/watch?v=nXq-57eN3aU)

https://www.youtube.com/watch?v=nXq-57eN3aU<br>

<br><br>



# RFID Tags

You can use NTAG 213,215 and 216 rfid tags for this mod
<br>
App to write rfid tags for the U1:  <a href=https://github.com/DnG-Crafts/U1-RFID>U1-RFID - Github</a>
<br>

The android app is available on google play<br>
<a href="https://play.google.com/store/apps/details?id=dngsoftware.u1fid&hl=en"><img src=https://github.com/DnG-Crafts/U1-Ace/blob/main/images/gp.webp width="20%" height="20%"></a>

<br><br>



# Klipper settings

To enable the script you need to add the following line to `printer.cfg`<br>

```
[ace_device]
```

You only need the above line but optional variables can be added to modify how the ace works.

```
# these are the serial connection options and should not need to be modified

serial: /dev/serial/by-id/usb-ANYCUBIC_ACE_1-if00
baud: 115200


# speeds, 100 seems to be the maximum speed the ace will allow

feed_speed: 90
load_speed: 100
retract_speed: 40


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

retract_length_slot1: 1150
retract_length_slot2: 1150
retract_length_slot3: 1150
retract_length_slot4: 1150

```
<br><br>

# Filament Dryer

The filament dryer can be enabled or disabled from the fluidd or mainsail web ui using the following gcode commands


Starts the dryer for 240 minutes at 55 degrees celsius.
```
ACE_START_DRYING TEMPERATURE=55 DURATION=240
```


Stops the dryer, there is a cooldown time before the dryer stops after running this command.
```
ACE_STOP_DRYING
```

<br><br>

# Cable and extruder setup

[![https://www.youtube.com/watch?v=NxCtS9ZYoLk](https://img.youtube.com/vi/NxCtS9ZYoLk/0.jpg)](https://www.youtube.com/watch?v=NxCtS9ZYoLk)

https://www.youtube.com/watch?v=NxCtS9ZYoLk<br>

### Ace pinout

<img src=https://github.com/DnG-Crafts/U1-Ace/blob/main/images/ACE_PORT.jpg width=50% height=50%>
