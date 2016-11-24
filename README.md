[](http://www.email81.com/white.asp?eid=b393a631fc85d632)
# virtual-reality-project
This project contains my STM32 microcontroller source code and the android game source code for my VR project. The custom board you can see in the images below contains a senosr fusion hub, a microcontroller a gyroscope, magnetometer and an accelerometer. The sensor fusion hub fuses data from the sensors and provides accurate orientation information (quaternions) to the mircocontroller. The microcontroller acts as communications bridge and sends the orientation data over a USB HID link to an android phone.
When the board is attached to the side of a google VR cardboard viewer, the system can accurately track a user's head movements with less than 20 ms of latency.

The unity game is based on the unity flyer sample VR game and is compatible with google's cardboard viewer. The android plugin I created for the unity game reads in the orientation data for head tracking from the custom board using Android's USB manager. 


[video of system in operation](https://youtu.be/L4SEYiw9Usc)

# Quick start guide

## Prerequisites

### Minimum phone specifications
* OS: Android kitkat or later
* 1GB or higher of RAM
* Multicore processor with > 1GHz clock speeds.
* The phone must be capable of acting as a OTG host.

### Phone setup
* [Install the VR demo apk using ADB](https://developer.vuforia.com/library/articles/Solution/How-To-install-an-APK-using-ADB)
* [Enable airplane mode]
 (https://support.google.com/androidwear/answer/6056901?hl=en)
* [Disable power saving mode.](http://www.microcenter.com/tech_center/article/6521/how_to_change_the_power_saver_settings_on_an_android_tablet)
* [Disable auto brightness](http://www.androidpolice.com/2014/06/26/android-l-feature-spotlight-auto-brightness-is-gone-adaptive-brightness-takes-its-place/)

## Procedure

### Step 1

Insert the end of the OTG micro usb cable labeled "HOST" in to the phone and the other end in to the VR demo unit.

![setup](https://github.com/ruairilong2020/virtual-reality-project/blob/master/WIKI_IMAGES/setup.jpg)

### Step 2

The first time you connect the phone to the device, android will ask you for permission to allow the app to be launched whenever the device is connected. Click  the "Use by default for this USB device" check box and then click OK; this will also give the app permission to access data from the device.

![screenshot](https://github.com/ruairilong2020/virtual-reality-project/blob/master/WIKI_IMAGES/screenshot.png)

### Step 3
If the LED on the VR board is green on startup, this indicates that the system was booted up without warm start parameters. A "cold startup" will require performing steps 4 and step 5 to calibrate the system, otherwise if the LED is blue proceed to step 6.

### Step 4

When the app launches, place the phone in the google cardboard headset. Leave the headset on a table for several seconds before picking it up so that sensor fusion hub can learn the gyro bias and gyro noise levels. 

### Step 5

Pickup the headset and wave it slowly around in figure 8 motions about each axis to get as much coverage as possible so that sensor fusion hub can perform a mag auto calibration and can compensate for the sources of magnetic distortions in the phone.

![calibrate](https://github.com/ruairilong2020/virtual-reality-project/blob/master/WIKI_IMAGES/calibrate.png)

When the system has calibrated itself, the LED will turn a teal color indicating that the warm start parameters have been saved in flash.

### Step 6
For optimal results with warm start the next time the board is powered on, ensure that the phone is in the headset and secured with the screen on before plugging in the USB OTG cable; otherwise you may have to perform a quick recal by doing a few figure eight motions to obtain an accurate heading.
 

## Troubleshooting Matrix
| Symptom       | Probable Cause           | Corrective action  |
| ------------- |-------------| -----|
| LED is not turned on      | The board is not powered or programmed. | Make sure the correct end of the cable is plugged in to the phone.|
| LED is Red      | Communication problem with sensor fusion hub.      |   Visually inspect and perform a continuity check on all of the sensor fusion module's solder pads and re-solder any bad connections if necessary. |
| LED is Green | Board did not boot up with warm start parameters.      |    Perform the system calibration steps above. |
| LED is White| Multiple issues encountered on startup.      |    Try a reboot, if the problem still persists, hook up an FTDI cable to obtain debug information |
| LED is Blue| Everything is OK.      |    Try not to have too much fun. |
| Observable pitch roll errors| VR frames misalignment.      |    Move your head slowly and laterally so that the system can attempt to autocorrect. |


