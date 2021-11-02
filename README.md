# xwiimote_controller

![ROS](https://img.shields.io/badge/ros-melodic-blue?style=for-the-badge&logo=ros&logoColor=white)

## Overview

This package is designed to control a robot with a Nintendo [Wiimote] remote w/o nunchuck using the new driver xwiimote.

The official *[ROS wiimote]* package is based on the [CWiiD library] and only works with older Wiimotes (*Nintendo RVL-CNT-01*). This package is based on the newer [xwiimote] and is also compatible with newer Wiimotes (`Nintendo RVL-CNT-01-TR`) and Nunchuks.

**Keywords:** ROS, Nintendo, Wiimote, xwiimote

### License

**Author: Alberto Tudela<br />**

The xwiimote_controller package has been tested under [ROS] Melodic on [Ubuntu] 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [xwiimote] (library for control wiimote)

	sudo rosdep install --from-paths src

#### Building xwiimote library
The xwiimote library (version 2-3build1) is included in Ubuntu packages. Unfortunately, you need to compile the latest version from sources if you want the Nunchuk to be recognized.

Build the xwiimote library as follows:

```console
$ sudo apt purge xwiimote libxwiimote2
$ sudo apt install  libudev-dev  libncurses-dev
$ git clone https://github.com/dvdhrm/xwiimote.git
$ cd xwiimote
$ sh autogen.sh
$ make
$ sudo make install
```

Then you can use it:

```console
$ xwiishow list
Listing connected Wii Remote devices:
  Found device #1: /sys/devices/pci0000:00/0000:00:1d.0/usb5/5-2/5-2:1.0/bluetooth/hci0/hci0:12/0005:057E:0330.0002
End of device list

$ xwiishow 1
```

#### Building xwiimote_controller

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ajtudela/xwiimote_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make


## Bluetooth configuration and pairing

First check Kernel module ```hid-wiimote``` is loaded.

```console
$ sudo modprobe hid-wiimote
$ lsmod | grep wii
Expected output:
hid_wiimote            XXXX  0
```

To make the loading of this module permanent after boot:

```console
$ sudo nano /etc/modules
Add:
hid-wiimote
```

Add current user to group "input" (from [here](https://github.com/dvdhrm/xwiimote/issues/6)):

```console
$ sudo usermod -aG input $USER
```

Or install udev rules:
```console
$ sudo cp /rules/wiimote.rules /etc/udev/rules.d/
```

Pair your Wiimote with your computer
using `blueman` (Bluetooth device manager) and connect to HID:

```console
$ sudo hcitool dev
Devices:
  hci0  00:19:0E:16:AF:22
$ sudo hcitool scan
Scanning ...
  40:F4:07:C5:B7:BD Nintendo RVL-CNT-01-TR
$ blueman-manager
```

Press the red sync button on the back of the WiiMote, the 4 leds will blink.
Then in the Bluetooth device manager:

```console
"Search" button
  Nintendo RVL-CNT-01-TR
  Peripheral
  40:F4:07:C5:B7:BD
Right click > Pair
Right click > Connect to HID
```
Check it works:

```console
$ sudo evtest
/dev/input/event15: Nintendo Wii Remote Accelerometer
/dev/input/event16: Nintendo Wii Remote IR
/dev/input/event17: Nintendo Wii Remote
/dev/input/event18: Nintendo Wii Remote Nunchuk
/dev/input/event19: Nintendo Wii Remote Motion Plus
```

Troubleshooting:

If you are prompted for PIN input, it is because
you pressed 1+2 instead of the red sync button.
If it still asks for the PIN code with the red button,
you can try "Installation > pair without PIN code".


## Usage

Run the main node with:

	roslaunch xwiimote_controller xwiimote.launch

## Nodes

### xwiimote_controller

Control the Nintendo Wiimmote.

#### Subscribed Topics

* **`joy/set_feedback`** ([sensor_msgs/JoyFeedbackArray])

	Topic where ROS clients control the Wiimote's leds and rumble (vibrator) facility.
		* Type of the feedback: TYPE_LED = 0 or TYPE_RUMBLE = 1.
		* Id number for each type of each feedback. The first led would be id = 0. If rumble, id = 0.
		* Intensity of the feedback as long as it would be publishing.

#### Published Topics

* **`/wiimote/state`** ([xwiimote_controller/State])

	Topic for comprehensive information about Wiimote state. Including battery percent, led status, acceleration and angular velocity.
	List of buttons and axes:
	
	11+2 buttons (O=released, 1=pressed):
	
	0. XWII_KEY_ONE
	1. XWII_KEY_TWO
	2. XWII_KEY_A
	3. XWII_KEY_B
	4. XWII_KEY_PLUS
	5. XWII_KEY_MINUS
	6. XWII_KEY_LEFT
	7. XWII_KEY_RIGHT
	8. XWII_KEY_UP
	9. XWII_KEY_DOWN
	10. XWII_KEY_HOME
	11. XWII_KEY_Z
	12. XWII_KEY_C
	
	2 axes:
	
	1. nunchuk left-right joystick (floating value in the range -1=left .. 1=right)
	2. nunchuk down-up joystick (floating value in the range -1=down .. 1=up)

* **`/wiimote/joy`** ([sensor_msgs/Joy])

	Topic for buttons information only to maintain compatibility.

* **`/wiimote/battery`** ([sensor_msgs/BatteryState])

	Topic for battery information only to maintain compatibility.

#### Parameters

* **`device_idx`** (int, default: -1)

	The index of the Wiimote device to use. 
	Starts at 1, so if you want to use the seconde Wiimote, use `_device_idx:=2`
	Leaves at -1 to skip this parameter, in this case you need to specify the device index with `~device_path`.

* **`device_path`** (std::string, default: "")

	The full path of the Wiimote device to use.
	It's an absolute sysfs path to the device's root-node. This is normally a path to `/sys/bus/hid/devices/[dev]/`. You can use this path to create a new struct xwii_iface object.
	Leaves empty (`""`) to skip this parameter, in this case you need to specify the device index with `~device_idx`.


[Ubuntu]: https://ubuntu.com/
[ROS]: http://www.ros.org
[Wiimote]: https://www.nintendo.es/Wii/Accesorios/Accesorios-Wii-Nintendo-Ib-eacute-rica-626430.html
[ROS wiimote]: http://wiki.ros.org/wiimote
[CWiiD library]: https://help.ubuntu.com/community/CWiiD
[xwiimote]: https://dvdhrm.github.io/xwiimote/
[sensor_msgs/JoyFeedbackArray]: http://docs.ros.org/api/sensor_msgs/html/msg/JoyFeedbackArray.html
[sensor_msgs/Joy]: http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html
[sensor_msgs/BatteryState]: http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html
[xwiimote_controller/State]: /msg/State.msg
