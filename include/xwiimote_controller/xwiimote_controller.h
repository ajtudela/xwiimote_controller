/*
 * ROS Node for using a wiimote control unit to direct a robot.
 * Copyright (c) 2020, Alberto Tudela.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

/*
 * Initial C++ implementation by
 *  Alberto Tudela <ajtudela@gmail.com>
 *
 * Revisions:
 *
 */

#ifndef XWIIMOTE_CONTROLLER_H
#define XWIIMOTE_CONTROLLER_H

// C
#include <stdio.h>
#include <poll.h>
extern "C" {
	#include "xwiimote.h"
}

// C++
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float32.h"

class WiimoteNode{
public:
	WiimoteNode(ros::NodeHandle& node, ros::NodeHandle& node_private);
	~WiimoteNode();
	int openInterface();
private:
	ros::NodeHandle node_, nodePrivate_;
	ros::ServiceServer paramsSrv_;
	ros::Publisher wiimoteStatePub_;
	ros::Subscriber rumbleSub_;
	ros::Time rumbleEnd_;
	
	int deviceIdx_;
	std::string devicePath_;
	struct xwii_iface *iface_;
	bool buttons_[11], nunchukButtons_[2], leds_[4], rumbleState_;
	float batteryPercent_, nunchukJoystick_[2], nunchuckAcceleration_[3], acceleration_[3], angularVelocity_[3];
	
	char *getDevice(int num);
	bool runInterface(struct xwii_iface *iface);
	void initializeWiimoteState();
	void publishState();
	void readLed();
	void readBattery();
	void toggleRumble(bool on);
	void setLed(unsigned int led_idx, bool on);
	void rumbleCallback(const std_msgs::Float32::ConstPtr& message);
	bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
};

#endif  // XWIIMOTE_CONTROLLER_H
