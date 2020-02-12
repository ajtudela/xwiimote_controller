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
#include "sensor_msgs/JoyFeedbackArray.h"

class WiimoteNode{
public:
	WiimoteNode(ros::NodeHandle& node, ros::NodeHandle& node_private);
	~WiimoteNode();
	int openInterface();
private:
	ros::NodeHandle node_, nodePrivate_;
	ros::ServiceServer paramsSrv_;
	ros::Publisher wiimoteStatePub_, joyPub_, wiimoteNunchukPub_;
	ros::Subscriber joySetFeedbackSub_;
	ros::Time rumbleEnd_;
	
	int deviceIdx_;
	std::string devicePath_;
	struct xwii_iface *iface_;
	bool wiimoteCalibrated_;
	bool buttons_[11], nunchukButtons_[2], leds_[4], rumbleState_;
	float batteryPercent_, nunchukJoystick_[2], nunchuckAcceleration_[3], acceleration_[3], angularVelocity_[3];
	float accelerationCal_[3];
	
	char *getDevice(int num);
	bool runInterface(struct xwii_iface *iface);
	void initializeWiimoteState();
	void publishJoy();
	void publishWiimoteState();
	void publishWiimoteNunchuk();
	void readLed();
	void readBattery();
	void toggleRumble(bool on);
	void setLed(unsigned int ledIdx, bool on);
	void setRumble(double duration);
	bool isPresentNunchuk();
	bool isPresentMotionPlus();
	void checkFactoryCalibrationData();
	void joySetFeedbackCallback(const sensor_msgs::JoyFeedbackArray::ConstPtr& feedback);	
	bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	
	// Convert wiimote accelerator readings from g's to m/sec^2:
	const double EARTH_GRAVITY_ = 9.80665;  // m/sec^2 @sea_level
};

#endif  // XWIIMOTE_CONTROLLER_H
