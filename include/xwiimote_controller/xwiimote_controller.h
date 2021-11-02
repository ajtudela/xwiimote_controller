/*
 * XWIIMOTE CONTROLLER CLASS
 *
 * Copyright (c) 2020-2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of xwiimote_controller project.
 * 
 * All rights reserved.
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
#include <ros/ros.h>
#include <sensor_msgs/JoyFeedbackArray.h>

class XWiimoteController{
public:
	XWiimoteController(ros::NodeHandle& node, ros::NodeHandle& node_private);
	~XWiimoteController();
	int openInterface();
	int runInterface();
	void closeInterface();
private:
	ros::NodeHandle node_, nodePrivate_;
	ros::ServiceServer paramsSrv_;
	ros::Publisher wiimoteStatePub_, joyPub_, wiimoteNunchukPub_, batteryPub_;
	ros::Subscriber joySetFeedbackSub_;
	ros::Time rumbleEnd_;

	int deviceIdx_;
	std::string devicePath_;
	struct xwii_iface *iface_;
	bool wiimoteCalibrated_, wiimoteConnected_;
	bool buttons_[15], nunchukButtons_[2], leds_[4], rumbleState_;
	float batteryPercent_, nunchukJoystick_[2], nunchuckAcceleration_[3], acceleration_[3], angularVelocity_[3];
	float accelerationCal_[3];

	void getParams();
	void joySetFeedbackCallback(const sensor_msgs::JoyFeedbackArray::ConstPtr& feedback);
	void publishBattery();
	void publishJoy();
	void publishWiimoteState();
	void publishWiimoteNunchuk();

	// Xwiimote related
	char *getDevice(int num);
	void initializeWiimoteState();
	void readLed();
	void readBattery();
	void toggleRumble(bool on);
	void setLed(unsigned int ledIdx, bool on);
	void setRumble(double duration);
	bool isPresentNunchuk();
	bool isPresentMotionPlus();
	void checkFactoryCalibrationData();

	// Convert wiimote accelerator readings from g's to m/sec^2:
	const double EARTH_GRAVITY_ = 9.80665;  // m/sec^2 @sea_level
};

#endif  // XWIIMOTE_CONTROLLER_H
