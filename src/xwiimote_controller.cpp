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
 
#include "xwiimote_controller/xwiimote_controller.h"
#include "xwiimote_controller/State.h"
#include "xwiimote_controller/IrSourceInfo.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <string>

/* Initialize the subscribers and publishers */
WiimoteNode::WiimoteNode(ros::NodeHandle& node, ros::NodeHandle& node_private) : node_(node), nodePrivate_(node_private){	
	paramsSrv_ = nodePrivate_.advertiseService("params", &WiimoteNode::updateParams, this);
	wiimoteStatePub_ = nodePrivate_.advertise<xwiimote_controller::State>("/wiimote/state", 1);
	rumbleSub_ = nodePrivate_.subscribe("/rumble", 10, &WiimoteNode::rumbleCallback, this);	
	
	initializeWiimoteState();	
}

/* Delete all parameteres */
WiimoteNode::~WiimoteNode(){
	nodePrivate_.deleteParam("device_idx");
	nodePrivate_.deleteParam("device_path");	
}

/* Initialize Wiimote State */
void WiimoteNode::initializeWiimoteState(){
	std_srvs::Empty empt; 
	updateParams(empt.request, empt.response);
	
	for(int i = 0; i < 11; i++){ 
		buttons_[i] = false;
	}
	for(int i = 0; i < 2; i++){
		nunchukButtons_[i] = false;
		nunchukJoystick_[i] = 0.0;
	}
	for(int i = 0; i < 4; i++){
		leds_[i] = false;
	}
	for(int i = 0; i < 3; i++){
		acceleration_[i] = 0.0;
		nunchuckAcceleration_[i] = 0.0;
		angularVelocity_[i] = 0.0;
	}
	
	batteryPercent_ = 0.0;	
	rumbleState_ = false;
}

/* Update parameters of the node */
bool WiimoteNode::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
	nodePrivate_.param<int>("device_idx", deviceIdx_, 1);
	nodePrivate_.param<std::string>("device_path", devicePath_, "");

	return true;
}

/* Get device to connect */
char *WiimoteNode::getDevice(int num){
	struct xwii_monitor *mon = xwii_monitor_new(false, false);
	if (!mon) {
		ROS_WARN("[xwiimote]: Cannot create monitor");
		return NULL;
	}

	char *ent;
	int i = 0;
	while ((ent = xwii_monitor_poll(mon))) {
		if (++i == num)
			break;
		free(ent);
	}

	xwii_monitor_unref(mon);
	if (!ent)
		ROS_WARN("[xwiimote]: Cannot find device with number #%d", num);

	return ent;
}

/* Open an interface for the wiimote */
int WiimoteNode::openInterface(){
	if(deviceIdx_ > 0) {
		devicePath_ = getDevice(deviceIdx_);
		if (devicePath_.empty()) {
		  ROS_FATAL("[xwiimote]: Cannot find device %i", deviceIdx_);
		  return -1;
		}
	}else if(devicePath_.empty()) {
		ROS_FATAL("[xwiimote]: You need to specify either device_idx or device_path!");
		return -1;
	}

	int ret = xwii_iface_new(&iface_, devicePath_.c_str());
	if(ret){
		ROS_FATAL("[xwiimote]: Cannot create xwii_iface '%s' err:%d", devicePath_.c_str(), ret);
		return -1;
	}

	ret = xwii_iface_open(iface_, xwii_iface_available(iface_) | XWII_IFACE_WRITABLE);
	if(ret) {
		ROS_FATAL("[xwiimote]:  Cannot open interface: %d", ret);
		return -1;
	}
	ROS_INFO("[xwiimote]: Successfully open Wiimote device '%s'", devicePath_.c_str());

	ret = runInterface(iface_);
	if (ret) {
		ROS_FATAL("[xwiimote]: Program failed; press any key to exit");
		return -1;
	}
	xwii_iface_unref(iface_);
}

/* Run the wiimote */
bool WiimoteNode::runInterface(struct xwii_iface *iface){
	struct xwii_event event;
	int ret = 0, fds_num;
	struct pollfd fds[2];

	memset(fds, 0, sizeof(fds));
	fds[0].fd = 0;
	fds[0].events = POLLIN;
	fds[1].fd = xwii_iface_get_fd(iface);
	fds[1].events = POLLIN;
	fds_num = 2;

	if (xwii_iface_watch(iface, true)) {
		ROS_FATAL("[xwiimote]: Cannot initialize hotplug watch descriptor");
		return false;
	}
	// Calibrate remote
	int32_t xCal, yCal, zCal, factor;
	xwii_iface_get_mp_normalization(iface_, &xCal, &yCal, &zCal, &factor);
	xCal = event.v.abs[0].x + xCal;
	yCal = event.v.abs[0].y + yCal;
	zCal = event.v.abs[0].z + zCal;
	xwii_iface_set_mp_normalization(iface_, xCal, yCal, zCal, factor);

	unsigned int code;
	float x, y, accex, accey, accez, naccex, naccey, naccez;
	bool pressed, needPub = false;
	while (ros::ok()) {
		ret = poll(fds, fds_num, -1);
		if (ret < 0) {
			if (errno != EINTR) {
				ret = -errno;
				ROS_FATAL("[xwiimote]: Cannot poll fds: %d", ret);
				break;
			}
		}

		ret = xwii_iface_dispatch(iface, &event, sizeof(event));
		if (ret) {
			if (ret != -EAGAIN)
			ROS_ERROR("[xwiimote]: Read failed with err:%d", ret);
			continue;
		}
		switch (event.type) {
			case XWII_EVENT_GONE:
				ROS_WARN("[xwiimote]: Device gone");
				fds[1].fd = -1;
				fds[1].events = 0;
				fds_num = 1;
				exit(1);
				break;
			case XWII_EVENT_NUNCHUK_KEY:
			case XWII_EVENT_KEY:
				needPub = true;
				code = event.v.key.code;
				pressed = event.v.key.state;
				switch (code) {
					case XWII_KEY_ONE:
						buttons_[0] = pressed;
						break;
					case XWII_KEY_TWO:
						buttons_[1] = pressed;
						break;
					case XWII_KEY_A:
						buttons_[2] = pressed;
						break;
					case XWII_KEY_B:
						buttons_[3] = pressed;						
						break;
					case XWII_KEY_PLUS:
						buttons_[4] = pressed;
						break;
					case XWII_KEY_MINUS:
						buttons_[5] = pressed;	
						break;				
					case XWII_KEY_LEFT:
						buttons_[6] = -pressed;
						break;
					case XWII_KEY_RIGHT:
						buttons_[7] =  pressed;
						break;
					case XWII_KEY_UP:
						buttons_[8] =  pressed;
						break;						
					case XWII_KEY_DOWN:
						buttons_[9] = -pressed;
						break;
					case XWII_KEY_HOME:
						buttons_[10] = pressed;
						break;
					case XWII_KEY_Z:
						nunchukButtons_[0] = pressed;
						break;
					case XWII_KEY_C:
						nunchukButtons_[1] = pressed;
						break;
					default:
						needPub = false;
						break;
				}
				break;
			case XWII_EVENT_NUNCHUK_MOVE:
				//ROS_INFO_THROTTLE(1, "XWII_EVENT_NUNCHUK_MOVE:%i, %i", event.v.abs[0].x, event.v.abs[0].y);
				needPub = true;
				x = 0.01 * event.v.abs[0].x;
				if (x < -1)
					x = -1;
				else if (x > 1)
					x = 1;
				nunchukJoystick_[0] = x;
				
				y = 0.01 * event.v.abs[0].y;
				if (y < -1)
					y = -1;
				else if (y > 1)
					y = 1;
				nunchukJoystick_[1] = y;
				
				naccex = event.v.abs[1].x;
				naccex /= 512;
				if (naccex >= 0)
					naccex = 10 * pow(naccex, 0.25);
				else
					accex = -10 * pow(-naccex, 0.25);
				nunchuckAcceleration_[0] = naccex;
				
				naccey = event.v.abs[1].y;
				naccey /= 512;
				if (naccey >= 0)
					naccey = 5 * pow(naccey, 0.25);
				else
					naccey = -5 * pow(-naccey, 0.25);
				nunchuckAcceleration_[1] = naccey;
				
				naccez = event.v.abs[1].z;
				naccez /= 512;
				if (naccez >= 0)
					naccez = 5 * pow(naccez, 0.25);
				else
					naccez = -5 * pow(-naccez, 0.25);
				nunchuckAcceleration_[2] = naccez;				
				break;
//#if 0
			case XWII_EVENT_WATCH:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_WATCH");
				break;
			case XWII_EVENT_ACCEL:
				//ROS_INFO_THROTTLE(1, "XWII_EVENT_ACCEL");
				needPub = true;
				accex = event.v.abs[0].x;
				accex /= 512;
				if (accex >= 0)
					accex = 10 * pow(accex, 0.25);
				else
					accex = -10 * pow(-accex, 0.25);
				acceleration_[0] = accex;
				
				accey = event.v.abs[0].y;
				accey /= 512;
				if (accey >= 0)
					accey = 5 * pow(accey, 0.25);
				else
					accey = -5 * pow(-accey, 0.25);
				acceleration_[1] = accey;
				
				accez = event.v.abs[0].z;
				accez /= 512;
				if (accez >= 0)
					accez = 5 * pow(accez, 0.25);
				else
					accez = -5 * pow(-accez, 0.25);
				acceleration_[2] = accez;				
				break;
			case XWII_EVENT_IR:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_IR");
				break;
			case XWII_EVENT_MOTION_PLUS:
				//ROS_INFO_THROTTLE(1, "XWII_EVENT_MOTION_PLUS");
				needPub = true;
				angularVelocity_[0] = event.v.abs[0].x;
				angularVelocity_[1] = event.v.abs[0].y;
				angularVelocity_[2] = event.v.abs[0].z;	
				break;
			case XWII_EVENT_CLASSIC_CONTROLLER_KEY:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_CLASSIC_CONTROLLER_KEY");
				break;
			case XWII_EVENT_CLASSIC_CONTROLLER_MOVE:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_CLASSIC_CONTROLLER_MOVE");
				break;
			case XWII_EVENT_BALANCE_BOARD:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_BALANCE_BOARD");
				break;
			case XWII_EVENT_PRO_CONTROLLER_KEY:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_PRO_CONTROLLER_KEY");
				break;
			case XWII_EVENT_PRO_CONTROLLER_MOVE:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_WAXWII_EVENT_PRO_CONTROLLER_MOVETCH");
				break;
			case XWII_EVENT_GUITAR_KEY:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_GUITAR_KEY");
				break;
			case XWII_EVENT_GUITAR_MOVE:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_GUITAR_MOVE");
				break;
			case XWII_EVENT_DRUMS_KEY:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_DRUMS_KEY");
				break;
			case XWII_EVENT_DRUMS_MOVE:
				ROS_INFO_THROTTLE(1, "XWII_EVENT_DRUMS_MOVE");
				break;
//#endif
			default:
				break;
		} // end switch

		// Check rumble end
		if(rumbleState_ && ros::Time::now() > rumbleEnd_) {
			toggleRumble(false);
			rumbleState_ = false;
		}
		
		// Check leds
		readLed();
		
		// Check Battery
		readBattery();
		
		if(needPub) {			
			publishState();
		}
		
		ros::spinOnce();
	} // end while ros::ok
	
	return ret;
}

/* Toggle rumble motor */
void WiimoteNode::toggleRumble(bool on){
	ROS_INFO_THROTTLE(1, "rumble(%i)", on);
	unsigned int ret = xwii_iface_rumble(iface_, on);
	if (ret) {
		ROS_ERROR("Error: Cannot toggle rumble motor: %d", ret);
	}
}

/* Set rumble for duration in seconds */
void WiimoteNode::rumbleCallback(const std_msgs::Float32::ConstPtr& message){
	// compute rumbleEnd
	double duration = message->data;
	if(duration < 1E-2){
		duration = 1E-2;
	}else if (duration > 10){
		duration = 10;
	}
	rumbleEnd_ = ros::Time::now() + ros::Duration(duration);
	ROS_INFO("Duration %f", duration);
	// start rumble
	toggleRumble(true);
	rumbleState_ = true;
}

/* Read leds */
void WiimoteNode::readLed(){
	bool stateLed;
	for(int i = 0; i < 4; i++){
		unsigned int ret = xwii_iface_get_led(iface_, i+1, &stateLed);
		leds_[i] = stateLed;
		if (ret) {
			ROS_ERROR("Error: Cannot read leds %d", ret);
		}
	}
}

/* Read battery percent */
void WiimoteNode::readBattery(){
	uint8_t capacity;
	unsigned int ret = xwii_iface_get_battery(iface_, &capacity);
	batteryPercent_ = (float)capacity;
	if (ret) {
		ROS_ERROR("Error: Cannot read battery %d", ret);
	}
}

/* Set Leds */
void WiimoteNode::setLed(unsigned int led_idx, bool on){
	ROS_INFO_THROTTLE(1, "led(led #%i: %i)", led_idx, on);
	unsigned int ret = xwii_iface_set_led(iface_, XWII_LED(led_idx), on);
	if (ret) {
		ROS_ERROR("Error: Cannot toggle led #%i: %d", led_idx, ret);
	}
}

/* Publish all */
void WiimoteNode::publishState(){
	xwiimote_controller::State state;
	state.header.stamp = ros::Time::now();
	
	for(int i = 0; i < 11; i++){
		state.buttons[i] = buttons_[i];
	}
	for(int i = 0; i < 2; i++){
		state.nunchuk_buttons[i] = nunchukButtons_[i];
		state.nunchuk_joystick[i] = nunchukJoystick_[i];
	}
	for(int i = 0; i < 4; i++){
		state.LEDs[i] = leds_[i];
	}

	state.linear_acceleration.x = acceleration_[0];
	state.linear_acceleration.y = acceleration_[1];
	state.linear_acceleration.z = acceleration_[2];
	
	state.nunchuk_acceleration.x = nunchuckAcceleration_[0];
	state.nunchuk_acceleration.y = nunchuckAcceleration_[1];
	state.nunchuk_acceleration.z = nunchuckAcceleration_[2];
	
	state.angular_velocity.x = angularVelocity_[0];
	state.angular_velocity.y = angularVelocity_[1];
	state.angular_velocity.z = angularVelocity_[2];
			
	state.rumble = rumbleState_;
	state.percent_battery = batteryPercent_;
	
	wiimoteStatePub_.publish(state);
}

int main(int argc, char **argv) {
	int deviceIdx_;
	std::string devicePath_;
	static struct xwii_iface *iface_;	
	
	ros::init(argc, argv, "xwiimote");
	ros::NodeHandle node("");
	ros::NodeHandle node_private("~");  
	
    try{
		ROS_INFO("[xwiimote]: Initializing node");		
		WiimoteNode wiimotenode(node, node_private);
		wiimotenode.openInterface();
	}
	catch(const char* s){
		ROS_FATAL_STREAM("[xwiimote]: " << s);
	}
	catch(...){
		ROS_FATAL_STREAM("[xwiimote]: Unexpected error");
	}

    return 0;
}
