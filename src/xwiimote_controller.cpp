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
#include "sensor_msgs/Joy.h"
#include <string>

/* Initialize the subscribers and publishers */
WiimoteNode::WiimoteNode(ros::NodeHandle& node, ros::NodeHandle& node_private) : node_(node), nodePrivate_(node_private){	
	paramsSrv_ = nodePrivate_.advertiseService("params", &WiimoteNode::updateParams, this);
	wiimoteStatePub_ = nodePrivate_.advertise<xwiimote_controller::State>("/wiimote/state", 1);
	joyPub_ = nodePrivate_.advertise<sensor_msgs::Joy>("/joy", 1);
	
	joySetFeedbackSub_ = nodePrivate_.subscribe<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 10, &WiimoteNode::joySetFeedbackCallback, this);
	
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
	wiimoteCalibrated_ = false;
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
	
	// Give the hardware time to zero the accelerometer and gyro after pairing
    // Ensure we are getting valid data before using
    sleep(1);

    checkFactoryCalibrationData();

    /*if (!wiimoteCalibrated_){
		ROS_ERROR("Wiimote not usable due to calibration failure.");
    }*/
	
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
				// Create a deadzone in the center
				if (fabs(x) <= 0.05){
					nunchukJoystick_[0] = 0.0;
				}else{
					nunchukJoystick_[0] = x;
				}
				
				y = 0.01 * event.v.abs[0].y;
				if (y < -1)
					y = -1;
				else if (y > 1)
					y = 1;
				// Create a deadzone in the center
				if (fabs(y) <= 0.05){
					nunchukJoystick_[1] = 0.0;
				}else{
					nunchukJoystick_[1] = y;
				}
				
				naccex = event.v.abs[1].x;
				/*naccex /= 512;
				if (naccex >= 0)
					naccex = 10 * pow(naccex, 0.25);
				else
					accex = -10 * pow(-naccex, 0.25);*/
				nunchuckAcceleration_[0] = naccex;
				
				naccey = event.v.abs[1].y;
				/*naccey /= 512;
				if (naccey >= 0)
					naccey = 5 * pow(naccey, 0.25);
				else
					naccey = -5 * pow(-naccey, 0.25);*/
				nunchuckAcceleration_[1] = naccey;
				
				naccez = event.v.abs[1].z;
				/*naccez /= 512;
				if (naccez >= 0)
					naccez = 5 * pow(naccez, 0.25);
				else
					naccez = -5 * pow(-naccez, 0.25);*/
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
				/*accex /= 512;
				if (accex >= 0)
					accex = 10 * pow(accex, 0.25);
				else
					accex = -10 * pow(-accex, 0.25);*/
				acceleration_[0] = accex;
				
				accey = event.v.abs[0].y;
				/*accey /= 512;
				if (accey >= 0)
					accey = 5 * pow(accey, 0.25);
				else
					accey = -5 * pow(-accey, 0.25);*/
				acceleration_[1] = accey;
				
				accez = event.v.abs[0].z;
				/*accez /= 512;
				if (accez >= 0)
					accez = 5 * pow(accez, 0.25);
				else
					accez = -5 * pow(-accez, 0.25);*/
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
			publishJoy();		
			publishWiimoteState();
			publishWiimoteNunchuk();
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

/* Set rumble and leds */
void WiimoteNode::joySetFeedbackCallback(const sensor_msgs::JoyFeedbackArray::ConstPtr& feedback){
	for (std::vector<sensor_msgs::JoyFeedback>::const_iterator it = feedback->array.begin(); it != feedback->array.end(); ++it){
		if ((*it).type == sensor_msgs::JoyFeedback::TYPE_LED){
			if((*it).intensity >= 0.5){
				if ((*it).id > 3){
					ROS_WARN("LED ID %d out of bounds; ignoring!", (*it).id);
				}else{	
					setLed((*it).id, true);
				}
			}else{
				if ((*it).id > 3){
					ROS_WARN("LED ID %d out of bounds; ignoring!", (*it).id);
				}else{	
					setLed((*it).id, false);
				}
			}
		}else if((*it).type == sensor_msgs::JoyFeedback::TYPE_RUMBLE){
			if ((*it).id > 0){
				ROS_WARN("RUMBLE ID %d out of bounds; ignoring!", (*it).id);
			}else{
				if((*it).intensity > 0){
					setRumble((*it).intensity);
				}
			}
		}else{
			ROS_WARN("Unknown JoyFeedback command; ignored");
		}
	}
}

/* Set rumble for duration in seconds */
void WiimoteNode::setRumble(double duration){
	// compute rumbleEnd
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
		unsigned int ret = xwii_iface_get_led(iface_, XWII_LED(i+1), &stateLed);
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
void WiimoteNode::setLed(unsigned int ledIdx, bool on){
	ROS_INFO_THROTTLE(1, "led(led #%i: %i)", XWII_LED(ledIdx+1), on);
	unsigned int ret = xwii_iface_set_led(iface_, XWII_LED(ledIdx+1),on);
	if (ret) {
		ROS_ERROR("Error: Cannot set led #%i: %d", XWII_LED(ledIdx+1), ret);
	}
}

/* Publish joy */
void WiimoteNode::publishJoy(){
	sensor_msgs::Joy joyData;

	joyData.header.stamp = ros::Time::now();
	
	for(int i = 0; i < 11; i++){
		joyData.axes.push_back(acceleration_[i]);
	}
	
	for(int i = 0; i < 11; i++){
		joyData.buttons.push_back(buttons_[i]);
	}

	joyPub_.publish(joyData);
}

/* Publish state */
void WiimoteNode::publishWiimoteState(){
	xwiimote_controller::State state;
	state.header.stamp = ros::Time::now();
	
	for(int i = 0; i < 11; i++){
		state.buttons[i] = buttons_[i];
	}
	
	for(int i = 0; i < 4; i++){
		state.LEDs[i] = leds_[i];
	}

	state.linear_acceleration.x = acceleration_[0] * EARTH_GRAVITY_;
	state.linear_acceleration.y = acceleration_[1] * EARTH_GRAVITY_;
	state.linear_acceleration.z = acceleration_[2] * EARTH_GRAVITY_;
	
	if(isPresentNunchuk()){
		for(int i = 0; i < 2; i++){
			state.nunchuk_buttons[i] = nunchukButtons_[i];
			state.nunchuk_joystick[i] = nunchukJoystick_[i];
		}
		state.nunchuk_acceleration.x = nunchuckAcceleration_[0] * EARTH_GRAVITY_;
		state.nunchuk_acceleration.y = nunchuckAcceleration_[1] * EARTH_GRAVITY_;
		state.nunchuk_acceleration.z = nunchuckAcceleration_[2] * EARTH_GRAVITY_;
	}
	
	if(isPresentMotionPlus()){
		state.angular_velocity.x = angularVelocity_[0];
		state.angular_velocity.y = angularVelocity_[1];
		state.angular_velocity.z = angularVelocity_[2];
	}
			
	state.rumble = rumbleState_;
	state.percent_battery = batteryPercent_;
	
	wiimoteStatePub_.publish(state);
}

/* Publish nunchuk */
void WiimoteNode::publishWiimoteNunchuk(){
	sensor_msgs::Joy wiimoteNunchukData;
	
	// Is the Nunchuk connected?
	if(isPresentNunchuk()){		
		// Is the Nunchuk publisher not advertised?
		if(nullptr == wiimoteNunchukPub_){
			wiimoteNunchukPub_ = nodePrivate_.advertise<sensor_msgs::Joy>("/wiimote/nunchuk", 1);			
		}
	}else{
		// Is the Nunchuk publisher advertised?
		if(nullptr != wiimoteNunchukPub_){
			wiimoteNunchukPub_.shutdown();
		}
	}
	
	wiimoteNunchukData.header.stamp = ros::Time::now();

	// Joy stick
	double stick[2];

	//calculateJoystickAxisXY(wiimote_state_.ext.nunchuk.stick, nunchuk_stick_min_, nunchuk_stick_max_, nunchuk_stick_center_, stick);

	wiimoteNunchukData.axes.push_back(nunchukJoystick_[0]);  // x
	wiimoteNunchukData.axes.push_back(nunchukJoystick_[1]);  // y

	wiimoteNunchukData.axes.push_back(nunchuckAcceleration_[0]);
	wiimoteNunchukData.axes.push_back(nunchuckAcceleration_[1]);
	wiimoteNunchukData.axes.push_back(nunchuckAcceleration_[2]);

	wiimoteNunchukData.buttons.push_back(nunchukButtons_[0]);
	wiimoteNunchukData.buttons.push_back(nunchukButtons_[1]);

	wiimoteNunchukPub_.publish(wiimoteNunchukData);	
}

/* Check is Nunchuk connected */
bool WiimoteNode::isPresentNunchuk(){	
	if(xwii_get_iface_name(XWII_IFACE_NUNCHUK) == NULL){
		return false;
	}else{
		return true;
	}
}

/* Check is Motion plus connected */
bool WiimoteNode::isPresentMotionPlus(){	
	if(xwii_get_iface_name(XWII_IFACE_MOTION_PLUS) == NULL){
		return false;
	}else{
		return true;
	}
}


/* Check an use factory calibration data */
void  WiimoteNode::checkFactoryCalibrationData(){
	/*int32_t xCal, yCal, zCal, factor;
	
	xwii_iface_get_mp_normalization(iface_, &xCal, &yCal, &zCal, &factor);
	if 
	 
	// Calibrate remote
	
	
	xCal = event.v.abs[0].x + xCal;
	yCal = event.v.abs[0].y + yCal;
	zCal = event.v.abs[0].z + zCal;
	xwii_iface_set_mp_normalization(iface_, xCal, yCal, zCal, factor);
	
	
	bool result = true;

	if(factor) == 0){
		if (wiimoteCalibrated_){
			ROS_WARN("Failed to read current Wiimote calibration data; proceeding with previous data");
		}else{
			ROS_ERROR("Failed to read Wiimote factory calibration data");
			result = false;
		}
	}else{
		// If any calibration point is zero; we fail
		if (!(xCal && yCal && zCal)){
			ROS_ERROR("Some Wiimote factory calibration data is missing; calibration failed");
			ROS_ERROR("Wiimote Cal:: [x]:%d, [y]:%d, [z]:%d", xCal, yCal, zCal);

			result = false;
		}else{
			wiimoteCalibrated_ = true;
			ROS_ERROR("Wiimote Cal:: [x]:%d, [y]:%d, [z]:%d", xCal, yCal, zCal);
		}
	}
	
	

  if (!getStateSample())
  {
    ROS_WARN("Could not read Wiimote state; nunchuk may not be calibrated if present.");
  }
  else
  {
    if (isPresentNunchuk())
    {
      if (wiimote_c::cwiid_get_acc_cal(wiimote_, wiimote_c::CWIID_EXT_NUNCHUK, &nunchuk_cal_) != 0)
      {
        if (nunchuk_calibrated_)
        {
          ROS_WARN("Failed to read current Nunchuk calibration data; proceeding with previous data");
        }
        else
        {
          ROS_ERROR("Failed to read Nunchuk factory calibration data");
          result = false;
          nunchuk_failed_calibration_ = true;
        }
      }
      else
      {
        // If any calibration point is zero; we fail
        if (!(nunchuk_cal_.zero[CWIID_X] && nunchuk_cal_.zero[CWIID_Y] && nunchuk_cal_.zero[CWIID_Z] &&
            nunchuk_cal_.one[CWIID_X] && nunchuk_cal_.one[CWIID_Y] && nunchuk_cal_.one[CWIID_Z]))
        {
          ROS_ERROR("Some Nunchuk factory calibration data is missing; calibration failed");
          ROS_ERROR("Nunchuk Cal:: zero[x]:%d, zero[y]:%d, zero[z]:%d,\n\tone[x]:%d, one[y]:%d, one[z]:%d",
              nunchuk_cal_.zero[CWIID_X], nunchuk_cal_.zero[CWIID_Y], nunchuk_cal_.zero[CWIID_Z],
              nunchuk_cal_.one[CWIID_X], nunchuk_cal_.one[CWIID_Y], nunchuk_cal_.one[CWIID_Z]);
          result = false;
          nunchuk_failed_calibration_ = true;
        }
        else
        {
          nunchuk_calibrated_ = true;
          ROS_DEBUG("Nunchuk Cal:: zero[x]:%d, zero[y]:%d, zero[z]:%d,\n\tone[x]:%d, one[y]:%d, one[z]:%d",
              nunchuk_cal_.zero[CWIID_X], nunchuk_cal_.zero[CWIID_Y], nunchuk_cal_.zero[CWIID_Z],
              nunchuk_cal_.one[CWIID_X], nunchuk_cal_.one[CWIID_Y], nunchuk_cal_.one[CWIID_Z]);
        }
      }
    }
  }

  if (wiimote_calibrated_)
  {
    // Save the current reporting mode
    uint8_t save_report_mode = wiimote_state_.rpt_mode;

    // Need to ensure we are collecting accelerometer
    uint8_t new_report_mode = save_report_mode | (CWIID_RPT_ACC | CWIID_RPT_EXT);

    if (new_report_mode != save_report_mode)
    {
      setReportMode(new_report_mode);
    }

    ROS_INFO("Collecting additional calibration data; keep wiimote stationary...");

    StatVector3d linear_acceleration_stat_old = linear_acceleration_stat_;
    linear_acceleration_stat_.clear();
    StatVector3d angular_velocity_stat_old = angular_velocity_stat_;
    angular_velocity_stat_.clear();

    bool failed = false;
    bool data_complete = false;
    int wiimote_data_points = 0;
    int motionplus_data_points = 0;

    while (!failed && !data_complete)
    {
      if (getStateSample())
      {
        if (wiimote_data_points < COVARIANCE_DATA_POINTS_)
        {
          linear_acceleration_stat_.addData(
              wiimote_state_.acc[CWIID_X],
              wiimote_state_.acc[CWIID_Y],
              wiimote_state_.acc[CWIID_Z]);

          ++wiimote_data_points;
        }

        if (isPresentMotionplus())
        {
          if (motionplus_data_points < COVARIANCE_DATA_POINTS_)
          {
            // ROS_DEBUG("New MotionPlus data :%03d: PHI: %04d, THETA: %04d, PSI: %04d", motionplus_data_points,
            //     wiimote_state_.ext.motionplus.angle_rate[CWIID_PHI],
            //     wiimote_state_.ext.motionplus.angle_rate[CWIID_THETA],
            //     wiimote_state_.ext.motionplus.angle_rate[CWIID_PSI]);
            angular_velocity_stat_.addData(
                wiimote_state_.ext.motionplus.angle_rate[CWIID_PHI],
                wiimote_state_.ext.motionplus.angle_rate[CWIID_THETA],
                wiimote_state_.ext.motionplus.angle_rate[CWIID_PSI]);

            ++motionplus_data_points;
          }
        }
      }
      else
      {
        failed = true;
      }

      if (wiimote_data_points >= COVARIANCE_DATA_POINTS_)
      {
        if (!isPresentMotionplus())
        {
          data_complete = true;
        }
        else
        {
          if (motionplus_data_points >= COVARIANCE_DATA_POINTS_)
          {
            data_complete = true;
          }
        }
      }
    }

    if (!failed)
    {
      ROS_DEBUG("Calculating calibration data...");

      // Check the standard deviations > 1.0
      TVectorDouble stddev = linear_acceleration_stat_.getStandardDeviationRaw();
      bool is_bad_cal = false;
      std::for_each(std::begin(stddev), std::end(stddev), [&](const double d)  // NOLINT(build/c++11)
      {
        if (d > 1.0)
        {
          is_bad_cal = true;
          ROS_DEBUG("Wiimote standard deviation > 1.0");
        }
      });  // NOLINT(whitespace/braces)

      if (!is_bad_cal)
      {
        TVectorDouble variance = linear_acceleration_stat_.getVarianceScaled(EARTH_GRAVITY_);

        ROS_DEBUG("Variance Scaled x: %lf, y: %lf, z: %lf", variance.at(CWIID_X),
            variance.at(CWIID_Y), variance.at(CWIID_Z));

        linear_acceleration_covariance_[0] = variance.at(CWIID_X);
        linear_acceleration_covariance_[1] = 0.0;
        linear_acceleration_covariance_[2] = 0.0;

        linear_acceleration_covariance_[3] = 0.0;
        linear_acceleration_covariance_[4] = variance.at(CWIID_Y);
        linear_acceleration_covariance_[5] = 0.0;

        linear_acceleration_covariance_[6] = 0.0;
        linear_acceleration_covariance_[7] = 0.0;
        linear_acceleration_covariance_[8] = variance.at(CWIID_Z);
      }
      else
      {
        ROS_ERROR("Failed calibration; using questionable data for linear acceleration");

        linear_acceleration_stat_ = linear_acceleration_stat_old;
        angular_velocity_stat_ = angular_velocity_stat_old;

        result = false;
      }

      if (angular_velocity_stat_.size() == COVARIANCE_DATA_POINTS_)
      {
        // Check the standard deviations > 50.0
        TVectorDouble gyro_stddev = angular_velocity_stat_.getStandardDeviationRaw();
        std::for_each(std::begin(gyro_stddev), std::end(gyro_stddev), [&](const double d)  // NOLINT(build/c++11)
        {
          if (d > 50.0)
          {
            is_bad_cal = true;
            ROS_DEBUG("MotionPlus standard deviation > 50");
          }
        });  // NOLINT(whitespace/braces)

        if (!is_bad_cal)
        {
          TVectorDouble gyro_variance = angular_velocity_stat_.getVarianceScaled(GYRO_SCALE_FACTOR_);

          ROS_DEBUG("Gyro Variance Scaled x: %lf, y: %lf, z: %lf", gyro_variance.at(CWIID_PHI),
              gyro_variance.at(CWIID_THETA), gyro_variance.at(CWIID_PSI));

          angular_velocity_covariance_[0] = gyro_variance.at(CWIID_PHI);
          angular_velocity_covariance_[1] = 0.0;
          angular_velocity_covariance_[2] = 0.0;

          angular_velocity_covariance_[3] = 0.0;
          angular_velocity_covariance_[4] = gyro_variance.at(CWIID_THETA);
          angular_velocity_covariance_[5] = 0.0;

          angular_velocity_covariance_[6] = 0.0;
          angular_velocity_covariance_[7] = 0.0;
          angular_velocity_covariance_[8] = gyro_variance.at(CWIID_PSI);
        }
        else
        {
          ROS_ERROR("Failed calibration; using questionable data for angular velocity");

          angular_velocity_stat_ = angular_velocity_stat_old;

          result = false;
        }
      }
      else
      {
        resetMotionPlusState();
      }
    }

    if (failed)
    {
      ROS_ERROR("Failed calibration; using questionable data");
      result = false;
    }
    else
    {
      struct timespec state_tv;

      if (clock_gettime(CLOCK_REALTIME, &state_tv) == 0)
      {
        calibration_time_ = ros::Time::now();
      }
      else
      {
        ROS_WARN("Could not update calibration time.");
      }
    }

    // Restore the pre-existing reporting mode
    if (new_report_mode != save_report_mode)
    {
      setReportMode(save_report_mode);
    }
  }

  // Publish the initial calibration state
  std_msgs::Bool imu_is_calibrated_data;
  imu_is_calibrated_data.data = result;
  imu_is_calibrated_pub_.publish(imu_is_calibrated_data);*/
}

int main(int argc, char **argv) {
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
