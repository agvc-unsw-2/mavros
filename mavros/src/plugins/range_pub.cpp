/**
 * @brief IMU publish plugin
 * @file range_pub.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <cmath>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3.h>

namespace mavplugin {
/* Note: this coefficents before been inside plugin class,
 * but after #320 something broken and in resulting plugins.so
 * there no symbols for that constants.
 * That cause plugin loader failure.
 *
 * objdump -x plugins.so | grep MILLI
 */

//! Gauss to Tesla coeff
static constexpr double GAUSS_TO_TESLA = 1.0e-4;
//! millTesla to Tesla coeff
static constexpr double MILLIT_TO_TESLA = 1000.0;
//! millRad/Sec to Rad/Sec coeff
static constexpr double MILLIRS_TO_RADSEC = 1.0e-3;
//! millG to m/s**2 coeff
static constexpr double MILLIG_TO_MS2 = 9.80665 / 1000.0;
//! millBar to Pascal coeff
static constexpr double MILLIBAR_TO_PASCAL = 1.0e2;

static constexpr double RAD_TO_DEG = 180.0 / M_PI;


/**
 * @brief IMU data publication plugin
 */
class RangePlugin : public MavRosPlugin {
public:
	RangePlugin() :
		range_nh("~range"),
		uas(nullptr),
		field_of_view(0)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		// we rotate the data from the aircraft-frame to the base_link frame.
		// Additionally we report the orientation of the vehicle to describe the
		// transformation from the ENU frame to the base_link frame (ENU <-> base_link).
		// THIS ORIENTATION IS NOT THE SAME AS THAT REPORTED BY THE FCU (NED <-> aircraft)
		range_nh.param<std::string>("frame_id", frame_id, "base_link");
		range_nh.param("field_of_view", field_of_view, 1.0);	// rads

		range_pub = range_nh.advertise<sensor_msgs::Range>("", 5);

	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_DISTANCE_SENSOR, &RangePlugin::handle_distance),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_RANGEFINDER, &RangePlugin::handle_rangefinder),
		};
	}

private:
	ros::NodeHandle range_nh;
	UAS *uas;
	std::string frame_id;

	ros::Publisher range_pub;

	double field_of_view;

	/* -*- helpers -*- */

	/* -*- message handlers -*- */

	void handle_distance(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

		mavlink_distance_sensor_t distance_raw;
		mavlink_msg_distance_sensor_decode(msg, &distance_raw);

		int rosSensorType = 0;
		switch(distance_raw.type) {
			case MAV_DISTANCE_SENSOR_LASER:			rosSensorType = 2; break; // laser (value not specified by ROS)
			case MAV_DISTANCE_SENSOR_ULTRASOUND:	rosSensorType = sensor_msgs::Range::ULTRASOUND; break; // ultrasonic
			case MAV_DISTANCE_SENSOR_INFRARED:		rosSensorType = sensor_msgs::Range::INFRARED; break; // infrared
		}

		auto range_msg = boost::make_shared<sensor_msgs::Range>();

		// fill
		range_msg->header = uas->synchronized_header(frame_id, distance_raw.time_boot_ms);
		range_msg->field_of_view = field_of_view;
		range_msg->radiation_type = rosSensorType;
		range_msg->min_range = distance_raw.min_distance * 0.01;
		range_msg->max_range = distance_raw.max_distance * 0.01;
		range_msg->range = distance_raw.current_distance * 0.01;

		// publish
		range_pub.publish(range_msg);
	}

	// from ardupilot
	void handle_rangefinder(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

		mavlink_rangefinder_t distance_raw;
		mavlink_msg_rangefinder_decode(msg, &distance_raw);

		auto range_msg = boost::make_shared<sensor_msgs::Range>();

		// fill
		range_msg->header.frame_id = frame_id;
		range_msg->header.stamp = ros::Time::now();
		range_msg->field_of_view = 0.001;
		range_msg->radiation_type = 2;
		range_msg->min_range = 0;
		range_msg->max_range = 0;
		range_msg->range = distance_raw.distance;

		// publish
		range_pub.publish(range_msg);
	}

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::RangePlugin, mavplugin::MavRosPlugin)

