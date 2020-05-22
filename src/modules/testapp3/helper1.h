#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include <airspeed/airspeed.h>
#include <commander/px4_custom_mode.h>
#include <conversion/rotation.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_hrt.h>
#include <ecl/geo/geo.h>
#include <systemlib/px4_macros.h>

#ifdef CONFIG_NET
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

#ifndef __PX4_POSIX
#include <termios.h>
#endif

//#include <uORB/topics/offboard_control_mode.h>
//#include <uORB/topics/vehicle_control_mode.h>

//#include <testapp3.h>
//#include <sample.h>

/////////////////////////////////////////////////////////////////////////////
//COPIED HEADER FILE
/////////////////////////////////////////////////////////////////////////////
//#include "mavlink_ftp.h"
//#include "mavlink_log_handler.h"
//#include "mavlink_mission.h"
//#include "mavlink_parameters.h"
//#include "mavlink_timesync.h"
//#include "tune_publisher.h"


#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cellular_status.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/onboard_computer_status.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/ping.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_bezier.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>

#include <uORB/topics/sensor_combined.h>

class Mavlink;

//class MavlinkReceiver : public ModuleParams
//{
//public:
	//MavlinkReceiver(Mavlink *parent);
	//~MavlinkReceiver() override;

/**
 * Start the receiver thread
 */
//static void receive_start(pthread_t *thread, Mavlink *parent);

//static void *start_helper(void *context);



//void acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result);

/**
 * Common method to handle both mavlink command types. T is one of mavlink_command_int_t or mavlink_command_long_t.
 */


Mavlink				*_mavlink;



// ORB publications
uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
uORB::Publication<position_setpoint_triplet_s>		_pos_sp_triplet_pub{ORB_ID(position_setpoint_triplet)};



// ORB subscriptions
uORB::Subscription	_control_mode_sub{ORB_ID(vehicle_control_mode)};

uORB::PublicationQueued<vehicle_command_s>	_cmd_pub{ORB_ID(vehicle_command)};

void testfunction(void);






///////////////////////////////////////////////////////////////////////////////////







#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif
