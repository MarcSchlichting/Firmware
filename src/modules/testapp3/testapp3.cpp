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



#include <helper1.h>




///////////////////////////////////////////////////////////////////////////////////







#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif

//using matrix::wrap_2pi;

extern "C" __EXPORT void testapp3_main(int argc, char *argv[]);

void testapp3_main(int argc, char *argv[])
{
    PX4_INFO("testapp3 started");

    int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_global_position));
	/* limit the update rate to 10 Hz */
	orb_set_interval(sensor_sub_fd, 100);


	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};


    printf("Arming of Vehicle Started!\n");
    vehicle_command_s vcmd{};

    vcmd.timestamp = hrt_absolute_time();

    vcmd.param1 = 1;
    vcmd.param2 = NAN;
    vcmd.param3 = NAN;
    vcmd.param4 = NAN;
    vcmd.param5 = (double)NAN;
    vcmd.param6 = (double)NAN;
    vcmd.param7 = NAN;
    vcmd.command = 400;
    vcmd.target_system = 1;
    vcmd.target_component = 1;
    vcmd.source_system = 190;
    vcmd.source_component = 191;
    vcmd.confirmation = 0;
    vcmd.from_external = true;

    _cmd_pub.publish(vcmd);
    printf("Arming of Vehicle should be completed!\n");
    testfunction();


    offboard_control_mode_s offboard_control_mode{};

    offboard_control_mode.ignore_position=1;
    offboard_control_mode.ignore_alt_hold=1;
    offboard_control_mode.ignore_velocity=0;
    offboard_control_mode.ignore_acceleration_force=1;
    offboard_control_mode.ignore_attitude=1;
    offboard_control_mode.ignore_bodyrate_x=1;
    offboard_control_mode.ignore_bodyrate_y=1;
    offboard_control_mode.ignore_bodyrate_z=1;

    bool is_force_sp = 0;
    bool is_takeoff_sp = 0;
    bool is_land_sp = 0;
    bool is_loiter_sp = 0;
    bool is_idle_sp = 0;


    offboard_control_mode.timestamp = hrt_absolute_time();
    _offboard_control_mode_pub.publish(offboard_control_mode);

    //if (_mavlink->get_forward_externalsp()) {

			vehicle_control_mode_s control_mode{};
			_control_mode_sub.copy(&control_mode);

			if (control_mode.flag_control_offboard_enabled) {
				if (is_force_sp && offboard_control_mode.ignore_position &&
				    offboard_control_mode.ignore_velocity) {

					PX4_WARN("force setpoint not supported");

				} else {
					/* It's not a pure force setpoint: publish to setpoint triplet  topic */
					position_setpoint_triplet_s pos_sp_triplet{};

					pos_sp_triplet.timestamp = hrt_absolute_time();
					pos_sp_triplet.previous.valid = false;
					pos_sp_triplet.next.valid = false;
					pos_sp_triplet.current.valid = true;

					/* Order of statements matters. Takeoff can override loiter.
					 * See https://github.com/mavlink/mavlink/pull/670 for a broader conversation. */
					if (is_loiter_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					} else if (is_takeoff_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

					} else if (is_land_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

					} else if (is_idle_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

					} else {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					}

					/* set the local pos values */

					pos_sp_triplet.current.position_valid = false;


					/* set the local vel values */

					pos_sp_triplet.current.velocity_valid = true;
					pos_sp_triplet.current.vx = 0;
					pos_sp_triplet.current.vy = 0;
					pos_sp_triplet.current.vz = 0;

					pos_sp_triplet.current.velocity_frame = 8;


					pos_sp_triplet.current.alt_valid = false;

					/* set the local acceleration values if the setpoint type is 'local pos' and none
					 * of the accelerations fields is set to 'ignore' */

					pos_sp_triplet.current.acceleration_valid = false;


					/* set the yaw sp value */
					pos_sp_triplet.current.yaw_valid = false;


					/* set the yawrate sp value */

					pos_sp_triplet.current.yawspeed_valid = false;


					//XXX handle global pos setpoints (different MAV frames)
					_pos_sp_triplet_pub.publish(pos_sp_triplet);
				}
			}


















    printf("Starting Offboard Mode\n");


    vcmd.timestamp = hrt_absolute_time();

    vcmd.param1 = 1;
    vcmd.param2 = 6;
    vcmd.param3 = 0;
    vcmd.param4 = NAN;
    vcmd.param5 = (double)NAN;
    vcmd.param6 = (double)NAN;
    vcmd.param7 = NAN;
    vcmd.command = 176;
    vcmd.target_system = 1;
    vcmd.target_component = 1;
    vcmd.source_system = 190;
    vcmd.source_component = 191;
    vcmd.confirmation = 0;
    vcmd.from_external = true;

    _cmd_pub.publish(vcmd);

    printf("Finished Starting Offboard Mode\n");


	for (int i = 0; i < 100; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

        if(poll_ret>0){

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_global_position_s raw;
                orb_copy(ORB_ID(vehicle_global_position), sensor_sub_fd, &raw);
				/* copy sensors raw data into local buffer */
				//offboard_control_mode_s offboard_control_mode{};

    offboard_control_mode.ignore_position=1;
    offboard_control_mode.ignore_alt_hold=1;
    offboard_control_mode.ignore_velocity=0;
    offboard_control_mode.ignore_acceleration_force=1;
    offboard_control_mode.ignore_attitude=1;
    offboard_control_mode.ignore_bodyrate_x=1;
    offboard_control_mode.ignore_bodyrate_y=1;
    offboard_control_mode.ignore_bodyrate_z=1;

    is_force_sp = 0;
    is_takeoff_sp = 0;
    is_land_sp = 0;
    is_loiter_sp = 0;
    is_idle_sp = 0;


    offboard_control_mode.timestamp = hrt_absolute_time();
    _offboard_control_mode_pub.publish(offboard_control_mode);

    //if (_mavlink->get_forward_externalsp()) {

			//vehicle_control_mode_s control_mode{};
			_control_mode_sub.copy(&control_mode);

			if (control_mode.flag_control_offboard_enabled) {
				if (is_force_sp && offboard_control_mode.ignore_position &&
				    offboard_control_mode.ignore_velocity) {

					PX4_WARN("force setpoint not supported");

				} else {
					/* It's not a pure force setpoint: publish to setpoint triplet  topic */
					position_setpoint_triplet_s pos_sp_triplet{};

					pos_sp_triplet.timestamp = hrt_absolute_time();
					pos_sp_triplet.previous.valid = false;
					pos_sp_triplet.next.valid = false;
					pos_sp_triplet.current.valid = true;

					/* Order of statements matters. Takeoff can override loiter.
					 * See https://github.com/mavlink/mavlink/pull/670 for a broader conversation. */
					if (is_loiter_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					} else if (is_takeoff_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

					} else if (is_land_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

					} else if (is_idle_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

					} else {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					}

					/* set the local pos values */

					pos_sp_triplet.current.position_valid = false;


					/* set the local vel values */

					pos_sp_triplet.current.velocity_valid = true;
					pos_sp_triplet.current.vx = 0;
					pos_sp_triplet.current.vy = 0;
					pos_sp_triplet.current.vz = -1;

					pos_sp_triplet.current.velocity_frame = 8;


					pos_sp_triplet.current.alt_valid = false;

					/* set the local acceleration values if the setpoint type is 'local pos' and none
					 * of the accelerations fields is set to 'ignore' */

					pos_sp_triplet.current.acceleration_valid = false;


					/* set the yaw sp value */
					pos_sp_triplet.current.yaw_valid = false;


					/* set the yawrate sp value */

					pos_sp_triplet.current.yawspeed_valid = false;


					//XXX handle global pos setpoints (different MAV frames)
					_pos_sp_triplet_pub.publish(pos_sp_triplet);
				}
			}


			}
    }

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */

	}

	PX4_INFO("exiting");


}
