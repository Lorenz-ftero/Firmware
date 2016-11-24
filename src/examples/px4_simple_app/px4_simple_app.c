/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* advertise attitude topic */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
                { .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				att.q[0] = raw.accelerometer_m_s2[0];
				att.q[1] = raw.accelerometer_m_s2[1];
				att.q[2] = raw.accelerometer_m_s2[2];

				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
        }

        warnx("this is warnx");
        

        struct vehicle_control_mode_s			_vcontrol_mode;

        //_vcontrol_mode.flag_control_retraction_phase_enabled=true;

        int _vcontrol_mode_sub = -1;
        _vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
        orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);


        if(_vcontrol_mode.flag_armed){
            PX4_INFO("vehicle is armed");
        }else{
            PX4_INFO("vehicle is NOT armed");
        }

        if(_vcontrol_mode.flag_control_acceleration_enabled){
            PX4_INFO("control mode is control acceleration");
        }else{
            PX4_INFO("control mode is NOT control acceleration");
        }

        if(_vcontrol_mode.flag_control_altitude_enabled){
            PX4_INFO("control mode is altitude control");
        }else{
            PX4_INFO("control mode is NOT altitude control");
        }

        if(_vcontrol_mode.flag_control_attitude_enabled){
            PX4_INFO("control mode is attitude control");
        }else{
            PX4_INFO("control mode is NOT attitude control");
        }

        if(_vcontrol_mode.flag_control_auto_enabled){
            PX4_INFO("control mode is auto control");
        }else{
            PX4_INFO("control mode is NOT auto control");
        }

        if(_vcontrol_mode.flag_control_climb_rate_enabled){
            PX4_INFO("control mode is climb rate control");
        }else{
            PX4_INFO("control mode is NOT climb rate control");
        }

        if(_vcontrol_mode.flag_control_force_enabled){
            PX4_INFO("control mode is force control");
        }else{
            PX4_INFO("control mode is NOT force control");
        }

        if(_vcontrol_mode.flag_control_manual_enabled){
            PX4_INFO("control mode is manual control");
        }else{
            PX4_INFO("control mode is NOT manual control");
        }

        if(_vcontrol_mode.flag_control_offboard_enabled){
            PX4_INFO("control mode is offboard control");
        }else{
            PX4_INFO("control mode is NOT offboard control");
        }

        if(_vcontrol_mode.flag_control_position_enabled){
            PX4_INFO("control mode is position control");
        }else{
            PX4_INFO("control mode is NOT position control");
        }

        if(_vcontrol_mode.flag_control_rates_enabled){
            PX4_INFO("control mode is rates control");
        }else{
            PX4_INFO("control mode is NOT rates control");
        }

        if(_vcontrol_mode.flag_control_rattitude_enabled){
            PX4_INFO("control mode is rattitude control");
        }else{
            PX4_INFO("control mode is NOT rattitude control");
        }

        if(_vcontrol_mode.flag_control_termination_enabled){
            PX4_INFO("control mode is termination control");
        }else{
            PX4_INFO("control mode is NOT termination control");
        }

        if(_vcontrol_mode.flag_control_velocity_enabled){
            PX4_INFO("control mode is velocity control");
        }else{
            PX4_INFO("control mode is NOT velocity control");
        }

        if(_vcontrol_mode.flag_external_manual_override_ok){
            PX4_INFO("control mode is external manual override");
        }else{
            PX4_INFO("control mode is NOT external manual override");
        }

        if(_vcontrol_mode.flag_system_hil_enabled){
            PX4_INFO("control mode is hil system");
        }else{
            PX4_INFO("control mode is NOT hil system");
        }
        /*

        if(_vcontrol_mode.flag_control_traction_phase_enabled){
            PX4_INFO("control mode is tractionphase");
        }else{
            PX4_INFO("control mode is NOT tractionphase");
        }

        if(_vcontrol_mode.flag_control_retraction_phase_enabled){
            PX4_INFO("control mode is retractionphase");
        }else{
            PX4_INFO("control mode is NOT retractionphase");
        }

        */

        PX4_INFO("exiting");

	return 0;
}
