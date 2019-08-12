/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
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
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */
 /**
 *   Modifications for data storing, video storing, and local gps data storing by
 *   @author Rodrigo Munguia 2016,2019
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}



// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

	Act_data_storing = false; // R. Munguia 2015

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	current_setpoint = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( not received_all and not time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				//----------------------------------------------------------------------------------------
				case MAVLINK_MSG_ID_SCALED_PRESSURE:
				{
                    mavlink_msg_scaled_pressure_decode(&message, &(current_messages.scaled_pressure));
                    mavlink_scaled_pressure_t SP;
                    SP = current_messages.scaled_pressure;
                    // cout << SP.press_abs << endl;   /*< Absolute pressure (hectopascal) (float)*/
                   // cout << SP.temperature << endl;    /*< Temperature measurement  (int16_t) (0.01 degrees celsius)*/


                   struct timespec tpe;  // create struct timespec
                   clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
                   if (Act_data_storing )
                   {
                        long us;
                        us = tpe.tv_nsec/1000;
                        long sec = tpe.tv_sec - (tpe.tv_sec/10000)*10000;
                        unsigned long time_in_micros = (1000000 * sec) + us;

                         filePressureSensor << time_in_micros << "," << std::setprecision(10) << SP.press_abs << "," << (int)SP.temperature <<"\n";

                   }

                    break;
				}


                case MAVLINK_MSG_ID_RANGEFINDER:
                {
                    mavlink_msg_rangefinder_decode(&message, &(current_messages.rangefinder));
                    mavlink_rangefinder_t RF;
                    RF = current_messages.rangefinder;
                    //cout << RF.distance << endl;  /*< distance in meters  (float)  */
                    // cout << RF.voltage << endl;  /*< raw voltage if available, zero otherwise  (float)*/
                    struct timespec tpe;  // create struct timespec
                   clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
                   if (Act_data_storing )
                   {
                        long us;
                        us = tpe.tv_nsec/1000;
                        long sec = tpe.tv_sec - (tpe.tv_sec/10000)*10000;
                        unsigned long time_in_micros = (1000000 * sec) + us;

                         fileRangeSensor << time_in_micros <<"," << std::setprecision(6) << RF.distance << "," << RF.voltage <<"\n";

                   }

                    break;
                }


				case MAVLINK_MSG_ID_GPS_RAW_INT:
				{

                    mavlink_msg_gps_raw_int_decode(&message, &(current_messages.gps_raw));
                    current_messages.time_stamps.gps_raw = get_time_usec();
                    this_timestamps.gps_raw = current_messages.time_stamps.gps_raw;

                    mavlink_gps_raw_int_t gpsr = current_messages.gps_raw;
                    //printf("Got message GLOBAL_POSITION_int (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
                  // printf("pos  (int):  %.i %i %i (m) sat: %i  v: %i  c:%i\n", gpsr.lat, gpsr.lon, gpsr.alt , gpsr.satellites_visible, gpsr.vel , gpsr.cog  );

                   struct timespec tpe;  // create struct timespec
                   clock_gettime(CLOCK_REALTIME, &tpe); // get the current time

                   if (Act_data_storing )
                   {
                        long us;
                        us = tpe.tv_nsec/1000;
                        long sec = tpe.tv_sec - (tpe.tv_sec/10000)*10000;
                        unsigned long time_in_micros = (1000000 * sec) + us;

                        // gpsr.vel -> GPS ground speed
                        // gpsr.cog -> Course over ground (NOT heading, but direction of movement) in degrees * 100

                         fileGPS << time_in_micros << ","<< gpsr.lat << "," << gpsr.lon << ","<< gpsr.alt << ","<< (int)gpsr.vel<< ","<<(int)gpsr.cog<< ","<<(int)gpsr.satellites_visible<<"\n";

                   }
                    break;
				}

				//--------------------------------------------------------------------------------------------------------------------------------------

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;


                   // printf("GPS ID = %i\n ", message.compid);

                    //mavlink_global_position_int_t pos2 = current_messages.global_position_int;
                    //printf("Got message GLOBAL_POSITION_int (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
                    //printf("    pos  (int):  %.i %i %i (m)\n", pos2.lat, pos2.lon, pos2.alt );

					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
				this_timestamps.sys_status                 &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
				this_timestamps.position_target_global_int &&
				this_timestamps.highres_imu                &&
				this_timestamps.attitude                   ;

		// give the write thread time to use the port
		if ( writing_status > false )
			usleep(100); // look for components of batches at 10kHz

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}



// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( not serial_port->status == 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}




    //***********************************************************************
    // Request data from ARDUPILOT
    mavlink_request_data_stream_t com;
    com.target_system    = system_id;
    com.target_component = autopilot_id;
    //com.req_stream_id    = MAV_DATA_STREAM_POSITION;
    com.req_stream_id    = MAV_DATA_STREAM_ALL;
    //com.req_stream_id    =  MAV_DATA_STREAM_RAW_SENSORS;

    com.req_message_rate    = 10;
    com.start_stop          = 01; //

    // Encode
    mavlink_message_t message;
    //mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);
    mavlink_msg_request_data_stream_encode(system_id, companion_id,&message,&com);

    // Send the message
    int len = serial_port->write_message(message);
    //**************************************************************************


	// we need this before starting the write thread


    /*
	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");

     */
	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE MAVLINK THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{



	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( not time_to_exit )
	{
		read_messages();
		//usleep(100000); // Read batches at 10Hz
        usleep(50000); // Read batches at 20Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;


	//write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( not time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		//write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}

void Autopilot_Interface::start_data_storing(char* directory)
{
   //f_idx = 0;
   Act_data_storing = true;

   strcpy(DataSetDir,  directory);
   strcat(DataSetDir,"/");
   // create file for storing file list


	//char file_name[100];
    char dir_file_name[100];
    strcpy(dir_file_name,  DataSetDir);  // Copy the directory name in dir_file_name
    strcat(dir_file_name, "GPS_raw.txt");
    fileGPS.open (dir_file_name);

    char dir_file_name1[100];
    strcpy(dir_file_name1,  DataSetDir);  // Copy the directory name in dir_file_name
    strcat(dir_file_name1, "RangeSensor.txt");
    fileRangeSensor.open (dir_file_name1);

    char dir_file_name2[100];
    strcpy(dir_file_name2,  DataSetDir);  // Copy the directory name in dir_file_name
    strcat(dir_file_name2, "PressureSensor.txt");
    filePressureSensor.open (dir_file_name2);
   // cout << dir_file_name;



   //***********************
}
void Autopilot_Interface::stop_data_storing(void)
{

   fileGPS.close();
   fileRangeSensor.close();
   filePressureSensor.close();

}

