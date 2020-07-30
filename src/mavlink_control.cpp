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
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
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

#include "mavlink_control.h"


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
	//char *uart_name = (char*)"/dev/ttyUSB0";   //  check port of telemetry
	char *uart_name = (char*)"/dev/ttyUSB0";   //  check port of telemetry
	int baudrate = 57600;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port);



	 // Create camera capture object
    Camera_capture camera_capture;

    // Create GPS capture object

    GPS_capture gps_capture;

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	camera_capture_quit = &camera_capture;
	gps_capture_quit = &gps_capture;

	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */
  // XXxx

     int option;
     string Soption = "";

     cout << "1) Camera  " << endl;
     cout << "2) Drone Data " << endl;
     cout << "3) Local GPS " << endl;
     cout << "4) Camera + Drone Data  " << endl;
     cout << "5) Camera + Drone Data + Local GPS " << endl;
     cout << "6) Camera + Local GPS" << endl;
     cout << "7) Drone Data + Local GPS " << endl;
     printf("\n");
     cout << "Please select an option : ";

     getline(cin, Soption);
     option = std::atoi(Soption.c_str());

     if(option == 1) // Checking if user selected option 1
     {

      camera_capture.start();

     }
     else if(option == 2) // Checking if user selected option 2
     {

      serial_port.start();
      autopilot_interface.start();

     }
     else if(option == 3) // Checking if user selected option 3
     {

      gps_capture.start();
     }
     else if(option == 4) // Checking if user selected option 4
     {
        camera_capture.start();
        serial_port.start();
        autopilot_interface.start();

     }
     else if(option == 5) // Checking if user selected option 4
     {

        camera_capture.start();
        serial_port.start();
        autopilot_interface.start();
        gps_capture.start();

     }
     else if(option == 6) // Checking if user selected option 4
     {

     }
     else if(option == 7) // Checking if user selected option 4
     {

     }
     else //if user has entered invalid choice (other than 1,2,3 or 4)
     {

     }






    // Start camera capture object


	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */
    sleep(1);
	commands(autopilot_interface,camera_capture,gps_capture);



	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	 * Now that we are done we can stop the threads and close the port
	 */
     autopilot_interface.stop();
     serial_port.stop();

    camera_capture.stop();

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void commands(Autopilot_Interface &api, Camera_capture &camera_cap, GPS_capture &gps_cap )
{

  int option;
  string Soption = "";

  do //do-while loop starts here.that display menu again and again until user select to exit program
  {

     Mavlink_Messages messages = api.current_messages;

    // mavlink_global_position_int_t pos2 = messages.global_position_int;
      mavlink_gps_raw_int_t gpsr = messages.gps_raw;
      mavlink_scaled_pressure_t SP = messages.scaled_pressure;
      mavlink_rangefinder_t RF = messages.rangefinder;
     //printf("Got message GLOBAL_POSITION_int (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
     printf("\n");
    // printf("current GPS position  (int):  %.i %i %i (m)\n", pos2.lat, pos2.lon, pos2.alt,  );
    printf("MAVlink data --------------------------\n");
    printf("pos  (int):  %i %i %i (m) sat: %i ", gpsr.lat, gpsr.lon, gpsr.alt , gpsr.satellites_visible );
    printf("\n");
    printf("RangeFinder :  distance: %f (m) voltage: %f (v) ", RF.distance, RF.voltage );
    printf("\n");
    printf("Pressure :  absolute: %f (hp) temperature: %i (deg) ", SP.press_abs, SP.temperature );
    printf("\n");

    GPSdata gps2;
    gps_cap.getdata(gps2);

    printf("GPS local data ------------------------\n");
    //fprintf(stdout, "GPS local: lat/lon/alt/vel/cou/sat: %i %i %i %i %i %i \n",gps2.lat,gps2.lon ,gps2.alt,gps2.vel,gps2.cou,gps2.ns);
    fprintf(stdout, " lat/lon/alt/sat: %i %i %i %i \n \n",gps2.lat,gps2.lon ,gps2.alt,gps2.ns);

     //mavlink_attitude_t imu2 = messages.attitude;
     //printf("Got message Attitude (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
     //printf("    Att  :  %f %f %f (deg)\n", imu2.roll , imu2.pitch, imu2.yaw);




     //Displaying Options for the menu
     cout << "1) Start Data Capture " << endl;
     cout << "2) Stop Data Capture " << endl;
     cout << "3) Capture single frame " << endl;
     cout << "Other Key Number) Refresh" << endl;
     cout << "Ctrl + c) Exit Program " << endl;
     //Prompting user to enter an option according to menu
     printf("\n");
     cout << "Please select an option : ";



    time_t t = time(NULL);
	tm* timePtr = localtime(&t);
	/*
	cout << "seconds= " << timePtr->tm_sec << endl;
  cout << "minutes = " << timePtr->tm_min << endl;
  cout << "hours = " << timePtr->tm_hour << endl;
  cout << "day of month = " << timePtr->tm_mday << endl;
  cout << "month of year = " << timePtr->tm_mon << endl;
  cout << "year = " << timePtr->tm_year << endl;
  cout << "weekday = " << timePtr->tm_wday << endl;
  cout << "day of year = " << timePtr->tm_yday << endl;
  */



     //cin.clear();
     //cin.ignore(numeric_limits<streamsize>::max(), '\n');
    // cin.ignore(256,'\n');

    // cin >> option;  // taking option value as input and saving in variable "option"
     getline(cin, Soption);
     option = std::atoi(Soption.c_str());



     if(option == 1) // Checking if user selected option 1
     {

         // start data capturing

        char dir[100];
        sprintf(dir, "data/%d-%d-%d-%d-%d", timePtr->tm_year+1900,timePtr->tm_mon,timePtr->tm_mday,timePtr->tm_hour,timePtr->tm_min );
        mkdir(dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        //cout << dir;
        camera_cap.start_data_storing(dir);
        api.start_data_storing(dir);
        gps_cap.start_data_storing(dir);

     }
     else if(option == 2) // Checking if user selected option 2
     {
         // stop data capturing
         api.stop_data_storing();
         camera_cap.stop_data_storing();
         gps_cap.stop_data_storing();

     }
     else if(option == 3) // Checking if user selected option 3
     {
        camera_cap.single_frame_capture();

     }
     else if(option == 4) // Checking if user selected option 4
     {
       //cout << "Terminating Program" << endl;
     }
     else //if user has entered invalid choice (other than 1,2,3 or 4)
     {
       //Displaying error message
       //cout << "Invalid Option entered" << endl;
     }
  }
  while(option != 99);  //condition of do-while loop


  /*
    for (int i = 1; i <1 ; i++)
   {
   // copy current messages
   Mavlink_Messages messages = api.current_messages;

   mavlink_global_position_int_t pos2 = messages.global_position_int;
    printf("Got message GLOBAL_POSITION_int (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
    printf("    pos  (int):  %.i %i %i (m)\n", pos2.lat, pos2.lon, pos2.alt );



   mavlink_attitude_t imu2 = messages.attitude;
   printf("Got message Attitude (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
   printf("    Att  :  %f %f %f (deg)\n", imu2.roll , imu2.pitch, imu2.yaw);


	printf("\n");

    usleep(500000); // check at 20Hz

    }
	*/
	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// camera
	try {
		camera_capture_quit->handle_quit(sig);
	}
	catch (int error){}

	// gps
	try {
		gps_capture_quit->handle_quit(sig);
	}
	catch (int error){}


	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		// end program here

		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


