/**
 *
 *   @author Rodrigo Munguia 2019
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "gps_capture.h"


// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
GPS_capture::GPS_capture()
{

	active_status = 0;      // whether the camera thread is running

	thread_tid = 0; // write thread id



	Act_data_storing = false; // data storing activate flag



}

void GPS_capture::start()
{
	int result;


    gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);

    if (gps_rec->stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        cerr << "No GPSD running.\n";
        return;
    }



	// --------------------------------------------------------------------------
	//   CREATE THREAD
	// --------------------------------------------------------------------------

	printf("START GPS THREAD \n");

	result = pthread_create( &thread_tid, NULL, &start_G_thread, this );




}
// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void GPS_capture::GPS_thread(void)
{
	// signal startup
	active_status = 1;


    printf("activating GPS .... \n");




   //namedWindow("camera",1);

   for(;;)
    {
// do while thread is live


      struct gps_data_t* newdata;

        if (!gps_rec->waiting(5000000))
          continue;

        if ((newdata = gps_rec->read()) == NULL) {
            cerr << "Read error.\n";
            return;
        } else {



           gpsdata.lat = newdata->fix.latitude*10000000L ;
           gpsdata.lon = newdata->fix.longitude*10000000L ;
           gpsdata.alt = newdata->fix.altitude*1000L;
           gpsdata.vel = newdata->fix.speed*100L;;
           gpsdata.cou = newdata->fix.track*100L;
           gpsdata.ns = newdata->satellites_used;

          // fprintf(stdout, "LATLON: lat/lon/alt/vel/cou/sat: %i %i %i %i %i %i \n",lat,lon ,alt,vel,cou,ns);

           // libgps_dump_state(newdata);
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

                         fileGPS << time_in_micros << ","<< gpsdata.lat << "," << gpsdata.lon << ","<< gpsdata.alt << ","<< gpsdata.vel<< ","<<gpsdata.cou<< ","<<gpsdata.ns<<"\n";

                   }
                int q = 10;

        }




    }





	// signal end
	active_status = false;

	return;

}
// ------------------------------------------------------------------------------
//  Pthread Starter Helper Function
// ------------------------------------------------------------------------------
void* start_G_thread(void *args)
{
	// takes an autopilot object argument
	GPS_capture *gps_capture = (GPS_capture *)args;

	// run the object's read thread

	gps_capture->start_GPS_thread();

	// done!
	return NULL;
}
// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void GPS_capture::start_GPS_thread(void)
{

	if ( not active_status == false )
	{
		fprintf(stderr,"GPS thread already running\n");
		return;
	}

	else
	{
		GPS_thread();
		return;
	}

}
// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void GPS_capture::stop()
{


	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE GPS THREAD\n");

	// signal exit
	//time_to_exit = true;

	// wait for exit
	pthread_join(thread_tid,NULL);

    //usleep(10000);

	// now the read and write threads are closed
	printf("\n");

     // cap.release();
	// still need to close the serial_port separately
}
// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void GPS_capture::handle_quit( int sig )
{



	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop GPS capture\n");
	}

}
// ------------------------------------------------------------------------------
//   Start data storing
// ------------------------------------------------------------------------------
void GPS_capture::start_data_storing(char* directory)
{
   f_idx = 0;
   Act_data_storing = true;

   strcpy(DataSetDir,  directory);
   strcat(DataSetDir,"/");
   // create file for storing file list


	//char file_name[100];
    char dir_file_name[100];

    strcpy(dir_file_name,  DataSetDir);  // Copy the directory name in dir_file_name

	 strcat(dir_file_name, "GPS2_raw.txt");

    fileGPS.open (dir_file_name);

   // cout << dir_file_name;



   //***********************
}
// ------------------------------------------------------------------------------
//   Stop data storing
// ------------------------------------------------------------------------------
void GPS_capture::stop_data_storing(void)
{

   Act_data_storing = false;





    fileGPS.close();


}

