/**
 *
 *   @author Rodrigo Munguia 2019
 *
 */


#include <iostream>

#include "libgpsmm.h"
#include <gps.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <cstring>


using namespace std;

// helper functions


struct GPSdata {
   int32_t lat;
   int32_t lon;
   int32_t alt;
   int32_t ns; // number of satellites used
   int32_t vel; // /* Speed over ground, meters/sec */
   int32_t cou;  //  /* Course made good (relative to true north) */
} ;


void* start_G_thread(void *args);

class GPS_capture
{

public:

	GPS_capture();
	//~Camera_capture();

    char active_status;

    void start();
	void stop();
    void start_GPS_thread(void);
    void handle_quit( int sig );
    void start_data_storing(char* directory);
    void stop_data_storing();

   void getdata(GPSdata &gpsd)
   {
   gpsd = gpsdata;
   };


private:

 GPSdata gpsdata;

 volatile bool Act_data_storing;  // secure variable for accesing from threads

 //  gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

 gpsmm *gps_rec;

//  gpsmm *gps_rec = new gpsmm;


   pthread_t thread_tid;

   void GPS_thread(void);

   char DataSetDir[100];
   int f_idx;
    ofstream fileGPS;



};
