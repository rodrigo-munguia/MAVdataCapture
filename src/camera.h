/**
 *
 *   @author Rodrigo Munguia 2016
 *
 */


//#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <fstream>
//#include <atomic>

using namespace cv;
using namespace std;


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Frame
{

  Mat data;

   struct timespec time;

};


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// helper functions

void* start_thread(void *args);

const int MaXnumImgs = 30*60*10;

class Camera_capture
{

public:

	Camera_capture();
	//~Camera_capture();

    char active_status;

    void start();
	void stop();
    void start_camera_thread(void);
    void handle_quit( int sig );
    void start_data_storing(char* directory);
    void stop_data_storing();
    void single_frame_capture();

private:

    vector<Frame> frames;

    vector<int> compression_params;

    int f_idx;

 volatile bool Act_data_storing;  // secure variable for accesing from threads

 volatile bool Act_single_frame_capturesecure;  // secure variable for accesing from threads

    VideoCapture cap;

    Mat images[MaXnumImgs];

   pthread_t thread_tid;

   void camera_thread(void);

   char DataSetDir[100];

   ofstream fileCAM;

   int n_frame;

};
