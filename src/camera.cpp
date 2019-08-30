/**
 *
 *   @author Rodrigo Munguia 2016
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "camera.h"

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Camera_capture::Camera_capture()
{

	active_status = 0;      // whether the camera thread is running

	thread_tid = 0; // write thread id

	f_idx = 0;  // initial frame index

    n_frame = 1;

	Act_data_storing = false; // data storing activate flag
	Act_single_frame_capturesecure = false;



    compression_params.push_back(IMWRITE_PXM_BINARY);
    compression_params.push_back(1);

   for(int i = 0; i < MaXnumImgs;++i) {
        // this is optional, preallocation so there's no allocation
        // during capture
        Frame f;
        //f.time = 0;
        f.data.create(320, 240, CV_8UC1);

        frames.push_back(f);
        //images[i].create(320, 240, CV_8UC1);
    }

}

void Camera_capture::start()
{
	int result;
    int cam;

    printf("Camera number:  ");
    cin >> cam;

    cap.open(cam);
    cap.set(CAP_PROP_FPS , 30);
    cap.set(CAP_PROP_FRAME_WIDTH,320);
    cap.set(CAP_PROP_FRAME_HEIGHT,240);

	// --------------------------------------------------------------------------
	//   CREATE THREAD
	// --------------------------------------------------------------------------

	printf("START CAMERA THREAD \n");

	result = pthread_create( &thread_tid, NULL, &start_thread, this );




}
// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Camera_capture::camera_thread(void)
{
	// signal startup
	active_status = 1;


    printf("activating CAMERA .... \n");




   //namedWindow("camera",1);

   for(;;)
    {

      Mat frame;

       struct timespec tpe;  // create struct timespec

       cap >> frame; // get a new frame from camera

       clock_gettime(CLOCK_REALTIME, &tpe); // get the current time

      // printf("%lu s, %lu ns\n", tpe.tv_sec,tpe.tv_nsec);

       Mat gray;

       // convert RGB image to gray
       cvtColor(frame, gray, COLOR_BGR2GRAY);



       if (Act_single_frame_capturesecure)
       {
           char file_name[100];

           sprintf(file_name, "data/%i.pgm", n_frame); // Create the name of the file: TimeStamp + file extension

           imwrite(file_name, gray, compression_params); // write frame to disk

          n_frame ++;
          Act_single_frame_capturesecure = false;
       }


       if  ((Act_data_storing ) && (f_idx < MaXnumImgs))
       {

           //gray.copyTo(images[f_idx]);
           gray.copyTo(frames[f_idx].data);
           frames[f_idx].time.tv_sec = tpe.tv_sec;
           frames[f_idx].time.tv_nsec = tpe.tv_nsec;
           f_idx++;
          // cout << f_idx << " ";
       }

      //  imwrite("image.pgm", gray, compression_params);
        //imwrite("image.jpg", frame);

        imshow("camera", frame );
        //imshow("edges", frame);
        if(waitKey(30) >= 0) break;
    }





	// signal end
	active_status = false;

	return;

}

void Camera_capture::single_frame_capture()
{
    Act_single_frame_capturesecure = true;
}


void Camera_capture::start_data_storing(char* directory)
{
   f_idx = 0;
   Act_data_storing = true;

   strcpy(DataSetDir,  directory);
   strcat(DataSetDir,"/");
   // create file for storing file list


	//char file_name[100];
    char dir_file_name[100];

    strcpy(dir_file_name,  DataSetDir);  // Copy the directory name in dir_file_name

	 strcat(dir_file_name, "frame_list.txt");

    fileCAM.open (dir_file_name);

   // cout << dir_file_name;



   //***********************
}
void Camera_capture::stop_data_storing(void)
{

   Act_data_storing = false;





   for(int i = 0; i < f_idx;++i) {
        // this is optional, preallocation so there's no allocation
        // during capture

        //frames[f_idx].time.tv_sec = tpe.tv_sec;
        //   frames[f_idx].time.tv_nsec = tpe.tv_nsec;

        // get time stamp in micro-seconds (us) *******
        // the 4 least signifficative numbers from tv_nsec are used fo time stamp
        // therefore the max lengh dataset can cover a capture of 2.7 hrs (9999 seconds) aprox

        long us;
        us = frames[i].time.tv_nsec/1000;

        long sec = frames[i].time.tv_sec - (frames[i].time.tv_sec/10000)*10000;
        //printf("%lu s\n", sec);

        unsigned long time_in_micros = (1000000 * sec) + us;

        // ***************************************************
        // write frame to disk

        char file_name[100];
        char dir_file_name[100];

        strcpy(dir_file_name,  DataSetDir);  // Copy the directory name in dir_file_name


        sprintf(file_name, "%lu.pgm", time_in_micros); // Create the name of the file: TimeStamp + file extension

        strcat(dir_file_name, file_name); // directory name + file name

        //cout << file_name;
        imwrite(dir_file_name, frames[i].data, compression_params); // write frame to disk

        //*****************************************************

        fileCAM << file_name << "\n";


    }


    fileCAM.close();


}


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Function
// ------------------------------------------------------------------------------
void* start_thread(void *args)
{
	// takes an autopilot object argument
	Camera_capture *camera_capture = (Camera_capture *)args;

	// run the object's read thread

	camera_capture->start_camera_thread();

	// done!
	return NULL;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Camera_capture::start_camera_thread(void)
{

	if ( not active_status == false )
	{
		fprintf(stderr,"camera thread already running\n");
		return;
	}

	else
	{
		camera_thread();
		return;
	}

}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void Camera_capture::stop()
{


	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE CAMERA THREAD\n");

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
void Camera_capture::handle_quit( int sig )
{



	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop camera capture\n");
	}

}
