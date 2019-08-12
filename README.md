# MAVdataCapture
This program uses MAVlink and OpenCV libraries for receiving data and video from a Quadrotor equiped with ardupilot and camera/video-transmitter.  Also capture local GPS readings.

--------------------------------------------------------------------
DataCapture 
--------------------------------------------------------------------

This program uses MAVlink and OpenCV libraries for receiving data and video from a Quadrotor equiped with ardupilot and camera/video-transmitter. 
Also capture local GPS readings.


The program rely on the following libraries for compiling:

- mavlink c library
- OpenCV
- libgps-dev




- Set the correct port of telemetry (Drone) in mavlink_control.cpp.     (APM Planner 2)

- DataCapture uses the gpsd daemon for monitoring the messages of the GPS attached to USB port.  So check that Messages are correctly been received from gpds: 


If gpds seem to be not working propertly:
-- Restart gpds ---------------------------------------------

sudo pkill gpsd

sudo dpkg-reconfigure gpsd

sudo systemctl stop gpsd.socket

sudo systemctl disable gpsd.socket

If gpsd not negotiate the right  baud rate, do the following:
sudo stty -F /dev/ttyUSB0 38400
gpsd /dev/ttyUSB0

------------------
check gps messages

gpspipe -r
gpspipe -w

minicom

xgps


-----------------------------
Installar librerias para uso de gpsd:

sudo apt-get install libgps-dev

codigo ejemplo: 

https://github.com/jcable/gpsd/blob/master/test_gpsmm.cpp

https://gist.github.com/elimpnick/8313815ac387e6757f751dc8960f03d7

---------------------------------

CodeBlocks issue: Run in terminal but not in codeblocks "ld_library_path error"

In Project -> Project build options -> Search directories -> Linker

Select "Use project options only" in Policy

----------------------------------
github
project link:
https://github.com/rodrigo-munguia/MAV_DataCapture.git

info:
https://www.itwriting.com/blog/11410-adding-a-visual-studio-code-workspace-to-a-github-repository.html








