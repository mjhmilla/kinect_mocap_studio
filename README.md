# kinect_mocap_studio
A program that provides a preview of the Azure-Kinect's skeleton data and saves the coordinates to file

@author: Matthew Millard
@date  : 25 November 2021

# Quick Start: Recording data from the Kinect

1. Connect the Kinect:
  a. To power supply it comes with
  b. Via USB cable to the laptop
  
2. Place the Kinect at least 70 cm away from you. Point the Kinect towards you

3. Open a terminal in the folder: 
  /home/orb/Research/Kinect/code/Kinect-Projects/kinect_mocap_studio/build

4. Run
  ./kinect_mocap_studio --help

You should see

----------
  USAGE: 
  
     ./kinect_mocap_studio  [-s <double>] [-f <int>] [-c <string>] [-d
                            <string>] [-o <string>] [-w <bool>] [-i <string>]
                            [--] [--version] [-h]
  
  
  Where: 
  
     -s <double>,  --smoothing <double>
       Amount of temporal smoothing in the skeleton tracker (0-1)
  
     -f <int>,  --fps <int>
       Frames per second: 5, 15, 30
  
     -c <string>,  --color_mode <string>
       Color resolution: OFF, 720P, 1080P, 1440P, 1536P, 2160P, 3072P
  
     -d <string>,  --depth_mode <string>
       Depth mode: OFF, NFOV_2X2BINNED, NFOV_UNBINNED, WFOV_2X2BINNED,
       WFOV_UNBINNED, PASSIVE_IR
  
     -o <string>,  --outfile <string>
       Name of output file excluding the file extension
  
     -w <bool>,  --write <bool>
       Write sensor data to *.mkv file
  
     -i <string>,  --infile <string>
       Input sensor file of type *.mkv to process
  
     --,  --ignore_rest
       Ignores the rest of the labeled arguments following this flag.
  
     --version
       Displays version information and exits.
  
     -h,  --help
       Displays usage information and exits.
  
  
     kinect_mocap_studio is a command-line tool to recordvideo and skeletal
     data from the Azure-Kinect
  
----------
  
5. Run the kinect:
  ./kinect_mocap_studio -f 15 -c 720P -d WFOV_UNBINNED -o testData -w 1


You should see a view of the depth camera with the skeleton tracking overlaid on people within the frame (in orange-yellow). The following output should also appear in the terminal:

----------
  Basic Navigation:
  
   Rotate: Rotate the camera by moving the mouse while holding mouse left button
   Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button
   Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel
   Select Center: Center the scene based on a detected joint by right clicking the joint with mouse
  
   Key Shortcuts
  
   ESC: quit
   h: help
   b: body visualization mode
   k: 3d window layout
  
  depth_mode         :WFOV_UNBINNED
  color_resolution   :720P
  frames_per_second  :15
  temporal smoothing :0
  output file name   :testData_WFOV_UNBINNED_720P_15fps.json
  video file name    :testData_WFOV_UNBINNED_720P_15fps.mkv
----------

The *.mkv file contains the raw depth camera and RGB images. The *.json file contains the output of the skeleton tracker.

6. To stop the recording, make sure the viewer has the main focus and press the 'ESC' button.

# Recording many successive trials using the Kinect

If you run the kinect camera again using exactly the same command, and thus exactly the same output file name, the program will add an integer to the end of the name so that you don't overwrite the old results. For example:

  ./kinect_mocap_studio -f 15 -c 720P -d WFOV_UNBINNED -o testData -w 1

will produce files with the names

  testData_WFOV_UNBINNED_720P_15fps_0.json
  testData_WFOV_UNBINNED_720P_15fps_0.mkv

# Reprocessing the skeleton tracking of a saved file

If you would like to reprocess data that you have already collected, perhaps with a different smoothing parameter, you can do so using the -i flag:

  ./kinect_mocap_studio -f 15 -c 720P -d WFOV_UNBINNED -s 0.5 -i testData_WFOV_UNBINNED_720P_15fps_0.mkv

Upon calling this function a view of the recorded depth data will appear and the skeleton tracking will be overlaid in yellow-orange. However, this time the movements of the skeleton tracking will be smoothed by a factor of 0.5. These results will be written to a *.json file with the same name as the *.mkv file. If necessary a number will be appended to the file name to prevent filename collisions. In this case:

  testData_WFOV_UNBINNED_720P_15fps_0_0.json

See the Kinect online documentation for further details of how the smoothing algorithm works.

# Installation notes for the Kinect libraries on Ubuntu 20.04

The Kinect libraries are supported to run on Ubuntu 18.04. While it is possible to install it on Ubuntu 20.04 this takes some extra time and patience. The following steps should help.

1. To install the Azure-Kinect tools on Ubuntu 20.04 begin by following the instructions here, which are copied below:

https://feedback.azure.com/forums/920053-azure-kinect-dk/suggestions/40368301-support-for-ubuntu-20-04-lts

----------------------------------------
These are the steps I followed to install k4a-tools, libk4a and libk4abt on Ubuntu 20.04. The general steps are as outlined in
https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download, with a couple of hacks to make things work on 20.04.
- use of 18.04 repo, even though OS is 20.04
- installed lower versions of tools and libraries (as latest versions of sensor and body tracker don't seem to be compatible on 20.04)

$ curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
$ sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
$ curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft-prod.list
$ curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
$ sudo apt-get update
$ sudo apt install libk4a1.3-dev
$ sudo apt install libk4abt1.0-dev
$ sudo apt install k4a-tools=1.3.0

- Verify sensor library by launching camera viewer

$ k4aviewer
----------------------------------------

I have to run k4aviewer using sudo.

----------------------------------------
- Clone and build [Azure Kinect Samples](https://github.com/microsoft/Azure-Kinect-Samples).
- Run `simple_3d_viewer` to verify body tracker works.
----------------------------------------

I ran into all kinds of problems here.

Still cranking through:
  https://feedback.azure.com/forums/920053-azure-kinect-dk/suggestions/40368301-support-for-ubuntu-20-04-lts

https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md

2. Clone repo: 
	https://github.com/microsoft/Azure-Kinect-Samples
3. Clone submodules: 
	git submodule update --init --recursive
4. As instructed here try building:
	https://github.com/microsoft/Azure-Kinect-Samples	
	
	I had to (libk4abt was already installed):
	
	apt install libxi-dev
	
	Then I tried:
	
	mkdir build
	cd build
	cmake .. -GNinja
	ninja

	At this point git will complain as you are probably missing a bunch of libraries. Simply google the name of the offending library/package, and figure out what you need to install. This worked without a hitch.
	
5. Next simple_3d_viewer failed to build upon calling ninja:

-----------
[6/33] Building C object body-tracking-samples/extern/glfw/src/src/CMakeFiles/glfw.dir/linux_joystick.c.o
../body-tracking-samples/extern/glfw/src/src/linux_joystick.c: In function ‘_glfwInitJoysticksLinux’:
../body-tracking-samples/extern/glfw/src/src/linux_joystick.c:224:46: warning: ‘%s’ directive output may be truncated writing up to 255 bytes into a region of size 19 [-Wformat-truncation=]
  224 |             snprintf(path, sizeof(path), "%s/%s", dirname, entry->d_name);
      |                                              ^~
../body-tracking-samples/extern/glfw/src/src/linux_joystick.c:224:13: note: ‘snprintf’ output 2 or more bytes (assuming 257) into a destination of size 20
  224 |             snprintf(path, sizeof(path), "%s/%s", dirname, entry->d_name);
      |             ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[33/33] Linking CXX executable bin/simple_3d_viewer
FAILED: bin/simple_3d_viewer 
: && /usr/bin/c++    -Wl,--as-needed,-rpath-link=/home/mjhmilla/dev/AzureKinect/Azure-Kinect-Samples/build/bin body-tracking-samples/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o  -o bin/simple_3d_viewer  -Wl,-rpath,"\$ORIGIN"  -lk4a  -lk4abt  body-tracking-samples/sample_helper_libs/window_controller_3d/libwindow_controller_3d.a  body-tracking-samples/extern/glfw/src/src/libglfw3.a  /usr/lib/x86_64-linux-gnu/librt.so  -lm  -ldl  /usr/lib/x86_64-linux-gnu/libX11.so  -lpthread  /usr/lib/x86_64-linux-gnu/libXrandr.so  /usr/lib/x86_64-linux-gnu/libXinerama.so  /usr/lib/x86_64-linux-gnu/libXcursor.so && :
/usr/bin/ld: body-tracking-samples/simple_3d_viewer/CMakeFiles/simple_3d_viewer.dir/main.cpp.o: in function `PlayFile(InputSettings)':
main.cpp:(.text+0xd42): undefined reference to `k4a_playback_open'
/usr/bin/ld: main.cpp:(.text+0xd89): undefined reference to `k4a_playback_get_calibration'
/usr/bin/ld: main.cpp:(.text+0xefd): undefined reference to `k4a_playback_get_next_capture'
/usr/bin/ld: main.cpp:(.text+0x10df): undefined reference to `k4a_playback_close'
collect2: error: ld returned 1 exit status
ninja: build stopped: subcommand failed.
-----------

The fix for this comes from this github post:
	https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1296

And it was:

"This could be resolved by adding k4arecord in target_link_libraries in CMakeLists.txt of simple_3d_viewer"

Quite literally (2nd last line added manually):

----------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

add_executable(simple_3d_viewer main.cpp)

target_include_directories(simple_3d_viewer PRIVATE ../sample_helper_includes)

# Dependencies of this library
target_link_libraries(simple_3d_viewer PRIVATE 
    k4a
    k4abt
    window_controller_3d::window_controller_3d
    glfw::glfw
    k4arecord
    )
----------------------------------------


# Learning more

To learn more, I recommend carefully going through the kinect_mocap_studio.cc code. This code was created using the examples that appear in the Azure-Kinect-Sensor-SDK and the Azure-Kinect-Samples. Note that not all of these samples are configured to be built in the original download. However, with a little bit of CMake work most of the samples can be built and run. There is a lot of additional functionality that appears in these examples that you can take advantage of in your own programs.


