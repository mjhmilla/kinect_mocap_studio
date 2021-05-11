#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <ctime>

#include <k4a/k4a.h>
#include <k4abt.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>

#include "FloorDetector.h"
#include "PointCloudGenerator.h"
#include "BodyTrackingHelpers.h"
#include "Utilities.h"
#include <Window3dWrapper.h>

#include <nlohmann/json.hpp>
#include <tclap/CmdLine.h>

#define VERIFY(result, error)                               \
  if(result != K4A_RESULT_SUCCEEDED)                        \
  {                                                         \
    printf("%s \n - (File: %s, Function: %s, Line: %d)\n",  \
            error, __FILE__, __FUNCTION__, __LINE__);       \
    exit(1);                                                \
  }                                                         \

#define VERIFY_WAIT(result, error)                          \
  if(result != K4A_WAIT_RESULT_SUCCEEDED)                   \
  {                                                         \
    printf("%s \n - (File: %s, Function: %s, Line: %d)\n",  \
            error, __FILE__, __FUNCTION__, __LINE__);       \
    exit(1);                                                \
  }

bool check_depth_image_exists(k4a_capture_t capture)
{
  k4a_image_t depth = k4a_capture_get_depth_image(capture);
  if (depth != nullptr)
  {
      k4a_image_release(depth);
      return true;
  }
  else
  {
      return false;
  }
}

/*
To do:
1. Use TCLAP to add some command line options
2. Add a preview of the skeleton + the room
3. Add buttons for start/stop and write
4. Add an option (pre-checked) to measure the position of the floor (averaged)
   and to add it to the json file using floor_detector_sample
5. Add an option (pre-checked) to append the trial number to the file name
6. Write a rbdl-toolkit plug in to visualize this data
*/

// Global State and Key Process Function
bool s_isRunning = true;
Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;


//Taken from Azure-Kinect-Samples/body-tracking-samples/simple_3d_viewer
void PrintAppUsage()
{
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse"
                " left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene "
                "with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by "
                "scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by "
                "right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

//Taken from Azure-Kinect-Samples/body-tracking-samples/simple_3d_viewer
int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1)
                       % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintAppUsage();
        break;
    }
    return 1;
}

//Taken from Azure-Kinect-Samples/body-tracking-samples/simple_3d_viewer
int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

//Taken from Azure-Kinect-Samples/body-tracking-samples/simple_3d_viewer
void VisualizeResult(k4abt_frame_t bodyFrame, Window3dWrapper& window3d,
                     int depthWidth, int depthHeight) {

  // Obtain original capture that generates the body tracking result
  k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
  k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);

  std::vector<Color> pointCloudColors(depthWidth * depthHeight,
                                      { 1.f, 1.f, 1.f, 1.f });

  // Read body index map and assign colors
  k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
  const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
  for (int i = 0; i < depthWidth * depthHeight; i++)
  {
    uint8_t bodyIndex = bodyIndexMapBuffer[i];
    if (bodyIndex != K4ABT_BODY_INDEX_MAP_BACKGROUND)
    {
      uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
      pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
    }
  }
  k4a_image_release(bodyIndexMap);

  // Visualize point cloud
  window3d.UpdatePointClouds(depthImage, pointCloudColors);

  // Visualize the skeleton data
  window3d.CleanJointsAndBones();
  uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
  for (uint32_t i = 0; i < numBodies; i++)
  {
    k4abt_body_t body;
    VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton),
           "Get skeleton from body frame failed!");
    body.id = k4abt_frame_get_body_id(bodyFrame, i);

    // Assign the correct color based on the body id
    Color color = g_bodyColors[body.id % g_bodyColors.size()];
    color.a = 0.4f;
    Color lowConfidenceColor = color;
    lowConfidenceColor.a = 0.1f;

    // Visualize joints
    for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++)
    {
      if (body.skeleton.joints[joint].confidence_level
          >= K4ABT_JOINT_CONFIDENCE_LOW)
      {
        const k4a_float3_t& jointPosition
            = body.skeleton.joints[joint].position;
        const k4a_quaternion_t& jointOrientation
            = body.skeleton.joints[joint].orientation;

        window3d.AddJoint(
          jointPosition,
          jointOrientation,
          body.skeleton.joints[joint].confidence_level
              >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
      }
    }

    // Visualize bones
    for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++)
    {
      k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
      k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

      if (body.skeleton.joints[joint1].confidence_level
          >= K4ABT_JOINT_CONFIDENCE_LOW &&
          body.skeleton.joints[joint2].confidence_level
          >= K4ABT_JOINT_CONFIDENCE_LOW)
      {
        bool confidentBone
            = body.skeleton.joints[joint1].confidence_level
            >= K4ABT_JOINT_CONFIDENCE_MEDIUM
            && body.skeleton.joints[joint2].confidence_level
            >= K4ABT_JOINT_CONFIDENCE_MEDIUM;

        const k4a_float3_t& joint1Position
            = body.skeleton.joints[joint1].position;
        const k4a_float3_t& joint2Position
            = body.skeleton.joints[joint2].position;

        window3d.AddBone(joint1Position, joint2Position,
                         confidentBone ? color : lowConfidenceColor);
      }
    }
  }

  k4a_capture_release(originalCapture);
  k4a_image_release(depthImage);
}


/**
Inspired by the code in
Azure-Kinect-Samples/body-tracking-samples/offline_processor

@param imu_result_json an empty json object to populate
@param imu_sample a sample from the imu

*/
void push_imu_data_to_json(nlohmann::json &imu_result_json,
                           k4a_imu_sample_t &imu_sample)
{

  imu_result_json["temperature"] = imu_sample.temperature;

  imu_result_json["acc_sample"].push_back(
              {   imu_sample.acc_sample.xyz.x,
                  imu_sample.acc_sample.xyz.y,
                  imu_sample.acc_sample.xyz.z});

  imu_result_json["acc_timestamp_usec"] =
          imu_sample.acc_timestamp_usec;

  imu_result_json["gyro_sample"].push_back(
              {   imu_sample.gyro_sample.xyz.x,
                  imu_sample.gyro_sample.xyz.y,
                  imu_sample.gyro_sample.xyz.z});

  imu_result_json["gyro_timestamp_usec"] =
          imu_sample.gyro_timestamp_usec;

}

/**
Inspired by the code in
Azure-Kinect-Samples/body-tracking-samples/offline_processor

@param body_result_json an empty json object to populate
@param body_frame a sample of the tracked bodies

*/
void push_body_data_to_json(nlohmann::json &body_result_json,
                                k4abt_frame_t &body_frame,
                                uint32_t num_bodies)
{
  for(size_t index_body = 0; index_body < num_bodies; ++index_body)
    {

      k4abt_body_t body;

      //Get the skeleton
      k4abt_frame_get_body_skeleton(body_frame, index_body,
                                    &body.skeleton);
      //Get the body id
      body.id = k4abt_frame_get_body_id(body_frame, index_body);

      body_result_json["body_id"] = body.id;

      for(int index_joint =0;
          index_joint < (int)K4ABT_JOINT_COUNT;++index_joint)
        {
          body_result_json["joint_positions"].push_back(
          {    body.skeleton.joints[index_joint].position.xyz.x,
               body.skeleton.joints[index_joint].position.xyz.y,
               body.skeleton.joints[index_joint].position.xyz.z });

          body_result_json["joint_orientations"].push_back(
          {  body.skeleton.joints[index_joint].orientation.wxyz.w,
             body.skeleton.joints[index_joint].orientation.wxyz.x,
             body.skeleton.joints[index_joint].orientation.wxyz.y,
             body.skeleton.joints[index_joint].orientation.wxyz.z });

          body_result_json["confidence_level"] =
                    body.skeleton.joints[index_joint].confidence_level;
      }

  }
}



int main(int argc, char**argv)
{


  //To-do's
  //1. Include TCLAP to get command line arguments.
  //2. Add arguments for the
  //    - [required] number of seconds to record
  //    - [required] name of the json file to write
  //    - [optional] skeleton tracking smoothing parameter
  //    - [optional] name of the mkv file to write

  std::string output_file_name;
  double temporal_smoothing       = 0.;
  //bool save_camera_data = false;
  int k4a_depth_mode              = 0;
  std::string k4a_depth_mode_str;
  std::string input_sensor_file_str;
  bool process_sensor_file        = false;
  int k4a_frames_per_second       = 0;
  std::string k4a_color_resolution_str;
  int k4a_color_resolution        = 0;
  bool record_sensor_data         = false;
  try{

    TCLAP::CmdLine cmd( "kinect_mocap_studio is a command-line tool to record"
                        "video and skeletal data from the Azure-Kinect",
                        ' ', "0.0");


    TCLAP::ValueArg<std::string> input_sensor_file_arg("i","infile",
        "Input sensor file of type *.mkv to process",false,"",
        "string");

    cmd.add( input_sensor_file_arg );

    TCLAP::ValueArg<bool> record_sensor_data_arg("w","write",
        "Write sensor data to *.mkv file",false,false,
        "bool");

    cmd.add( record_sensor_data_arg );

    TCLAP::ValueArg<std::string> output_name_arg("o","outfile",
        "Name of output file excluding the file extension",false,"output",
        "string");

    cmd.add( output_name_arg );

    TCLAP::ValueArg<std::string> depth_mode_arg("d","depth_mode",
      "Depth mode: OFF, NFOV_2X2BINNED, NFOV_UNBINNED, "
      "WFOV_2X2BINNED, WFOV_UNBINNED, PASSIVE_IR",false,
       "NFOV_UNBINNED","string");


    cmd.add( depth_mode_arg );

    TCLAP::ValueArg<std::string> k4a_color_resolution_arg("c","color_mode",
      "Color resolution: OFF, 720P, 1080P, "
      "1440P, 1536P, 2160P, 3072P",false,
       "OFF","string");

    cmd.add( k4a_color_resolution_arg );

    TCLAP::ValueArg<int> k4a_frames_per_second_arg("f","fps",
      "Frames per second: 5, 15, 30",false,
      30,"int");

    cmd.add( k4a_frames_per_second_arg );

    TCLAP::ValueArg<double> temporal_smoothing_arg("s","smoothing",
      "Amount of temporal smoothing in the skeleton tracker (0-1)",false,
      K4ABT_DEFAULT_TRACKER_SMOOTHING_FACTOR,"double");

    cmd.add( temporal_smoothing_arg );



    //TCLAP::SwitchArg mkv_switch("m","mkv","Record rgb and depth camera data"
    //                              " to an *.mkv file", cmd, false);

    // Parse the argv array.
    cmd.parse( argc, argv );

    // Get the value parsed by each arg.
    output_file_name      = output_name_arg.getValue();

    k4a_depth_mode_str        = depth_mode_arg.getValue();
    if( std::strcmp(k4a_depth_mode_str.c_str(),"OFF")==0){
        k4a_depth_mode=0;
    }else if(std::strcmp(k4a_depth_mode_str.c_str(),"NFOV_2X2BINNED")==0){
        k4a_depth_mode=1;
    }else if(std::strcmp(k4a_depth_mode_str.c_str(),"NFOV_UNBINNED")==0){
        k4a_depth_mode=2;
    }else if(std::strcmp(k4a_depth_mode_str.c_str(),"WFOV_2X2BINNED")==0){
        k4a_depth_mode=3;
    }else if(std::strcmp(k4a_depth_mode_str.c_str(),"WFOV_UNBINNED")==0){
        k4a_depth_mode=4;
    }else if(std::strcmp(k4a_depth_mode_str.c_str(),"PASSIVE_IR")==0){
        k4a_depth_mode=5;
    }else{
      std::cerr << "error: depth_mode must be: OFF, NFOV_2X2BINNED, "
                << "NFOV_UNBINNED, WFOV_2X2BINNED, WFOV_UNBINNED, PASSIVE_IR."
                << std::endl;
      exit(1);
    }

    k4a_color_resolution_str        = k4a_color_resolution_arg.getValue();
    if( std::strcmp(k4a_color_resolution_str.c_str(),"OFF")==0){
        k4a_color_resolution=0;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"720P")==0){
        k4a_color_resolution=1;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"1080P")==0){
        k4a_color_resolution=2;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"1440P")==0){
        k4a_color_resolution=3;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"1536P")==0){
        k4a_color_resolution=4;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"2160P")==0){
        k4a_color_resolution=5;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"3072P")==0){
        k4a_color_resolution=6;
    }else{
      std::cerr << "error: color resolution must be: OFF, 720P, 1080P, "
                   "1440P, 1536P, 2160P, 3072P" << std::endl;
      exit(1);
    }

    record_sensor_data = record_sensor_data_arg.getValue();

    k4a_frames_per_second = k4a_frames_per_second_arg.getValue();
    if( k4a_frames_per_second != 5
        && k4a_frames_per_second != 15
        && k4a_frames_per_second != 30){
        std::cerr << "error: fps must be 5, 15, or 30"
                  << std::endl;
        exit(1);
    }

    temporal_smoothing    = temporal_smoothing_arg.getValue();
    if(temporal_smoothing > 1.0 || temporal_smoothing < 0.0){
        std::cerr << "error: temporal_smoothing must be between 0.0-1.0"
                  << std::endl;
        exit(1);
    }


    input_sensor_file_str = input_sensor_file_arg.getValue();
    if(input_sensor_file_str.length() > 0)
    {
      std::string mkv_ext = ".mkv";
      if(input_sensor_file_str.find(mkv_ext) !=
         (input_sensor_file_str.length()-4))
      {
        std::cerr << "error: input sensor file must be of type *.mkv"
                  << std::endl;
        exit(1);
      }
      process_sensor_file = true;
      output_file_name =
          input_sensor_file_str.substr(0,input_sensor_file_str.length()-4);
    }

    if(process_sensor_file && record_sensor_data)
    {
        std::cerr << "error: cannot process an input file (-i) and write"
                     "(-w) the sensor data at the same time" << std::endl;
        exit(1);
    }

  } catch (TCLAP::ArgException &e)  // catch exceptions
  {
    std::cerr << "error: " << e.error() << " for arg " << e.argId()
              << std::endl;
    exit(1);
  }






  PrintAppUsage();

  int frame_count       = 0;
  //int frame_count_max   = 100;

  //It appears as though some of the configuration information is
  //not written to the mkv files and so can become lost. Here I'm
  //using an ugly but practical solution of embedding this meta data in
  //the file name.
  if(!process_sensor_file){
    std::stringstream output_file_name_ss;
    output_file_name_ss << output_file_name;
    output_file_name_ss << "_" << k4a_depth_mode_str
                        << "_" << k4a_color_resolution_str
                        << "_" << k4a_frames_per_second << "fps";
    output_file_name = output_file_name_ss.str();
  }


  std::string output_json_file = output_file_name+".json";
  std::string output_sensor_file = output_file_name+".mkv";

  //Check to make sure the file name is unique
  int counter = 0;
  bool name_collision=false;
  do{
    std::ifstream jsonFile(output_json_file.c_str());
    std::ifstream sensorFile(output_sensor_file.c_str());
    name_collision=false;

    if(jsonFile.good()){
        name_collision=true;
        jsonFile.close();
    }
    if(sensorFile.good()){
        name_collision=true;
        sensorFile.close();
    }
    if(name_collision){
        std::stringstream ss;
        ss << output_file_name << "_" << counter;
        output_json_file   = ss.str()+".json";
        output_sensor_file = ss.str()+".mkv";
        counter++;
    }

  }while(name_collision);

  //int32_t short_timeout_in_ms
  //    = int32_t(1000. / (3.*double(k4a_frames_per_second)) );

  //int32_t long_timeout_in_ms
  //    = int32_t(1000./ double(k4a_frames_per_second) );

  //
  // Configure and start the device
  //
  k4a_device_t device = NULL;
  k4a_playback_t playback_handle = NULL;

  k4a_calibration_t sensor_calibration;
  k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;


  if(process_sensor_file)
  {
    VERIFY(k4a_playback_open(input_sensor_file_str.c_str(), &playback_handle),
           "error: k4a_playback_open() failed");
    VERIFY(k4a_playback_get_calibration(playback_handle, &sensor_calibration),
           "error: k4a_playback_get_calibration() failed");

    k4a_record_configuration_t record_config;
    VERIFY(k4a_playback_get_record_configuration(playback_handle,&record_config),
           "error: k4a_playback_get_record_configuration() failed");

    switch(record_config.depth_mode){
      case K4A_DEPTH_MODE_OFF:
        {
          k4a_depth_mode_str = "OFF";
        } break;
      case K4A_DEPTH_MODE_NFOV_2X2BINNED:
        {
          k4a_depth_mode_str = "NFOV_2X2BINNED";
        } break;
      case K4A_DEPTH_MODE_NFOV_UNBINNED:
        {
          k4a_depth_mode_str = "NFOV_UNBINNED";
        } break;
      case K4A_DEPTH_MODE_WFOV_2X2BINNED:
        {
          k4a_depth_mode_str = "WFOV_2X2BINNED";
        } break;
      case K4A_DEPTH_MODE_WFOV_UNBINNED:
        {
          k4a_depth_mode_str = "WFOV_UNBINNED";
        } break;
      case K4A_DEPTH_MODE_PASSIVE_IR:
        {
          k4a_depth_mode_str = "PASSIVE_IR";
        } break;
      default:
      {
        std::cerr << "error: unrecognized depth_mode in recording" << std::endl;
      }
    };

    switch(record_config.color_resolution){
      case K4A_COLOR_RESOLUTION_OFF:
        {
          k4a_color_resolution_str = "OFF";
        } break;
      case K4A_COLOR_RESOLUTION_720P:
        {
          k4a_color_resolution_str = "720P";
        } break;
      case K4A_COLOR_RESOLUTION_1080P:
        {
          k4a_color_resolution_str = "1080P";
        } break;
      case K4A_COLOR_RESOLUTION_1440P:
        {
          k4a_color_resolution_str = "1440P";
        } break;
      case K4A_COLOR_RESOLUTION_1536P:
        {
          k4a_color_resolution_str = "1536P";
        } break;
      case K4A_COLOR_RESOLUTION_2160P:
        {
          k4a_color_resolution_str = "2160P";
        } break;
      case K4A_COLOR_RESOLUTION_3072P:
        {
          k4a_color_resolution_str = "3072P";
        } break;
      default:
      {
        std::cerr << "error: unrecognized color resolution in recording" << std::endl;
      }
    };

    k4a_frames_per_second = record_config.camera_fps;

    //device_config.camera_fps = record_config.camera_fps;
    //device_config.color_format = record_config.color_format;
    //device_config.color_resolution = record_config.color_resolution;
    //device_config.depth_delay_off_color_usec = record_config.depth_delay_off_color_usec;
    //device_config.depth_mode = record_config.depth_mode;
    //device_config.subordinate_delay_off_master_usec = record_config.subordinate_delay_off_master_usec;
    //device_config.wired_sync_mode = record_config.wired_sync_mode;

  }else{

    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");
    // Start camera. Make sure depth camera is enabled.
    device_config.depth_mode       = k4a_depth_mode_t(k4a_depth_mode);

    switch (k4a_frames_per_second){
      case 5:{
        device_config.camera_fps   = K4A_FRAMES_PER_SECOND_5;
      } break;
      case 15:{
        device_config.camera_fps   = K4A_FRAMES_PER_SECOND_15;
      } break;
      case 30:{
        device_config.camera_fps   = K4A_FRAMES_PER_SECOND_30;
      } break;
      default:{
        std::cerr << "error: fps must be 5, 15, or 30"
                  << std::endl;
        exit(1);
      }
    };

    if( std::strcmp(k4a_color_resolution_str.c_str(),"OFF")==0){
        device_config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"720P")==0){
        device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"1080P")==0){
        device_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"1440P")==0){
        device_config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"1536P")==0){
        device_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"2160P")==0){
        device_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    }else if(std::strcmp(k4a_color_resolution_str.c_str(),"3072P")==0){
        device_config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
    }else{
      std::cerr << "error: color resolution must be: OFF, 720P, 1080P, "
                   "1440P, 1536P, 2160P, 3072P" << std::endl;
      exit(1);
    }


    VERIFY(k4a_device_start_cameras(device, &device_config),
         "Start K4A cameras failed!");

    //Get the sensor calibration information
    VERIFY(k4a_device_get_calibration(device, device_config.depth_mode,
                      device_config.color_resolution,
                      &sensor_calibration),
          "Get depth camera calibration failed!");
  }

  int depthWidth
      = sensor_calibration.depth_camera_calibration.resolution_width;
  int depthHeight
      = sensor_calibration.depth_camera_calibration.resolution_height;

  //
  // Echo the configuration to the command terminal
  //
  std::cout << "depth_mode         :" << k4a_depth_mode_str       << std::endl;
  std::cout << "color_resolution   :" << k4a_color_resolution_str << std::endl;
  std::cout << "frames_per_second  :" << k4a_frames_per_second    << std::endl;
  std::cout << "temporal smoothing :" << temporal_smoothing       << std::endl;
  std::cout << "output file name   :" << output_json_file         << std::endl;
  if(record_sensor_data){
    std::cout << "video file name    :"  << output_sensor_file    << std::endl;
  }

  //
  //Initialize and start the body tracker
  //
  k4abt_tracker_t tracker = NULL;
  k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
  VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker),
        "Body tracker initialization failed!");
  k4abt_tracker_set_temporal_smoothing(tracker,temporal_smoothing);

  //Start the IMU
  if(!process_sensor_file){
    VERIFY(k4a_device_start_imu(device),"Start K4A imu failed!");
  }
  //
  // JSON pre-amble
  //

  //Store the json pre-amble data
  nlohmann::json json_output;
  nlohmann::json frames_json = nlohmann::json::array();

  time_t now = time(0);
  char* dt = ctime(&now);
  json_output["start_time"] = dt;

  json_output["k4abt_sdk_version"] = K4ABT_VERSION_STR;

  json_output["depth_mode"]         = k4a_depth_mode_str;
  json_output["color_resolution"]   = k4a_color_resolution_str;
  json_output["frames_per_second"]  = k4a_frames_per_second;
  json_output["temporal_smoothing"] = temporal_smoothing;

  // Store all joint names to the json
  json_output["joint_names"] = nlohmann::json::array();
  for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
  {
    json_output["joint_names"].push_back(
                g_jointNames.find((k4abt_joint_id_t)i)->second);
  }

  // Store all bone linkings to the json
  json_output["bone_list"] = nlohmann::json::array();
  for (int i = 0; i < (int)g_boneList.size(); i++)
  {
    json_output["bone_list"].push_back(
                { g_jointNames.find(g_boneList[i].first)->second,
                  g_jointNames.find(g_boneList[i].second)->second });
  }

  //
  // PointCloudGenerator for floor estimation.
  //
  Samples::PointCloudGenerator pointCloudGenerator{ sensor_calibration };
  Samples::FloorDetector floorDetector;

  //
  // Visualization Window
  //

  Window3dWrapper window3d;
  window3d.Create("3D Visualization", sensor_calibration);
  window3d.SetCloseCallback(CloseCallback);
  window3d.SetKeyCallback(ProcessKey);

  k4a_record_t recording;

  if(record_sensor_data){
    if (K4A_FAILED(k4a_record_create(output_sensor_file.c_str(), device,
                                    device_config, &recording)))
    {
      std::cerr << "error: k4a_record_create() failed, unable to create "
                << output_sensor_file << std::endl;
      exit(1);
    }
    if( K4A_FAILED(k4a_record_add_imu_track(recording)) )
    {
      std::cerr << "Error: k4a_record_add_imu_track() failed" << std::endl;
      exit(1);
    }
    if( K4A_FAILED(k4a_record_write_header(recording)) )
    {
      std::cerr << "Error: k4a_record_write_header() failed" << std::endl;
      exit(1);
    }
  }

  //
  // Process each frame
  //

  do
  {
    k4a_capture_t sensor_capture = nullptr;

    bool capture_ready = false;
    if(process_sensor_file){
      k4a_stream_result_t stream_result =
          k4a_playback_get_next_capture(playback_handle, &sensor_capture);
      if (stream_result == K4A_STREAM_RESULT_EOF)
      {
        break;
      }
      else if( stream_result == K4A_STREAM_RESULT_SUCCEEDED)
      {
        capture_ready=true;
      }
      else
      {
        std::cerr << "error: k4a_playback_get_next_capture() failed at "
                  << frame_count << std::endl;
        exit(1);
      }

      if(check_depth_image_exists(sensor_capture)==false)
      {
        std::cerr << "error: stream contains no depth image at " << frame_count
                  << std::endl;
        capture_ready=false;
      }

    }else{
      k4a_wait_result_t get_capture_result
          = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);

      if(get_capture_result == K4A_WAIT_RESULT_SUCCEEDED){
          capture_ready=true;
      }else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
      {
          printf("error: k4a_device_get_capture() timed out \n");
          //break;
      }
      else
      {
        printf("error: k4a_device_get_capture(): %d\n", get_capture_result);
        //break;
      }
    }

    //Process the data
    if (capture_ready)
    {

      k4a_wait_result_t queue_capture_result =
          k4abt_tracker_enqueue_capture(tracker, sensor_capture,
                                        K4A_WAIT_INFINITE);
      k4a_image_t depth_image = k4a_capture_get_depth_image(sensor_capture);

      if(record_sensor_data)
      {
        k4a_result_t write_sensor_capture
            = k4a_record_write_capture(recording, sensor_capture);

        if (K4A_FAILED(write_sensor_capture))
        {
          std::cerr << "error: k4a_record_write_capture() returned "
                    << write_sensor_capture << std::endl;
          break;
        }
      }

      // Remember to release the sensor capture once you finish using it
      k4a_capture_release(sensor_capture);


      if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
      {
        // It should never hit timeout when K4A_WAIT_INFINITE is set.
        printf("Error! Add capture to tracker process queue timeout!\n");
        break;
      }
      else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
      {
        printf("Error! Add capture to tracker process queue failed!\n");
        break;
      }

      k4abt_frame_t body_frame = NULL;
      k4a_wait_result_t pop_frame_result =
              k4abt_tracker_pop_result(tracker, &body_frame,
                                       K4A_WAIT_INFINITE);

      if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
      {


        uint32_t num_bodies   = k4abt_frame_get_num_bodies(body_frame);
        uint64_t timestamp    = k4abt_frame_get_device_timestamp_usec(
                                  body_frame);

        nlohmann::json frame_result_json;
        frame_result_json["timestamp_usec"] = timestamp;
        frame_result_json["frame_id"]       = frame_count;
        frame_result_json["num_bodies"]     = num_bodies;
        frame_result_json["imu"]            = nlohmann::json::array();
        frame_result_json["bodies"]         = nlohmann::json::array();
        frame_result_json["floor"]         = nlohmann::json::array();


        //Question:
        // Is it faster to access bodies or to call
        // k4abt_frame_get_body_skeleton?

        //Fetch and save the skeleton tracking data to a json object
        nlohmann::json body_result_json;
        push_body_data_to_json(body_result_json, body_frame, num_bodies);
        frame_result_json["bodies"].push_back(body_result_json);

        //Fetch and save the imu data to a json object
        nlohmann::json imu_result_json;
        k4a_imu_sample_t imu_sample;
        bool imu_data_ready = false;
        if(process_sensor_file)
        {
          k4a_stream_result_t imu_result =
            k4a_playback_get_next_imu_sample(playback_handle,&imu_sample);
          if(imu_result == K4A_STREAM_RESULT_SUCCEEDED)
          {
            imu_data_ready = true;
          }
          else if(imu_result == K4A_STREAM_RESULT_EOF)
          {
            imu_data_ready = false;
          }
          else
          {
            std::cerr << "error: k4a_playback_get_next_imu_sample() failed at "
                      << frame_count << std::endl;
            exit(1);
          }

        }else{
          VERIFY_WAIT( k4a_device_get_imu_sample(device,&imu_sample,
                                                 K4A_WAIT_INFINITE),
                         "Timed out waiting for IMU data");
          imu_data_ready=true;
        }
        if(record_sensor_data)
        {
          k4a_result_t write_imu_sample
            = k4a_record_write_imu_sample(recording, imu_sample);
          if (K4A_FAILED(write_imu_sample))
          {
              std::cerr << "error: k4a_record_write_imu_sample() returned "
                        << write_imu_sample << std::endl;
              //break;
          }
        }


        if(imu_data_ready){
          push_imu_data_to_json(imu_result_json, imu_sample);
          frame_result_json["imu"].push_back(imu_result_json);
        }
        //Fit a plane to the depth points that are furthest away from
        //the camera in the direction of gravity (this will fail when the
        //camera accelerates by 0.2 m/s2 in any direction)
        //This uses code from teh floor_detector example code
        

        if(imu_data_ready)
        {
          // Update point cloud.
          pointCloudGenerator.Update(depth_image);

          // Get down-sampled cloud points.
          const int downsampleStep = 2;
          const auto& cloudPoints = pointCloudGenerator.GetCloudPoints(downsampleStep);

          // Detect floor plane based on latest visual and inertial observations.
          const size_t minimumFloorPointCount = 1024 / (downsampleStep * downsampleStep);
          const auto& maybeFloorPlane =
                  floorDetector.TryDetectFloorPlane(cloudPoints, imu_sample,
                                sensor_calibration, minimumFloorPointCount);

          // Visualize point cloud.
          window3d.UpdatePointClouds(depth_image);

          // Visualize the floor plane.
          nlohmann::json floor_result_json;
          if (maybeFloorPlane.has_value())
          {
              // For visualization purposes, make floor origin the projection of a point 1.5m in front of the camera.
              Samples::Vector cameraOrigin = { 0, 0, 0 };
              Samples::Vector cameraForward = { 0, 0, 1 };

              auto p = maybeFloorPlane->ProjectPoint(cameraOrigin)
                     + maybeFloorPlane->ProjectVector(cameraForward) * 1.5f;

              auto n = maybeFloorPlane->Normal;

              window3d.SetFloorRendering(true, p.X, p.Y, p.Z, n.X, n.Y, n.Z);
              floor_result_json["point"].push_back(
                    {p.X*1000.f, p.Y*1000.f, p.Z*1000.f});
              floor_result_json["normal"].push_back({n.X, n.Y, n.Z});
              floor_result_json["valid"]=true;
          }
          else
          {
              window3d.SetFloorRendering(false, 0, 0, 0);
              floor_result_json["point"].push_back({0.,0.,0.});
              floor_result_json["normal"].push_back({0.,0.,0.});
              floor_result_json["valid"]=false;
          }
          frame_result_json["floor"].push_back(floor_result_json);
        }
        //Vizualize the tracked result
        VisualizeResult(body_frame, window3d, depthWidth, depthHeight);
        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();


        k4abt_frame_release(body_frame);
        k4a_image_release(depth_image);

        // Remember to release the body frame once you finish using it
        frames_json.push_back(frame_result_json);
        frame_count++;

      }
      else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
      {
        //  It should never hit timeout when K4A_WAIT_INFINITE is set.
        printf("error: timeout k4abt_tracker_pop_result()\n");
        //break;
      }
      else
      {
        printf("Pop body frame result failed!\n");
        //break;
      }
    }


  } while (s_isRunning);

  printf("Finished body tracking processing!\n");

  //Write sensor data to file
  if(record_sensor_data)
  {
    k4a_record_flush(recording);
    k4a_record_close(recording);
    std::cout << "Sensor data written to " << output_sensor_file << std::endl;
  }

  //Write the frame_data_time_series to file
  now = time(0);
  dt = ctime(&now);
  json_output["end_time"] = dt;

  json_output["frames"] = frames_json;
  std::ofstream output_file(output_json_file.c_str());
  output_file << std::setw(4) << json_output << std::endl;
  std::cout   << frame_count << " Frames written to "
              << output_json_file << std::endl;


  window3d.Delete();
  k4abt_tracker_shutdown(tracker);
  k4abt_tracker_destroy(tracker);

  if(process_sensor_file)
  {
    k4a_playback_close(playback_handle);
  }
  else
  {
    k4a_device_stop_cameras(device);
    k4a_device_stop_imu(device);
    k4a_device_close(device);
  }



  return 0;
}
