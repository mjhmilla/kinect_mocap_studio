﻿#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>


#include <k4a/k4a.h>
#include <k4abt.h>


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
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
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
  //bool save_camera_data = false;

  try{

    TCLAP::CmdLine cmd( "kinect_mocap_studio is a command-line tool to record"
                        "video and skeletal data from the Azure-Kinect",
                        ' ', "0.0");

    TCLAP::ValueArg<std::string> output_name_arg("o","oname",
        "Name of output file",false,"output","string");

    cmd.add( output_name_arg );

    //TCLAP::SwitchArg mkv_switch("m","mkv","Record rgb and depth camera data"
    //                              " to an *.mkv file", cmd, false);

    // Parse the argv array.
    cmd.parse( argc, argv );

    // Get the value parsed by each arg.
    output_file_name = output_name_arg.getValue();
    //save_camera_data = mkvSwitch.getValue();


  } catch (TCLAP::ArgException &e)  // catch exceptions
  {
    std::cerr << "error: " << e.error() << " for arg " << e.argId()
              << std::endl;
    exit(1);
  }


  int frame_count       = 0;
  //int frame_count_max   = 100;

  std::string output_json_file = output_file_name+".json";


  int32_t timeout_in_ms = 1;

  //
  // Configure and starte the device
  //
  k4a_device_t device = NULL;
  VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

  // Start camera. Make sure depth camera is enabled.
  k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  deviceConfig.depth_mode       = K4A_DEPTH_MODE_NFOV_UNBINNED;
  deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
  VERIFY(k4a_device_start_cameras(device, &deviceConfig),
       "Start K4A cameras failed!");

  //Get the sensor calibration information
  k4a_calibration_t sensor_calibration;
  VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode,
                    deviceConfig.color_resolution,
                    &sensor_calibration),
        "Get depth camera calibration failed!");
  int depthWidth
      = sensor_calibration.depth_camera_calibration.resolution_width;
  int depthHeight
      = sensor_calibration.depth_camera_calibration.resolution_height;

  //Initialize and start the body tracker
  k4abt_tracker_t tracker = NULL;
  k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
  VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker),
        "Body tracker initialization failed!");

  //Start the IMU
  VERIFY(k4a_device_start_imu(device),"Start K4A imu failed!");


  //
  // JSON pre-amble
  //

  //Store the json pre-amble data
  nlohmann::json json_output;
  nlohmann::json frames_json = nlohmann::json::array();

  json_output["k4abt_sdk_version"] = K4ABT_VERSION_STR;

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
  // Visualization Window
  //

  Window3dWrapper window3d;
  window3d.Create("3D Visualization", sensor_calibration);
  window3d.SetCloseCallback(CloseCallback);
  window3d.SetKeyCallback(ProcessKey);




  //
  // Process each frame
  //

  do
  {
    k4a_capture_t sensor_capture = nullptr;

    k4a_wait_result_t get_capture_result
        = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);

    if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
    {


      k4a_wait_result_t queue_capture_result =
          k4abt_tracker_enqueue_capture(tracker, sensor_capture,
                                        K4A_WAIT_INFINITE);

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
        VERIFY_WAIT( k4a_device_get_imu_sample(device,&imu_sample,
                                               timeout_in_ms),
                       "Timed out waiting for IMU data");
        push_imu_data_to_json(imu_result_json, imu_sample);
        frame_result_json["imu"].push_back(imu_result_json);

        //Vizualize the tracked result
        VisualizeResult(body_frame, window3d, depthWidth, depthHeight);
        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();


        k4abt_frame_release(body_frame);

        // Remember to release the body frame once you finish using it
        frames_json.push_back(frame_result_json);
        frame_count++;

      }
      else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
      {
        //  It should never hit timeout when K4A_WAIT_INFINITE is set.
        printf("Error! Pop body frame result timeout!\n");
        break;
      }
      else
      {
        printf("Pop body frame result failed!\n");
        break;
      }
    }
    else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
      // It should never hit time out when K4A_WAIT_INFINITE is set.
      printf("Error! Get depth frame time out!\n");
      break;
    }
    else
    {
      printf("Get depth capture returned error: %d\n", get_capture_result);
      break;
    }

  } while (s_isRunning);

  printf("Finished body tracking processing!\n");

  //Write the frame_data_time_series to file

  json_output["frames"] = frames_json;
  std::ofstream output_file(output_json_file.c_str());
  output_file << std::setw(4) << json_output << std::endl;
  std::cout   << frame_count << " Frames written to "
              << output_json_file << std::endl;


  window3d.Delete();
  k4abt_tracker_shutdown(tracker);
  k4abt_tracker_destroy(tracker);

  k4a_device_stop_cameras(device);
  k4a_device_stop_imu(device);
  k4a_device_close(device);

  return 0;
}
