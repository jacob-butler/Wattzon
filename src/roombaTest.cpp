#include "../include/roomba.hpp"
#include <vector>
#include <iostream>
#include <utility>
#include <chrono>
#include <cstdlib>
#include <cmath>

#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

using std::vector;
using std::cout;
using std::endl;
using std::pair;

const float PI = 3.14159265359;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STREAM          RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_Z16    // rs2_format is identifies how binary data is encoded within a frame   //
#define WIDTH           640               // Defines the number of columns for each frame                         //
#define HEIGHT          480               // Defines the number of lines for each frame                           //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float distanceToTravel(float turnAng){
    float roombaRadius = 175;
    float viewDistance = 300;
    float B = roombaRadius + 2*viewDistance;
    return B*std::sqrt(abs(std::sin(PI/180*(turnAng))/std::cos(PI/180*(turnAng/2)-1)));
   
}

float turnLeftOrRight(const std::pair<float,float> &roombaPos, const std::pair<float,float> &targetPos, float turnAng, float turnSpeed){ 
    float distance = distanceToTravel(turnAng);
    float phi = std::atan2(roombaPos.second,roombaPos.first);
    std::pair<float,float> leftPoint = roombaPos; 
    leftPoint.first = targetPos.first - (std::cos(PI/180*(turnAng)+phi)*distance) - leftPoint.first;
    leftPoint.second = targetPos.second - (std::sin(PI/180*(turnAng)+phi)*distance) - leftPoint.second;
    float leftDistance = leftPoint.first*leftPoint.first+leftPoint.second*leftPoint.second;
    std::pair<float,float> rightPoint = roombaPos;
    rightPoint.first = targetPos.first - (std::cos(PI/180*(-turnAng)+phi)*distance) - rightPoint.first;
    rightPoint.second = targetPos.second - (std::sin(PI/180*(-turnAng)+phi)*distance) - rightPoint.second;  
    float rightDistance = rightPoint.first*rightPoint.first+rightPoint.second*rightPoint.second;
    if(rightDistance < leftDistance)
        return -abs(turnSpeed);
    return abs(turnSpeed); 
}

void stopIfObjectInPath(){
    rs2_error* e = 0;

    // Create a context object. This object owns the handles to all connected realsense devices.
    // The returned object should be released with rs2_delete_context(...)
    rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);


    /* Get a list of all the connected devices. */
    // The returned object should be released with rs2_delete_device_list(...)
    rs2_device_list* device_list = rs2_query_devices(ctx, &e);


    int dev_count = rs2_get_device_count(device_list, &e);

    printf("There are %d connected RealSense devices.\n", dev_count);
    if (0 == dev_count)
        return;

    // Get the first connected device
    // The returned object should be released with rs2_delete_device(...)
    rs2_device* dev = rs2_create_device(device_list, 0, &e);

    // Create a pipeline to configure, start and stop camera streaming
    // The returned object should be released with rs2_delete_pipeline(...)
    rs2_pipeline* pipeline =  rs2_create_pipeline(ctx, &e);


    // Create a config instance, used to specify hardware configuration
    // The retunred object should be released with rs2_delete_config(...)
    rs2_config* config = rs2_create_config(&e);


    // Request a specific configuration
    rs2_config_enable_stream(config, STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);


    // Start the pipeline streaming
    // The retunred object should be released with rs2_delete_pipeline_profile(...)
    rs2_pipeline_profile* pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
    if (e)
    {
        printf("The connected device doesn't support depth streaming!\n");
        exit(EXIT_FAILURE);
    }
    
    std::cout << "Program started" << std::endl;
    Roomba roomba("/dev/tty.usbserial-DN0267K3", 115200);
    //roomba.start(); 
    roomba.setForward(0);
    auto start = std::chrono::high_resolution_clock::now(); 
    auto end = std::chrono::high_resolution_clock::now(); 
    float lastTime = 0;
    float startAngle = 0;
    float endAngle = 0;
    float distanceToGo = 0;
    float objectWasNear = false;
    pair<float,float> previousPosition(0,0); 
    pair<float,float> positionToBe{3000,000};
    bool refindingTarget = false;
    while (true)
    {
        // This call waits until a new composite_frame is available
        // composite_frame holds a set of frames. It is used to prevent frame drops
        // The retunred object should be released with rs2_release_frame(...)
        if(e){
            std::cout << "Camera Error" << std::endl;
        } 
        rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, 5000, &e);


        // Returns the number of frames embedded within the composite frame
        int num_of_frames = rs2_embedded_frames_count(frames, &e);


        int i;
        for (i = 0; i < num_of_frames; ++i)
        {
            float directionToTarget = 180/PI*std::atan2(positionToBe.second-roomba.getPosition().second, positionToBe.first-roomba.getPosition().first);
            
            // The retunred object should be released with rs2_release_frame(...)
            rs2_frame* frame = rs2_extract_frame(frames, i, &e);


            // Check if the given frame can be extended to depth frame interface
            // Accept only depth frames and skip other frames
            if (0 == rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, &e))
                continue;

            // Get the depth frame's dimensions
            int width = rs2_get_frame_width(frame, &e);

            int height = rs2_get_frame_height(frame, &e);


            // Query the distance from the camera to the object in the center of the image
            bool objectNear = false; 
            for(int i = 0; i<41; ++i){
                float dist_to_center1 = rs2_depth_frame_get_distance(frame, width*i / 40, height*3 / 9, &e);
                float dist_to_center2 = rs2_depth_frame_get_distance(frame, width*i / 40, height*6 / 9, &e);
                float dist_to_center3 = rs2_depth_frame_get_distance(frame, width*i / 40, 410, &e);
                //printf("distance %.3f \n", dist_to_center);
                if((dist_to_center1 < 0.30 && dist_to_center1 > 0.0) ||
                   (dist_to_center2 < 0.30 && dist_to_center2 > 0.0) ||
                   (dist_to_center3 < 0.30 && dist_to_center3 > 0.0)){
                    objectNear = true;
                }
            }
            rs2_release_frame(frame);
            cout << "Distance to go: " <<  distanceToGo << endl;
            if(objectNear){
                std::cout << refindingTarget << endl;
                distanceToGo = 0;
                if(roomba.getTurningVelocity() == 0){
                    roomba.turn(turnLeftOrRight(roomba.getPosition(),positionToBe,45,50));
                    startAngle = roomba.getDirection();
                }
                objectWasNear = true; 
            }else{
                if(objectWasNear){
                    //cout << roomba.getDirection()-startAngle << endl;
                    objectWasNear = false;
                    distanceToGo = distanceToTravel(roomba.getDirection()-startAngle);
                }
                if(distanceToGo > 0){
                    if(roomba.getForwardVelocity() == 0){
                         roomba.setForward(150);
                    }else{
                        pair<float,float> positionToGo(roomba.getPosition().first - previousPosition.first,roomba.getPosition().second - previousPosition.second);
                        cout << "position to go is: " << roomba.getPosition().first - previousPosition.first 
                             << "," << roomba.getPosition().second - previousPosition.second 
                             << endl;
                        distanceToGo -= std::sqrt(positionToGo.first*positionToGo.first+positionToGo.second*positionToGo.second);
                    }
                }else{
                    if(abs(roomba.getDirection()-directionToTarget) > 1){
                        refindingTarget = true;
                        if(roomba.getTurningVelocity() == 0){
                           roomba.turn(turnLeftOrRight(roomba.getPosition(),positionToBe,45,50));
                        }
                    }else{
                        refindingTarget = false;
                        if(roomba.getForwardVelocity() == 0){
                            roomba.setForward(150);
                        }
                    }
                }
                
            }
            if(roomba.BumperAndWheels() > 0 || (abs(roomba.getPosition().first-positionToBe.first)<50 &&
              (abs(roomba.getPosition().second-positionToBe.second))<50)){              
                roomba.setForward(0);
                rs2_pipeline_stop(pipeline, &e);

                // Release resources
                rs2_delete_pipeline_profile(pipeline_profile);
                rs2_delete_config(config);
                rs2_delete_pipeline(pipeline);
                rs2_delete_device(dev);
                rs2_delete_device_list(device_list);
                rs2_delete_context(ctx); 
                return;
            }    
            std::cout << "Is there an object in View: " << objectNear << std::endl;
            auto end = std::chrono::high_resolution_clock::now();    
            std::chrono::duration<float> diff = end-start; 
            previousPosition = roomba.getPosition();
            roomba.update(diff.count()-lastTime);
            std::cout << "Angle of roomba: " << roomba.getDirection() << std::endl;
            std::cout << "Direction to target: " << directionToTarget << endl;
            std::cout << "Positions is: " << roomba.getPosition().first << ","<< roomba.getPosition().second << std::endl;
            lastTime = diff.count();
        }

        rs2_release_frame(frames);
    }

    // Stop the pipeline streaming
    rs2_pipeline_stop(pipeline, &e);

    // Release resources
    rs2_delete_pipeline_profile(pipeline_profile);
    rs2_delete_config(config);
    rs2_delete_pipeline(pipeline);
    rs2_delete_device(dev);
    rs2_delete_device_list(device_list);
    rs2_delete_context(ctx); 
}

void testingAtan2(){
    float y = -1;
    float x = 0;
    std::cout << "Atan2 of y= " << y << ", " << x << " is "<< std::atan2(y,x)*180/PI << std::endl;
}

int main()
{
    //testingAtan2();
    //actionLoop();
    stopIfObjectInPath();
    //cout << "We should turn" << turnLeftOrRight(std::pair<float,float>{500,500},std::pair<float,float>{1000,1700},30,50) << std::endl;
    //Roomba roomba("/dev/tty.usbserial-DN0267K3", 115200);
    //std::cout << ((10&0xFF00)>>8) << std::endl;
    //std::cout << ((10&0x00FF)) << std::endl;
    //std::cout << (((-10)&0xFF00)>>8) << std::endl;
    //std::cout << (((-10)&0x00FF)) << std::endl;

    //roomba.stop();
    //roomba.start();
    //roomba.turn(-100);
}
