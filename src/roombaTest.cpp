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

void actionLoop(){
    std::cout << "Program started" << std::endl;
    pair<float,float> position(0,0);
    pair<float,float> checkPointOne(300,0);
    pair<float,float> checkPointTwo(300,-300);
    float angle = 0;
    float angleVelocity = 0;
    float radius = 75;
    Roomba roomba("/dev/tty.usbserial-DN0267K3", 115200);
    //roomba.start(); 
    float velocity = 150;
    float distance = 900*900;
    float lastTime = 0; 
    roomba.setForward(velocity);
    auto start = std::chrono::high_resolution_clock::now(); 
    auto end = std::chrono::high_resolution_clock::now(); 
    bool checkOne = true;
    bool checkTwo = false;
    bool turn = false;
    while(true){
        end = std::chrono::high_resolution_clock::now();
        if(roomba.BumperAndWheels() > 0){
            roomba.setForward(0);
            return;
        } 
        if(checkOne){
            if(abs(position.first-checkPointOne.first)<5 && abs(position.second-checkPointOne.second)<5){
                velocity = 0;
                turn = true;
                checkOne = false;
                angleVelocity = -50;
                roomba.setForward(0);
                roomba.turn(angleVelocity);
            }
        }
        if(turn){
            if(abs(angle) >= 90){
                roomba.turn(0);
                turn = false;
                angleVelocity = 0;
                checkTwo = true;
                roomba.setForward(150);
                velocity = 150;
            }
        }
        if(checkTwo){
            if(abs(position.first-checkPointTwo.first)<10 && abs(position.second-checkPointTwo.second)<10){
                velocity = 0;
                roomba.setForward(0);
                return;
            }

        }
        float angularVelocity = 180/PI*angleVelocity/radius;
        auto end = std::chrono::high_resolution_clock::now();    
        std::chrono::duration<float> diff = end-start; 
        angle += angularVelocity*(diff.count()-lastTime);
        position.first += std::cos(angle*3.14159/180)*velocity*(diff.count()-lastTime);
        position.second += std::sin(angle*3.14159/180)*velocity*(diff.count()-lastTime);
        std::cout << "angle: " << angle << std::endl;
        std::cout << "angularVelocity: " << angularVelocity << std::endl;
        std::cout << "Positions is: " << position.first << ","<< position.second << std::endl;
        lastTime = diff.count();
    }
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
    roomba.setForward(150);
    auto start = std::chrono::high_resolution_clock::now(); 
    auto end = std::chrono::high_resolution_clock::now(); 
    float lastTime = 0;
    while (true)
    {
        // This call waits until a new composite_frame is available
        // composite_frame holds a set of frames. It is used to prevent frame drops
        // The retunred object should be released with rs2_release_frame(...)
        rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, 5000, &e);


        // Returns the number of frames embedded within the composite frame
        int num_of_frames = rs2_embedded_frames_count(frames, &e);


        int i;
        for (i = 0; i < num_of_frames; ++i)
        {
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

            // Print the distance
            if(objectNear){
                roomba.turn(-50);
            }
            if(!objectNear && roomba.getTurningVelocity() != 0){
                roomba.setForward(150);
            }
            if(roomba.BumperAndWheels() > 0){
                roomba.setForward(0);
                return;
                rs2_pipeline_stop(pipeline, &e);

                // Release resources
                rs2_delete_pipeline_profile(pipeline_profile);
                rs2_delete_config(config);
                rs2_delete_pipeline(pipeline);
                rs2_delete_device(dev);
                rs2_delete_device_list(device_list);
                rs2_delete_context(ctx); 
            }    
            std::cout << "Is there an object in View: " << objectNear << std::endl;
            auto end = std::chrono::high_resolution_clock::now();    
            std::chrono::duration<float> diff = end-start; 
            roomba.update(diff.count()-lastTime);
            std::cout << "angle: " << roomba.getDirection() << std::endl;
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
    //Roomba roomba("/dev/tty.usbserial-DN0267K3", 115200);
    //std::cout << ((10&0xFF00)>>8) << std::endl;
    //std::cout << ((10&0x00FF)) << std::endl;
    //std::cout << (((-10)&0xFF00)>>8) << std::endl;
    //std::cout << (((-10)&0x00FF)) << std::endl;

    //roomba.stop();
    //roomba.start();
    //roomba.turn(-100);
}
