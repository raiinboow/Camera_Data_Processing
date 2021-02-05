/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-23
Description:        RScamera
*****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include "CameraModel.h"
#include "Utility.h"

using namespace std;
using namespace cv;
using namespace rs2;


/*****************************************************************
Class Name: RScamera
Description: Class to retrive rawdata from RealSense camera
*****************************************************************/

class RScamera{

    private:

        pipeline pipe;
        frameset frames;

        bool running;

        int raw_width;
        int raw_height;
        int raw_fps;
        int sample_rate;
        float fps;

        Mat raw_l, raw_r;
        config cfg;
        Motion imu;

        bool record;
        bool save_bag;
        bool save_video;
        string path_record;
        string file_bag;
        string file_left;
        string file_right;
        VideoWriter writer_left;
        VideoWriter writer_right;

        bool read_from_file;
        string path_file;


        /*Funciton Name: UpdateRawImage()                                                          */
        /*Description: Update raw image                                                                    */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                          */

        void UpdateRawImage();


        /*Funciton Name: UpdateIMU()                                                                      */
        /*Description: Update IMU data                                                                      */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                         */
        void UpdateIMU();


        /*Funciton Name: int_to_string(int i)                                                            */
        /*Description: convert integer to double digit string                              */
        /*Input: int i - integer to convert                                                                      */        
        /*Output: string resul                                                                                           */

        inline string int_to_string(int i);


    public:

        /*Funciton Name: RScamera()                                          								  */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */ 

        RScamera();


        /*Funciton Name: RScamera(string config, int sample_rate)                                                    */
        /*Description: constructor from configuration and specified sample_rate                         */
        /*Input: string config - path to configuration file
                        int sample_rate - (down) sample rate, actual_FPS = raw_FPS/sample_rate     */    

        RScamera(string config, int sample_rate=1);


        /*Funciton Name: Start()                                                                                    */
        /*Description: Start pipeline                                                                              */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                          */

        void Start();


        /*Funciton Name: Stop()                                                                                    */
        /*Description: Stop pipeline                                                                              */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                          */

        void Stop();

        /*Funciton Name: Pause()                                                                                    */
        /*Description: Pause pipeline                                                                              */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                          */       

        void Pause();


        /*Funciton Name: Running()                                                                            */
        /*Description: Return whether pipeline is running                                 */
        /*Input: void                                                                                                             */        
        /*Output: bool running                                                                                       */

        bool Running();


        /*Funciton Name: UpdateFrames()                                                                */
        /*Description: Retrive frame from pipeline and update raw data     */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                          */        

        void UpdateFrames();


        /*Funciton Name: EnableRecord(string path, bool save_bag, bool save_video)  */
        /*Description: Enable recording                                                                                                 */
        /*Input: string path - folder to save the recorded files
                        bool save_bag - save the raw data bag
                        bool save_video - save video in .avi                                                                          */        
        /*Output: void                                                                                                                                      */       

        void EnableRecord(string path, bool save_bag, bool save_video);


        /*Funciton Name: DisableRecord()                                                                */
        /*Description: Disable recording                                                                     */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                         */        

        void DisableRecord();


        /*Funciton Name: EnableRead(string path)                                               */
        /*Description: Enable reading from a .bag file                                          */
        /*Input: string path - path to a .bag file                                                        */        
        /*Output: void                                                                                                         */        

        void EnableRead(string path);


        /*Funciton Name: DisableRead()                                                                     */
        /*Description: Disable reading                                                                          */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                         */

        void DisableRead();


        /*Funciton Name: RawLeft()                                                                              */
        /*Description: Get raw image left                                                                    */
        /*Input: void                                                                                                             */        
        /*Output: Mat raw_l                                                                                              */

        Mat RawLeft();


        /*Funciton Name: RawRight()                                                                           */
        /*Description: Get raw image right                                                                 */
        /*Input: void                                                                                                             */        
        /*Output: Mat raw_r                                                                                              */        

        Mat RawRight();


        /*Funciton Name: IMU()                                                                                      */
        /*Description: Get IMU data                                                                               */
        /*Input: void                                                                                                             */        
        /*Output: Motion imu                                                                                           */        
        
        Motion IMU();

};
