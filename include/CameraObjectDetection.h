/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-18
Description:        CameraObjectDetection
*****************************************************************/

# pragma once

#include <fstream>
#include <opencv2/viz.hpp>

#include "Global.h"
#include "Detection.h"
#include "Disparity.h"
#include "Lane.h"
#include "RScamera.h"
#include "Utility.h"
#include "Visualization.h"
#include "GroundFilter.h"
#include "ObjectDetector.h"
#include "MonoDetector.h"

using namespace cv;
using namespace std;


/*Class Name: CameraObjectDetection*/
/*Description: Create objects according to configuration and run pipeline*/
class CameraObjectDetection{

    private:

        // Final output
        vector<Object_3D> list_3D;

        // Control variables
        bool READ_FROM_FILE;                        // 1 - read .bag from PATH_FILE, 0 - live
        bool RECORD;                                            // 1 - record live data
        bool SAVE_BAG;                                       // 1 - save .bag file
        bool SAVE_VIDEO;                                   // 1- save .avi video
        bool RUN_ALGORITHM;                       // 1- run detection algorithm
        bool IS_MONO;                                        // 1 - use mono camera geometry
        int SAMPLE_RATE;                                  // downsample rate
        bool MONO_DETECTION;                    // 1- run mono detection algorithm

        // Detection objects
        Motion abs_motion;
        CameraModel t265;
        MonoCameraModel mt265;
        RScamera rs_t265;
        MonoLane mlane;
        CameraLane lane;
        Detection detector;
        Visualization vis;
        Transform cam_vehicle;
        MonoDetector md;

        // Paths
        string PATH_STEREO_CALIBRATION;
        string PATH_DETECTION_CONFIG;
        string PATH_VISUAL;
        string PATH_MONTAGE;
        string PATH_LANE;
        string PATH_RECORD;
        string PATH_FILE;

        /*Funciton Name: InitObjects()                                                                        */
        /*Description: Initialize objects according to configuration                */
        /*Input: void                                                                                                             */
        /*Output: void                                                                                                          */
        
        void InitObjects();


        /*Funciton Name: InitMotion()                                                                          */
        /*Description: Initialize abs_motion according to configuration       */
        /*Input: void                                                                                                              */
        /*Output: void                                                                                                          */       

        void InitMotion();


        /*Funciton Name: InitVisual()                                                                             */
        /*Description: Initialize global visualization variables                            */
        /*Input: void                                                                                                              */
        /*Output: void                                                                                                           */    

        void InitVisual();


        /*Funciton Name: RunAlgorithm(Mat left, Mat right)                              */
        /*Description: Run detection algorithm                                                        */
        /*Input: Mat left - left rectified image                                                             */
        /*              Mat right - right rectified image                                                       */        
        /*Output: void                                                                                                           */    

        void RunAlgorithm(Mat left, Mat right);


        /*Funciton Name: PrintProcessingTime()                                                     */
        /*Description: Print processing time in the console                                 */
        /*Input: void                                                                                                               */        
        /*Output: void                                                                                                           */         

        void PrintProcessingTime();

    
    public:

        /*Funciton Name: CameraObjectDetection()                                               */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */          

        CameraObjectDetection();


        /*Funciton Name: CameraObjectDetection(string config)                     */
        /*Description: Constructor from configuration file                                    */
        /*Input: string config - path of configuration file                                        */                 

        CameraObjectDetection(string config);


        /*Funciton Name: Run()                                                                                        */
        /*Description: Run detection pipeline                                                            */
        /*Input: void                                                                                                               */        
        /*Output: void                                                                                                           */           

        void Run();

};



