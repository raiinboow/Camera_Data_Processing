/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-18
Description:        Detection
*****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>

#include "Global.h"
#include "Lane.h"
#include "CameraModel.h"
#include "Disparity.h"
#include "GroundFilter.h"
#include "ObjectFilter.h"
#include "ObjectDetector.h"
#include "MonoDetector.h"

using namespace std;
using namespace cv;


/*****************************************************************
Class Name: Detection
Description: Create objects according to configuration and run detection
*****************************************************************/

class Detection{

    private:

        string config;                                              // path to configuration file
        vector<Object_2D> list_2D;                  // 2D object list
        vector<Object_3D> list_3D;                  // 3D object list

        // Camera model
        CameraModel* cm;                                
        MonoCameraModel* mcm;

        // Lane model
        CameraLane* lane;
        MonoLane* mlane;

        // Disparity calculator
        DisparitySGBM sgbm;
        DisparityBM bm;
        DisparityELAS elas;

        // Ground filter
        MonoGroundFilter mgf;
        HoughGroundFilter hgf;
        RANSACGroundFilter rgf;

        // Object detector
        ContourDetector cd;
        UVDisparityDetector uvd;

        // Objct filter
        NMSFilter nmsf;
        LaneFilter lf;
        MonoLaneFilter mlf;
        
        // Constants
        Mat Q;
        int offset_x;
        int offset_y;
        int min_disparity;

        // Control variables
        bool FILTER_GROUND;                                      // 1 - filter ground
        bool FILTER_BACKGROUND;                           // 1 - filter background
        bool FILTER_LANE;                                              // 1- filter objects not on lane  
        int DISPARITY_TYPE;                                           // 0 - SGBM, 1- BM, 2 - ELAS
        int DETECTOR_TYPE;                                          // 0 - Contour, 1 - UVdisparity
        int GROUND_FILTER_TYPE;                              // 0 - Mono, 1 - Online
        int ONLINE_GROUND_FILTER_TYPE;           // 0 - Hough, 1 - RANSAC
        int NUM_OF_RANSAC_FILTERING;                // Iterations of RANSAC ground filtering


        /*Funciton Name: ReadParameters(string config)                                    */
        /*Description: Read parameters from configuration file                        */
        /*Input: string config - path to configuration file                                       */        
        /*Output: void                                                                                                          */  

        void ReadParameters(string config);


        /*Funciton Name: Initialize(string config)                                                   */
        /*Description: Initialize objects according to configuration                */
        /*Input: string config - path to configuration file                                      */        
        /*Output: void                                                                                                          */

        void Initialize(string config);


        /*Funciton Name: UpdateMinDisparity()												        */
        /*Description: Update min_disparity according to lane depth            */
        /*Input: void                                                                                                               */        
        /*Output: void                                                                                                           */     

        void inline UpdateMinDisparity();
    

    public:

        /*Funciton Name: Detection()                                          								     */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */  
        
        Detection();
        
        
        /*Funciton Name: Detection(CameraLane* lane, string config)           */
        /*Description: Constructor from configuration file, stereo model      */
        /*Input: CameraLane* lane - pointer to lane model
                        string config - path to configuration file                                        */  

        Detection(CameraLane* lane, string config);


        /*Funciton Name: Detection(MonoLane* mlane, string config)           */
        /*Description: Constructor from configuration file, mono model       */
        /*Input: MonoLane* mlane - pointer to monolane model
                        string config - path to configuration file                                        */  
        
        Detection(MonoLane* mlane, string config);


        /*Funciton Name: Update2D(Mat left, Mat right)                                     */
        /*Description: Detect 2D objects from rectified image pair                 */
        /*Input: Mat left - left rectified image                                      
                        Mat right - right rectified image														 */        
        /*Output: void                                                                                                          */

        void Update2D(Mat left, Mat right);


        /*Funciton Name: Update3D()                                                                          */
        /*Description: Project 2D objects to 3D											               */
        /*Input: void																				                              */        
        /*Output: void                                                                                                          */

        void Update3D();


        /*Funciton Name: PrintList2D()						                                                   */
        /*Description: Print 2D object list in the console                    		             */
        /*Input: void                                                                                                               */        
        /*Output: void                                                                                                           */      

        void PrintList2D();


        /*Funciton Name: PrintList2D(vector<Object_2D> list)					      */
        /*Description: Print input 2D object list in the console                           */
        /*Input: void                                                                                                               */        
        /*Output: void                                                                                                           */     
        
        static void PrintList2D(vector<Object_2D> list);


        /*Funciton Name: PrintList3D()						                                                   */
        /*Description: Print 3D object list in the console                    		             */
        /*Input: void                                                                                                               */        
        /*Output: void                                                                                                           */        

        void PrintList3D();


        /*Funciton Name: PrintList3D(vector<Object_3D> list)					      */
        /*Description: Print input 3D object list in the console                           */
        /*Input: void                                                                                                               */        
        /*Output: void                                                                                                           */     

        static void PrintList3D(vector<Object_3D> list);


        /*Funciton Name: List2D()													   	 					        */
        /*Description: Return current 2D object list     					                         */
        /*Input: void                                                                                                               */        
        /*Output: vector<Object_2D> list_2D                                                             */     

        vector<Object_2D> List2D();


        /*Funciton Name: List3D()													   	 					        */
        /*Description: Return current 3D object list     					                         */
        /*Input: void                                                                                                               */        
        /*Output: vector<Object_3D> list_3D                                                             */  

        vector<Object_3D> List3D();       

};

