/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-23
Description:        ObjectDetector
*****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>

#include "Utility.h"
#include "CameraModel.h"
#include "Lane.h"
#include "GroundFilter.h"

using namespace cv;
using namespace std;


/*****************************************************************
Class Name: ObjectDetector
Description: Parent class to detect objects from disparity map
*****************************************************************/

class ObjectDetector{

    protected:

        int DISPARITY_CALC_TYPE;        // 0-median, 1-average, 2-max
        CameraModel* cm;
        vector<Object_2D> list_2D;
        int out_cx;
        int out_cy;

        /*Funciton Name: CalculateDisparity(Mat img, Rect box)                      */
        /*Description: Calculate the disparity from given box                             
                                    DISPARITY_CALC_TYPE:  0 - median
                                                                                        1 - average
                                                                                        2 - max                                          */
        /*Input: Mat img - disparity map
                        Rect box - calculated area                                                                   */   
        /*Output: int disparity value                                                                                */

        int CalculateDisparity(Mat img, Rect box);


        /*Funciton Name: RotateObject2D(float angle)                                         */
        /*Description: Rotate all 2D objects for an angle                                       */
        /*Input: float angle - rotation in degree,                                                        */   
        /*Output: void                                                                                                           */       

        void RotateObject2D(float angle);
    

    public:

        /*Funciton Name: ObjectDetector()                                          			               */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */   

        ObjectDetector();


        /*Funciton Name: ObjectDetector(CameraModel* cm)           	               */
        /*Description: Constructor with camera source                                         */
        /*Input: CameraModel* cm - pointer to a CameraModel                        */   

        ObjectDetector(CameraModel* cm);


        /*Funciton Name: List2D()                                          			                                  */
        /*Description: Return current 2D object list                                                 */
        /*Input: void                                                                                                               */   
        /*Output: vector<Object_2D> list_2D                                                              */        

        vector<Object_2D> List2D();


        /*Funciton Name: DetectObject2D(Mat disparity)	                                  */
        /*Description: Virtual function to detect object from dispariy            */
        /*Input: Mat disparity - disparity map                                                            */   
        /*Output: vector<Object_2D> list_2D                                                             */   

        virtual vector<Object_2D> DetectObject2D(Mat disparity)=0;

};



/*****************************************************************
Class Name: ContourDetector
Description: Detect object from contour
*****************************************************************/

class ContourDetector: public ObjectDetector{

    private:
        
        float min_depth;
        float depth_step;
        int num_of_waypoints;
        Lane* lane;
        int pre_median_kernel;
        int median_kernel;
        int min_disparity;
        int canny_threshold;
        string config;
        vector<float> list_depth;

        /*Funciton Name: Generate2DList(Mat&src, vector<Rect> boxes)      */
        /*Description: Generate 2D list from dispariy map and boxes              */
        /*Input: Mat src - disparity map                                                            
                        vector<Rect> boxes - detected boxes                                            */   
        /*Output: void                                                                                                           */  

        void Generate2DList(Mat&src, vector<Rect> boxes);


        /*Funciton Name: InitiateParameter(string config)                                  */
        /*Description: Read parameter from configuration file                          */
        /*Input: string config - path to configuration file                                       */   
        /*Output: void                                                                                                           */

        void InitiateParameter(string config);


    public:

        /*Funciton Name: ContourDetector()                                     			               */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */    

        ContourDetector();


        /*Funciton Name: ContourDetector(CameraModel* cm, string config)        */
        /*Description: Constructor from CameraModel and configuration                 */
        /*Input: CameraModel* cm - pointer to a CameraModel                        
                        string config - path to configuration file                                                    */      

        ContourDetector(CameraModel* cm, string config);


        /*Funciton Name: ContourDetector(CameraModel* cm, Lane* lane, string config)        */
        /*Description: Constructor from CameraModel, Lane and configuration                             */
        /*Input: CameraModel* cm - pointer to a CameraModel  
                        Lane* lane - pointer to a Lane                      
                        string config - path to configuration file                                                                            */        
        
        ContourDetector(CameraModel* cm, Lane* lane, string config);


        /*Funciton Name: DetectObject2D(Mat disparity)	                                  */
        /*Description: Detect object from dispariy                                                   */
        /*Input: Mat disparity - disparity map                                                            */   
        /*Output: vector<Object_2D> list_2D                                                             */          

        vector<Object_2D> DetectObject2D(Mat disparity);

};



/*****************************************************************
Class Name: UVDisparityDetector
Description: Detect object from UV-Disparity (experimental, incomplete)
*****************************************************************/

class UVDisparityDetector{

    private:

        OnlineGroundFilter ogf;
        Mat u_disp, v_disp;
        int uvd_min_count;


    public:

        /*Funciton Name: UVDisparityDetector()                             			                 */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */ 

        UVDisparityDetector();


        /*Funciton Name: UVDisparityDetector(string config)                              */
        /*Description: Constructor from configuration                                            */
        /*Input: string config - path to configuration file                                         */       

        UVDisparityDetector(string config);


        /*Funciton Name: DetectObject2D(Mat disparity)	                                  */
        /*Description: Detect object from dispariy                                                   */
        /*Input: Mat disparity - disparity map                                                            */   
        /*Output: vector<Object_2D> list_2D                                                             */ 

        vector<Object_2D> DetectObject2D(Mat disparity);

};