/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        GroundFilter
*****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>

#include "CameraModel.h"

using namespace std;
using namespace cv;


/*****************************************************************
Class Name: GroundFilter
Description: Parent class to filter ground
*****************************************************************/

class GroundFilter{

    public:

        /*Funciton Name: GroundFilter()                                          						     */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */    

        GroundFilter();


        /*Funciton Name:FilterGround(Mat& disparity)                                                    */                            
        /*Description: virtual function to filter ground                                                      */
        /*Input: Mat& disparity - disparity map                                                                    */    
        /*Output: void                                                                                                                      */

        virtual void FilterGround(Mat& disparity) = 0;


        /*Funciton Name: FilterBackground(Mat& disparity, int min_disparity)    */                               						     
        /*Description: Filter pixels smalle rthan min_disparity                                      */
        /*Input: Mat& disparity - disparity map 
                        int min_disparity - minimal disparity                                                        */    
        /*Output: void                                                                                                                       */

        void FilterBackground(Mat& disparity, int min_disparity);

};


/*****************************************************************
Class Name: VDisparityGroundFilter
Description: ground filter in v-disparity space
*****************************************************************/

class VDisparityGroundFilter: public GroundFilter{

    protected: 

        float ground_k;
        float ground_b;
        int ground_tolerance;

    public:

        /*Funciton Name: VDisparityGroundFilter()                                          		   */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */    

        VDisparityGroundFilter();


        /*Funciton Name:FilterGround(Mat& disparity)                                             */                            
        /*Description: virtual function to filter ground                                                */
        /*Input: Mat& disparity - disparity map                                                               */    
        /*Output: void                                                                                                                 */        

        void FilterGround(Mat& disparity);

};


/*****************************************************************
Class Name: MonoGroundFilter
Description: ground filter using mono-geometry
*****************************************************************/

class MonoGroundFilter: public VDisparityGroundFilter{

    private:

        MonoCameraModel* mcm;
        // bool is_stable;
        // void Rotate(Mat& img, float angle);

    public:

        /*Funciton Name: MonoGroundFilter()                                          		          */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */    

        MonoGroundFilter();


        /*Funciton Name: MonoGroundFilter(MonoCameraModel* mcm, string config)	   */
        /*Description:  constructor with mono camera model and configuration file            */
        /*Input: MonoCameraModel* mcm - pointer to a MonoCameraModel
                        string config - path to configuration file                                                                    */      

        MonoGroundFilter(MonoCameraModel* mcm, string config);


        /*Funciton Name:UpdateGroundPlane()                                                            */                            
        /*Description: upadate ground_k and ground_b                                             */
        /*Input: void                                                                                                                     */    
        /*Output: void                                                                                                                 */        

        void UpdateGroundPlane();

};



/*****************************************************************
Class Name: OnlineGroundFilter
Description: ground plane is calculated from v-disparity online
*****************************************************************/


class OnlineGroundFilter: public VDisparityGroundFilter{

    protected:

        // Calculation parameters
        int min_count;
        int max_disparity;
        int min_disparity;
        
        // Projected disparity maps
        Mat v_disparity;
        Mat u_disparity;


        /*Funciton Name: CalcUVdisp(Mat disparity, bool u, bool v, int start_row, int start_column)      */
        /*Description: update u_disparity and v_disparity from disparity map,
                                        a trapezoid area from (start_row, start_column)                                                              */
        /*Input: Mat disparity - disparity map
                        bool u - calculate u-disparity
                        bool v - calculate v- disparity 
                        int start_row - row of the top-left point
                        int start_column - column of the top-left point                                                                                */  
        /*Output: void                                                                                                                                                                 */

        void CalcUVdisp(Mat disparity, bool u, bool v, int start_row=0, int start_column=0);


    public:

        /*Funciton Name: OnlineGroundFilter()                                          		          */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */    

        OnlineGroundFilter();


        /*Funciton Name: OnlineGroundFilter(string config)                 		          */
        /*Description: constructor from configuration file                                     */
        /*Input: string config - path to configuration file                                        */         
        OnlineGroundFilter(string config);


        /*Funciton Name: Udisparity(Mat disparity, int min_disp, int max_disp, int min_count)              */
        /*Description: calculate and return u-disparity                                                                                                 */
        /*Input: Mat disparity - disparity map
                        int min_disp - minimal disparity
                        int max_disp - maximal disparity
                        int min_count - minimal count                                                                                                                 */  
        /*Output: Mat u_disparity                                                                                                                                             */

        Mat Udisparity(Mat disparity, int min_disp, int max_disp, int min_count);


        /*Funciton Name: Vdisparity(Mat disparity, int min_disp, int max_disp, int min_count)              */
        /*Description: calculate and return v-disparity                                                                                                 */
        /*Input: Mat disparity - disparity map
                        int min_disp - minimal disparity
                        int max_disp - maximal disparity
                        int min_count - minimal count                                                                                                                 */  
        /*Output: Mat v_disparity                                                                                                                                             */

        Mat Vdisparity(Mat disparity, int min_disp, int max_disp, int min_count);

};



/*****************************************************************
Class Name: HoughGroundFilter
Description: ground plane detected from Hough transform
*****************************************************************/

class HoughGroundFilter: public OnlineGroundFilter{

    private:

        // Parameter for Hough line detection
        int hough_min_count;
        int hough_min_length;
        int hough_max_gap;

    
    public:

        /*Funciton Name: HoughGroundFilter()                                         		         */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */   

        HoughGroundFilter();


        /*Funciton Name: HoughGroundFilter(string config)                 		         */
        /*Description: constructor from configuration file                                     */
        /*Input: string config - path to configuration file                                        */  

        HoughGroundFilter(string config);


        /*Funciton Name: UpdateGroundPlane(Mat disparity)           		              */
        /*Description: update ground_k and ground_b from disparity map      */
        /*Input: Mat disparity - diaprity map                                                                     */  
        /*Output: void                                                                                                                  */

        void UpdateGroundPlane(Mat disparity);

};



/*****************************************************************
Class Name: RANSACGroundFilter
Description: ground plane detected from RANSAC algorithm
*****************************************************************/

class RANSACGroundFilter: public OnlineGroundFilter{

    private:

        int num_samples;
        float distance_threshold;
        int start_row;
        int start_column;
        int max_iteration;

        float last_ground_k;
        float last_ground_b;

        int num_median_filter;

        vector<Point2i> list_point;
        vector<Point2i> list_sample_point;
        vector<Point2i> list_inliers;

        LineModel ground_line;

        // MedianFilter for ground_k and ground_b   
        MedianFilter mfk;
        MedianFilter mfb;

        // int num_data;
        int num_inliers;
        int max_cost;


        /*Funciton Name: UpdateList()                                                             		              */
        /*Description: update list_point from v_disparity                                            */
        /*Input: void                                                                                                                      */  
        /*Output: void                                                                                                                  */
        void UpdateList();


        /*Funciton Name: GetRandomSamples()                                                                      */
        /*Description: get samples from list_point and save to list_sample_point    */
        /*Input: void                                                                                                                               */  
        /*Output: void                                                                                                                           */        

        void GetRandomSamples();


        /*Funciton Name: ComputeLine()                                                                                     */
        /*Description: compute a line from list_sample_point                                           */
        /*Input: void                                                                                                                               */  
        /*Output: LineModel line                                                                                                      */       

        LineModel ComputeLine();


        /*Funciton Name: ComputeCost(LineModel line)                                                      */
        /*Description: find inliers for a given line and count newly added inliers       */
        /*Input: void                                                                                                                               */  
        /*Output: LineModel line                                                                                                      */

        int ComputeCost(LineModel line);


        /*Funciton Name: ComputeDistance(LineModel line, Point p)                            */
        /*Description: compute point to line distance                                                            */
        /*Input: LineModel line - line
                        Point p - point                                                                                                           */  
        /*Output: float distance                                                                                                        */        

        float ComputeDistance(LineModel line, Point p);


    public:

        /*Funciton Name: RANSACGroundFilter()                                            	       */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */   

        RANSACGroundFilter();


        /*Funciton Name: HoughGroundFilter(string config)                 		         */
        /*Description: constructor from configuration file                                     */
        /*Input: string config - path to configuration file                                        */         

        RANSACGroundFilter(string config);


        /*Funciton Name: UpdateGroundPlane(Mat disparity)           		              */
        /*Description: update ground_k and ground_b from disparity map      */
        /*Input: Mat disparity - diaprity map                                                                     */  
        /*Output: void                                                                                                                  */
        
        void UpdateGroundPlane(Mat disparity);

};