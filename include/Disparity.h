/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        Disparity
*****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include "CameraModel.h"
#include "Utility.h"
#include "ELAS/StereoEfficientLargeScale.h"

using namespace std;
using namespace cv;


/*****************************************************************
Class Name: Disparity
Description: Parent class for all disparity class
*****************************************************************/

class Disparity{

    protected:

        // Preprocessing
        int pre_gaussian_kernel;
        int pre_median_kernel;

        // Essential parameter
        int max_disparity;
        int window_size;

        // Optional parameter
        int prefilter_size;
        int prefilter_cap;
        int uniqueness_ratio;
        int texture_threshold;


        /*Funciton Name: Preprocessing(Mat& img_in, Mat& img_out)           */
        /*Description: histogram equalization, median and Gaussian              */
        /*Input: Mat& img_in - input image
                        Mat& img_out - output image                                                             */    
        /*Output: void                                                                                                            */  

        void Preprocessing(Mat& img_in, Mat& img_out);


        /*Funciton Name: InitMatcher()                                                                          */
        /*Description: virtual function to initialize matchers                                */
        /*Input: void                                                                                                                */    
        /*Output: void                                                                                                            */  

        virtual void InitMatcher()=0;
    

    public:

        /*Funciton Name: Disparity()                                                                                */
        /*Description: Default constructor                                                                     */
        /*Input: void                                                                                                                 */    

        Disparity();


        /*Funciton Name: Disparity(string config)                                                      */
        /*Description: Constructor from configuration file                                     */
        /*Input: string config - configurateion file                                                       */           

        Disparity(string config);


         /*Funciton Name: CalculateDispMap(Mat img_l, Mat img_r)               */
        /*Description: virtual function to calculate disparity map                     */
        /*Input: Mat img_l - left image
                         Mat img_r - right image                                                                         */    
        /*Output: Mat disparity                                                                                          */  

        virtual Mat CalculateDispMap(Mat img_l, Mat img_r)=0;

};



/*****************************************************************
Class Name: DisparitySGBM
Description: Calculate disparity map with SGBM algorithm
*****************************************************************/

class DisparitySGBM: public Disparity{

    private:

        // Mathcer
        Ptr<StereoSGBM> sSGBM_l;
        Ptr<ximgproc::DisparityWLSFilter> wls_filter;
        Ptr<StereoMatcher> sSGBM_r;

        // Parameter for WLS matcher
        int wls_lambda;
        int wls_sigma;

        /*Funciton Name: InitMatcher()                                                                          */
        /*Description: initialize matchers                                                                       */
        /*Input: void                                                                                                                */    
        /*Output: void                                                                                                            */          

        void InitMatcher();


    public:

        /*Funciton Name: DisparitySGBM()                                                                   */
        /*Description: Default constructor                                                                     */
        /*Input: void                                                                                                                 */    

        DisparitySGBM();


        /*Funciton Name: DisparitySGBM(string config)                                          */
        /*Description: Constructor from configuration file                                     */
        /*Input: string config - configurateion file                                                       */          

        DisparitySGBM(string config);


        /*Funciton Name: CalculateDispMap(Mat img_l, Mat img_r)                  */
        /*Description: calculate disparity map                                                             */
        /*Input: Mat img_l - left image
                         Mat img_r - right image                                                                         */    
        /*Output: Mat disparity                                                                                          */  

        Mat CalculateDispMap(Mat img_l, Mat img_r);


};


/*****************************************************************
Class Name: DisparityBM
Description: Calculate disparity map with BM algorithm
*****************************************************************/

class DisparityBM: public Disparity{

    private:

        // Matcher
        Ptr<StereoBM> sBM;

        // Speckle parameter
        int speckle_window;
        int speckle_range;

        /*Funciton Name: InitMatcher()                                                                          */
        /*Description: initialize matchers                                                                       */
        /*Input: void                                                                                                                */    
        /*Output: void                                                                                                            */   

        void InitMatcher();

    public:

        /*Funciton Name: DisparityBM()                                                                         */
        /*Description: Default constructor                                                                     */
        /*Input: void                                                                                                                 */    

        DisparityBM();


        /*Funciton Name: DisparityBM(string config)                                               */
        /*Description: Constructor from configuration file                                     */
        /*Input: string config - configurateion file                                                       */    

        DisparityBM(string config);


        /*Funciton Name: CalculateDispMap(Mat img_l, Mat img_r)                  */
        /*Description: calculate disparity map                                                             */
        /*Input: Mat img_l - left image
                            Mat img_r - right image                                                                         */    
        /*Output: Mat disparity                                                                                             */         

        Mat CalculateDispMap(Mat img_l, Mat img_r);

};



/*****************************************************************
Class Name: DisparityELAS
Description: Calculate disparity map with ELAS algorithm
*****************************************************************/

class DisparityELAS:public Disparity{

    private:

        // Matcher
        StereoEfficientLargeScale disp_elas;

        /*Funciton Name: InitMatcher()                                                                          */
        /*Description: initialize matchers                                                                       */
        /*Input: void                                                                                                                */    
        /*Output: void                                                                                                            */  

        void InitMatcher();


    public:

        /*Funciton Name: DisparityELAS()                                                                     */
        /*Description: Default constructor                                                                     */
        /*Input: void                                                                                                                 */  

        DisparityELAS();

        
        /*Funciton Name: DisparityELAS(string config)                                            */
        /*Description: Constructor from configuration file                                     */
        /*Input: string config - configurateion file                                                       */         

        DisparityELAS(string config);


        /*Funciton Name: CalculateDispMap(Mat img_l, Mat img_r)                  */
        /*Description: calculate disparity map                                                             */
        /*Input: Mat img_l - left image
                            Mat img_r - right image                                                                         */    
        /*Output: Mat disparity                                                                                             */  
        
        Mat CalculateDispMap(Mat img_l, Mat img_r);

};