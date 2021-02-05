/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-18
Description:        Detection
*****************************************************************/

#include<fstream>

#include "Global.h"
#include "Disparity.h"


/*****************************************************************
Class Name: Disparity
Description: Parent class for all disparity class
*****************************************************************/

/*Funciton Name: Disparity()                                                                                */
/*Description: Default constructor                                                                     */
/*Input: void                                                                                                                 */    

Disparity::Disparity(){}


/*Funciton Name: Disparity(string config)                                                      */
/*Description: Constructor from configuration file                                     */
/*Input: string config - configurateion file                                                       */    

Disparity::Disparity(string config){


    FileStorage fs(config, FileStorage::READ);

    fs["pre_gaussian_kernel"] >> pre_gaussian_kernel;
    fs["pre_median_kernel"] >> pre_median_kernel;
    fs["max_disparity"] >> max_disparity;
    fs["window_size"] >> window_size;
    fs["prefilter_size"] >> prefilter_size;
    fs["prefilter_cap"] >> prefilter_cap;
    fs["texture_threshold"] >> texture_threshold;
    fs["uniqueness_ratio"] >> uniqueness_ratio;

}


/*Funciton Name: Preprocessing(Mat& img_in, Mat& img_out)           */
/*Description: histogram equalization, median and Gaussian              */
/*Input: Mat& img_in - input image
                 Mat& img_out - output image                                                             */    
/*Output: void                                                                                                              */  

void Disparity::Preprocessing(Mat& img_in, Mat& img_out){

    equalizeHist(img_in, img_out);
    if(pre_median_kernel>0) medianBlur(img_out, img_out, pre_median_kernel);
	if(pre_gaussian_kernel>0) GaussianBlur(img_out, img_out, Size(pre_gaussian_kernel, pre_gaussian_kernel),0);

}



/*****************************************************************
Class Name: DisparitySGBM
Description: Calculate disparity map with SGBM algorithm
*****************************************************************/

/*Funciton Name: DisparitySGBM()                                                                   */
/*Description: Default constructor                                                                     */
/*Input: void                                                                                                                 */    

DisparitySGBM::DisparitySGBM(){}


/*Funciton Name: DisparitySGBM(string config)                                          */
/*Description: Constructor from configuration file                                     */
/*Input: string config - configurateion file                                                       */    

DisparitySGBM::DisparitySGBM(string config): Disparity(config){

    FileStorage fs(config, FileStorage::READ);
    fs["wls_lambda"] >> wls_lambda;
    fs["wls_sigma"] >> wls_sigma;

    InitMatcher();

}


/*Funciton Name: InitMatcher()                                                                          */
/*Description: initialize matchers                                                                       */
/*Input: void                                                                                                                */    
/*Output: void                                                                                                            */    

void DisparitySGBM::InitMatcher(){

    sSGBM_l = StereoSGBM::create(0, max_disparity, window_size);

    sSGBM_l->setP1(8 * window_size * window_size);											
    sSGBM_l->setP2(32 * window_size * window_size);
    sSGBM_l->setPreFilterCap(prefilter_cap);
    sSGBM_l->setMode(StereoSGBM::MODE_SGBM_3WAY);
    sSGBM_l->setUniquenessRatio(uniqueness_ratio);

    wls_filter = ximgproc::createDisparityWLSFilter(sSGBM_l);
    wls_filter->setLambda(wls_lambda);
    wls_filter->setSigmaColor(wls_sigma);

    sSGBM_r = ximgproc::createRightMatcher(sSGBM_l);

}


/*Funciton Name: CalculateDispMap(Mat img_l, Mat img_r)                  */
/*Description: calculate disparity map                                                             */
/*Input: Mat img_l - left image
                    Mat img_r - right image                                                                         */    
/*Output: Mat disparity                                                                                             */  

Mat DisparitySGBM::CalculateDispMap(Mat img_l, Mat img_r){

    Mat temp_l, temp_r;
    Mat output = Mat::zeros({ img_l.cols, img_r.rows }, CV_8UC1);

    Preprocessing(img_l, temp_l);
    Preprocessing(img_r, temp_r);

    Mat left = Mat::zeros({ temp_l.cols + max_disparity, temp_l.rows }, CV_8UC1);
	Mat right = Mat::zeros({ temp_r.cols + max_disparity, temp_r.rows }, CV_8UC1);

	temp_l.copyTo(left(Rect(max_disparity, 0,temp_l.cols, temp_l.rows)));
	temp_r.copyTo(right(Rect(max_disparity, 0, temp_r.cols, temp_r.rows)));

    Mat disp_filter;

    Mat disp_l, disp_r;
    sSGBM_l->compute(left, right, disp_l);
    sSGBM_r->compute(right, left, disp_r);
    wls_filter->filter(disp_l, left, disp_filter, disp_r);

    Mat temp;
    ximgproc::getDisparityVis(disp_filter, temp, 1);
    temp(Rect(max_disparity, 0, output.cols, output.rows)).copyTo(output);

    return output;


}



/*****************************************************************
Class Name: DisparityBM
Description: Calculate disparity map with BM algorithm
*****************************************************************/

/*Funciton Name: DisparityBM()                                                                         */
/*Description: Default constructor                                                                     */
/*Input: void                                                                                                                 */    

DisparityBM::DisparityBM(){}


/*Funciton Name: DisparityBM(string config)                                               */
/*Description: Constructor from configuration file                                     */
/*Input: string config - configurateion file                                                       */    

DisparityBM::DisparityBM(string config): Disparity(config){

    FileStorage fs(config, FileStorage::READ);
    fs["speckle_window"] >> speckle_window;
    fs["speckle_range"] >> speckle_range;

    InitMatcher();

}


/*Funciton Name: InitMatcher()                                                                          */
/*Description: initialize matchers                                                                       */
/*Input: void                                                                                                                */    
/*Output: void                                                                                                            */    

void DisparityBM::InitMatcher(){

    sBM = StereoBM::create(max_disparity, window_size);

    sBM->setPreFilterType(StereoBM::PREFILTER_XSOBEL);
    sBM->setPreFilterSize(prefilter_size);
    sBM->setPreFilterCap(prefilter_cap);
    sBM->setTextureThreshold(texture_threshold);
    sBM->setUniquenessRatio(uniqueness_ratio);
    sBM->setSpeckleWindowSize(speckle_window);
    sBM->setSpeckleRange(speckle_range);

}


/*Funciton Name: CalculateDispMap(Mat img_l, Mat img_r)                  */
/*Description: calculate disparity map                                                             */
/*Input: Mat img_l - left image
                    Mat img_r - right image                                                                         */    
/*Output: Mat disparity                                                                                             */  

Mat DisparityBM::CalculateDispMap(Mat img_l, Mat img_r){

    Mat temp_l, temp_r; 
    Mat output = Mat::zeros({ img_l.cols, img_r.rows }, CV_8UC1);

    Preprocessing(img_l, temp_l);
    Preprocessing(img_r, temp_r);

    Mat left = Mat::zeros({ temp_l.cols + max_disparity, temp_l.rows }, CV_8UC1);
	Mat right = Mat::zeros({ temp_r.cols + max_disparity, temp_r.rows }, CV_8UC1);

	temp_l.copyTo(left(Rect(max_disparity, 0,temp_l.cols, temp_l.rows)));
	temp_r.copyTo(right(Rect(max_disparity, 0, temp_r.cols, temp_r.rows)));

    Mat disp_filter;

     sBM->compute(left, right, disp_filter);

     Mat temp;
    ximgproc::getDisparityVis(disp_filter, temp, 1);
    temp(Rect(max_disparity, 0, output.cols, output.rows)).copyTo(output);

    return output;

}



/*****************************************************************
Class Name: DisparityELAS
Description: Calculate disparity map with ELAS algorithm
*****************************************************************/

/*Funciton Name: DisparityELAS()                                                                     */
/*Description: Default constructor                                                                     */
/*Input: void                                                                                                                 */  

DisparityELAS::DisparityELAS(){

    disp_elas = StereoEfficientLargeScale(0,128);

}


/*Funciton Name: DisparityELAS(string config)                                            */
/*Description: Constructor from configuration file                                     */
/*Input: string config - configurateion file                                                       */    

DisparityELAS::DisparityELAS(string config):Disparity(config){

    InitMatcher();

}


/*Funciton Name: InitMatcher()                                                                          */
/*Description: initialize matchers                                                                       */
/*Input: void                                                                                                                */    
/*Output: void                                                                                                            */  

void DisparityELAS::InitMatcher(){

    disp_elas = StereoEfficientLargeScale(0,max_disparity);

	// we can set various parameter
		//elas.elas.param.ipol_gap_width=;
		//elas.elas.param.speckle_size=getParameter("speckle_size");
		//elas.elas.param.speckle_sim_threshold=getParameter("speckle_sim");

}


/*Funciton Name: CalculateDispMap(Mat img_l, Mat img_r)                  */
/*Description: calculate disparity map                                                             */
/*Input: Mat img_l - left image
                    Mat img_r - right image                                                                         */    
/*Output: Mat disparity                                                                                             */  

Mat DisparityELAS::CalculateDispMap(Mat img_l, Mat img_r){

    Mat output, temp, temp_l, temp_r;
    Preprocessing(img_l, temp_l);
    Preprocessing(img_r, temp_r);

	disp_elas(temp_l,temp_r,temp,max_disparity);
	temp.convertTo(output,CV_8U,1.0/8);

    return output;

}


