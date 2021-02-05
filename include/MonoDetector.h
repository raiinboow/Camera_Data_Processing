/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        MonoDetector
*****************************************************************/

# pragma once

#include <opencv2/opencv.hpp>

using namespace cv;


/*****************************************************************
Class Name: MonoDetector
Description: Class to detect object from a single image (not complete)
*****************************************************************/

class MonoDetector{

    private:

        // Canny edge parameters
        int canny_low;
        int canny_high;

    public:

        /*Funciton Name: Detection()                                          								     */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */     

        MonoDetector();


        /*Funciton Name: Detect(Mat img)                                								   */
        /*Description: Detect object from a single image                                      */
        /*Input: Mat img - input image                                                                           */     
        /*Output: void                                                                                                            */    

        void Detect(Mat img);

};