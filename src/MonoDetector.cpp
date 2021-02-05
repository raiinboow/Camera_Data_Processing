/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        MonoDetector
*****************************************************************/

# include "MonoDetector.h"


/*****************************************************************
Class Name: MonoDetector
Description: Class to detect object from a single image(not complete)
*****************************************************************/

/*Funciton Name: Detection()                                          								     */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */     

MonoDetector::MonoDetector(){

    canny_low = 30;
    canny_high = 50;

}


/*Funciton Name: Detect(Mat img)                                								   */
/*Description: Detect object from a single image                                      */
/*Input: Mat img - input image                                                                           */     
/*Output: void                                                                                                            */

void MonoDetector::Detect(Mat img){

    GaussianBlur(img, img, Size(5,5), 2, 0);
    Mat edges;
    threshold(img, img, 100, 255, THRESH_BINARY);

    imshow("thresholded", img);
    equalizeHist(img, img);

    Canny(img, edges, canny_low, canny_high);

    imshow("Edges" ,edges);

}

