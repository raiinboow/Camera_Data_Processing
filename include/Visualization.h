/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-24
Description:        Visualization
*****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>

#include "Lane.h"

using namespace std;
using namespace cv;


/*****************************************************************
Class Name: Visualization
Description: Class to visualize lane and object
*****************************************************************/

class Visualization{

    private:

        Scalar lane_color; 
        int lane_thickness;

        Scalar box_color;
        int box_thickness;

    public:

        /*Funciton Name: Visualization()                                          							   */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */  
        Visualization();


        /*Funciton Name: DrawLane(Mat img, MonoLane lane)                        */
        /*Description: Visualize lane on image                                                          */
        /*Input: Mat img - source image
                        MonoLane lane - MonoLane                                                             */        
        /*Output: Mat image with lanes                                                                       */

        Mat DrawLane(Mat img, MonoLane lane);


        /*Funciton Name: DrawBox(Mat img, vector<Object_2D> list_2D)    */
        /*Description: Visualize objects on image                                                     */
        /*Input: Mat img - source image
                        vector<Object_2D> list_2D - 2D object list                                 */        
        /*Output: Mat image with objects                                                                    */       

        Mat DrawBox(Mat img, vector<Object_2D> list_2D);


        /*Funciton Name: SetLaneColor(Scalar lc)                                                 */
        /*Description: Set lane color                                                                             */
        /*Input: Scalar lc - BGR scalar                                                                           */        
        /*Output: void                                                                                                          */      

        void SetLaneColor(Scalar lc);


        /*Funciton Name: SetLaneThickness(int lt)                                                 */
        /*Description: Set lane thickness                                                                     */
        /*Input: int lt - thickness                                                                                       */        
        /*Output: void                                                                                                          */

        void SetLaneThickness(int lt);


        /*Funciton Name: SetBoxColor(Scalar bc)                                                  */
        /*Description: Set box color                                                                              */
        /*Input: Scalar bc - BGR scalar                                                                           */        
        /*Output: void                                                                                                          */

        void SetBoxColor(Scalar bc);


        /*Funciton Name: SetBoxThickness(int bt)                                                  */
        /*Description: Set box thickness                                                                       */
        /*Input: int bt - thickness                                                                                     */        
        /*Output: void                                                                                                          */
        
        void SetBoxThickness(int bt);

};