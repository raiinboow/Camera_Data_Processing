/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        Motion
*****************************************************************/

# pragma once

#include <opencv2/opencv.hpp>

#include"Utility.h"

using namespace std;
using namespace cv;


/*****************************************************************
Class Name: Motion
Description: Class to handle motion
*****************************************************************/

class Motion{

    private:

        Point3f init_position;

    public:

        Point3f position;
        Point3f velocity;
        EulerAngles rotation;   


        /*Funciton Name: Motion()                                          								            */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */   

        // Motion();


        /*Funciton Name: InitPosition(Point3f init_pos)                                        */
        /*Description: Initialize motion with a 3D point                                         */
        /*Input: Point3f init_pos - initial position                                                     */        
        /*Output: void                                                                                                           */          

        void InitPosition(Point3f init_pos);


        /*Funciton Name: Reset()                                                                                      */
        /*Description: Reset motion back to initial position                                 */
        /*Input: void                                                                                                                */        
        /*Output: void                                                                                                           */           
        void Reset();


        /*Funciton Name: PrintMotion(string s)                                                         */
        /*Description: Print motion                                                                                  */
        /*Input: string s - additional information to be printed                          */        
        /*Output: void                                                                                                           */         

        void PrintMotion(string s);


        /*Funciton Name: Update(Motion rel_motion)                                           */
        /*Description: Update motion with a relative motion                             */
        /*Input: Motion rel_motion - relative motion                                              */        
        /*Output: void                                                                                                           */         

        void Update(Motion rel_motion);
    
};