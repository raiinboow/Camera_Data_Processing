/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        Custom structures and utility classes
*****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

using namespace std;
using namespace cv;
using namespace rs2;


/*****************************************************************
Structure Name: Object_3D
Description: structure to describe an object in 3D
*****************************************************************/

struct Object_3D{

    Point3f tl;
    Point3f br;
    float distance;

};



/*****************************************************************
Structure Name: Object_2D
Description: structure to describe an object in 2D
*****************************************************************/

struct Object_2D{

    Point2f tl;
    Point2f br;
    float disparity;

};



/*****************************************************************
Structure Name: EulerAngles
Description: structure to describe an Euler angle
*****************************************************************/

struct EulerAngles {

    float roll, pitch, yaw;

};



/*****************************************************************
Structure Name: LineModel
Description: structure to describe a line
*****************************************************************/

struct LineModel{

    float mx, my, sx, sy;

};



/*****************************************************************
Class Name: Transform
Description: Class to perform general transform
*****************************************************************/

class Transform{

    private:

        Mat R;
        Mat T;

    public:

        /*Funciton Name: Transform()                                          								    */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */  

        Transform();


        /*Funciton Name: Transform(string config)                                                  */
        /*Description: Constructor from configuration file                                   */
        /*Input: string config - path to configuration file                                        */  

        Transform(string config);


        /*Funciton Name: PerspectiveTransform(Object_3D obj)                       */
        /*Description: Perform perspective transform to an object                   */
        /*Input: Object_3D obj - 3D object                                                                     */        
        /*Output: Object_3D transformed 3D object                                                 */   

        Object_3D PerspectiveTransform(Object_3D obj);


        /*Funciton Name: PerspectiveTransform(vector<Object_3D> list_3D)     */
        /*Description: Perform perspective transform to a list of 3D objects         */
        /*Input: vector<Object_3D> list_3D - list of 3D objects                                    */        
        /*Output: vector<Object_3D> transformed list of 3D objects                       */   

        vector<Object_3D> PerspectiveTransform(vector<Object_3D> list_3D);


        /*Funciton Name: quaternion_to_euler_rad(rs2_quaternion q)         */
        /*Description: Convert quaternion to Euler angle in rad                         */
        /*Input: rs2_quaternion q - quaternion                                                           */        
        /*Output: EulerAngles angles                                                                               */   

        static EulerAngles quaternion_to_euler_rad(rs2_quaternion q);


        /*Funciton Name: quaternion_to_euler_grad(rs2_quaternion q)       */
        /*Description: Convert quaternion to Euler angle in grad                       */
        /*Input: rs2_quaternion q - quaternion                                                           */        
        /*Output: EulerAngles angles                                                                               */   

        static EulerAngles quaternion_to_euler_grad(rs2_quaternion q);

};



/*****************************************************************
Class Name: ValueFilter
Description: Parent class for value filters
*****************************************************************/

class ValueFilter{

    protected:

        int N;

    public:

        /*Funciton Name: ValueFilter()                                         								     */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */     

        ValueFilter();

        /*Funciton Name: ValueFilter(int n)                                								     */
        /*Description: Constructor with filter size                                                      */
        /*Input: int n - filter size                                                                                         */           

        ValueFilter(int n);


        /*Funciton Name: Filter(int input)                                  								    */
        /*Description: Virtual function to filter the input                                        */
        /*Input: int input - input value                                                                            */   
        /*Output: int output_value                                                                                  */

        virtual int Filter(int input) = 0;

};



/*****************************************************************
Class Name: ValueFilter
Description: Parent class for value filters
*****************************************************************/

class MedianFilter:public ValueFilter{

    private:

        queue<int> history;

    public:

        /*Funciton Name: MedianFilter()                                         							  */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */   

        MedianFilter();


        /*Funciton Name: MedianFilter(int n)                                						      */
        /*Description: Constructor with filter size                                                      */
        /*Input: int n - filter size                                                                                         */   

        MedianFilter(int n);


        /*Funciton Name: Filter(int input)                                  								    */
        /*Description: Perform median filter                                                                */
        /*Input: int input - input value                                                                            */   
        /*Output: int output_value                                                                                  */

        int Filter(int input);

};




