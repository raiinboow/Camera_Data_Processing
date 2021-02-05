/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        Custom structures and utility classes
*****************************************************************/

#include "Utility.h"


/*****************************************************************
Class Name: Transform
Description: Class to perform general transform
*****************************************************************/

/*Funciton Name: Transform()                                          								    */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */   

Transform::Transform(){}


/*Funciton Name: Transform(string config)                                                  */
/*Description: Constructor from configuration file                                   */
/*Input: string config - path to configuration file                                        */  

Transform::Transform(string config){

     FileStorage fs(config, FileStorage::READ);

    fs["R"] >> R;
    fs["T"] >> T;

}


/*Funciton Name: quaternion_to_euler_rad(rs2_quaternion q)         */
/*Description: Convert quaternion to Euler angle in rad                         */
/*Input: rs2_quaternion q - quaternion                                                           */        
/*Output: EulerAngles angles                                                                               */   

EulerAngles Transform::quaternion_to_euler_rad(rs2_quaternion q){
    
    EulerAngles angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;

}


/*Funciton Name: quaternion_to_euler_grad(rs2_quaternion q)       */
/*Description: Convert quaternion to Euler angle in grad                       */
/*Input: rs2_quaternion q - quaternion                                                           */        
/*Output: EulerAngles angles                                                                               */   

EulerAngles Transform::quaternion_to_euler_grad(rs2_quaternion q){
    
    EulerAngles angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp)*180/M_PI;

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp)*180/M_PI; // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp)*180/M_PI;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp)*180/M_PI;

    return angles;

}


/*Funciton Name: PerspectiveTransform(Object_3D obj)                       */
/*Description: Perform perspective transform to an object                   */
/*Input: Object_3D obj - 3D object                                                                     */        
/*Output: Object_3D transformed 3D object                                                 */   

Object_3D Transform::PerspectiveTransform(Object_3D obj){

    Object_3D output;

    output.tl.x = R.at<float>(0,0)* obj.tl.x + R.at<float>(0,1)* obj.tl.y + R.at<float>(0,2)* obj.tl.z + T.at<float>(0,0);
    output.tl.y = R.at<float>(1,0)* obj.tl.x + R.at<float>(1,1)* obj.tl.y + R.at<float>(1,2)* obj.tl.z + T.at<float>(1,0);
    output.tl.z = R.at<float>(2,0)* obj.tl.x + R.at<float>(2,1)* obj.tl.y + R.at<float>(2,2)* obj.tl.z + T.at<float>(2,0);
    
    output.br.x = R.at<float>(0,0)* obj.br.x + R.at<float>(0,1)* obj.br.y + R.at<float>(0,2)* obj.br.z + T.at<float>(0,0);
    output.br.y = R.at<float>(1,0)* obj.br.x + R.at<float>(1,1)* obj.br.y + R.at<float>(1,2)* obj.br.z + T.at<float>(1,0);
    output.br.z = R.at<float>(2,0)* obj.br.x + R.at<float>(2,1)* obj.br.y + R.at<float>(2,2)* obj.br.z + T.at<float>(2,0);

    return output;

}


/*Funciton Name: PerspectiveTransform(vector<Object_3D> list_3D)     */
/*Description: Perform perspective transform to a list of 3D objects         */
/*Input: vector<Object_3D> list_3D - list of 3D objects                                    */        
/*Output: vector<Object_3D> transformed list of 3D objects                       */   

vector<Object_3D>Transform::PerspectiveTransform(vector<Object_3D> list_3D){

    vector<Object_3D> output;

    for(int i =0; i< list_3D.size(); i++){

        output.push_back(PerspectiveTransform(list_3D[i]));

    }

    return output;

}



/*****************************************************************
Class Name: ValueFilter
Description: Parent class for value filters
*****************************************************************/

/*Funciton Name: ValueFilter()                                         								     */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */   

ValueFilter:: ValueFilter(){

    N = 3;

}


/*Funciton Name: ValueFilter(int n)                                								     */
/*Description: Constructor with filter size                                                      */
/*Input: int n - filter size                                                                                         */   

ValueFilter::ValueFilter(int n){

    if(n>0) N = n;

    else N = 3;

}



/*****************************************************************
Class Name: ValueFilter
Description: Parent class for value filters
*****************************************************************/

/*Funciton Name: MedianFilter()                                         							  */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */   

MedianFilter::MedianFilter(){}


/*Funciton Name: MedianFilter(int n)                                						      */
/*Description: Constructor with filter size                                                      */
/*Input: int n - filter size                                                                                         */   

MedianFilter::MedianFilter(int n):ValueFilter(n){}


/*Funciton Name: Filter(int input)                                  								    */
/*Description: Perform median filter                                                                */
/*Input: int input - input value                                                                            */   
/*Output: int output_value                                                                                  */

int MedianFilter:: Filter(int input){

    if(history.size()==N){

        history.pop();
        history.push(input);

        queue<int> temp(history);
        vector<int> vec;

        while(temp.size()!=0){
            vec.push_back(temp.front());
            temp.pop();
        }

        std::nth_element(vec.begin(), vec.begin() + vec.size() / 2, vec.end());
        int output = vec[vec.size() / 2];

        return output;

    }
    else{

        history.push(input);
        return input;
        
    }



}


