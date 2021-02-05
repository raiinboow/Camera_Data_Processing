/*******************************************************************
Author:                 J. Gong
Date:                      2020-12-18
Description:        Lane, CameraLane, MonoLane
********************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "Global.h"
#include "Lane.h"

using namespace std;
using namespace cv;


/******************************************************************
Class Name: Lane
Description: Define physical ROI
******************************************************************/

/*Funciton Name: Lane()                                          					         			      */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                                */     

Lane::Lane(){}


/*Funciton Name: Lane(string config)                                                             */
/*Description: Constructor from configuration file                                    */
/*Input: string config - path to configuration file                                        */  

Lane::Lane(string config){

    FileStorage fs(config, FileStorage::READ);

    fs["width"] >> width;
    fs["min_depth"] >> min_depth;
    fs["depth_step"] >> depth_step;
    fs["num_of_waypoints"] >> num_of_waypoints;

    // Calculate depth and lines
    this->depth = this->min_depth + this->depth_step * this->num_of_waypoints;
    for(int i=0; i<num_of_waypoints; i++)   list_depth.push_back(min_depth + i * depth_step);

}


/*Funciton Name: LaneDepthStep()   					    		   	 					        */
/*Description: Return depth step                        					                          */
/*Input: void                                                                                                               */        
/*Output: float depth_step                                                                                   */     

float Lane::LaneDepthStep(){

    return depth_step;

}


/*Funciton Name: LaneNumWP()   			              				   	 					      */
/*Description: Return number of way points    					                          */
/*Input: void                                                                                                               */        
/*Output: int num_of_waypoints                                                                      */    

int Lane::LaneNumWP(){

    return num_of_waypoints;

}


/*Funciton Name: LaneDepth()   					    	         	   	 					          */
/*Description: Return depth                                  					                           */
/*Input: void                                                                                                               */        
/*Output: float depth                                                                                              */   

float Lane::LaneDepth(){

    return depth;

}


/*Funciton Name: LaneWidth()   					    	         	   	 					          */
/*Description: Return width                                  					                           */
/*Input: void                                                                                                               */        
/*Output: float width                                                                                              */  

float Lane::LaneWidth(){

    return width;

}


/*Funciton Name: LaneDepth()   					    	         	   	 					          */
/*Description: Return depth                                  					                           */
/*Input: void                                                                                                               */        
/*Output: float min_depth                                                                                   */  

float Lane::LaneMinDepth(){

    return min_depth;

}


/*Funciton Name: ListDepth()   					    	         	   	 					          */
/*Description: Return depth list                           					                           */
/*Input: void                                                                                                             */        
/*Output: vector<float> list_depth                                                                 */  

vector<float> Lane::ListDepth(){

    return list_depth;

}



/******************************************************************
Class Name: CameraLane
Description: Define physical ROI, width projection to image
******************************************************************/

/*Funciton Name: CameraLane()                          					         			        */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                                */     

CameraLane::CameraLane(){}


/*Funciton Name: CameraLane(string config)                                             */
/*Description: Constructor from configuration file                                    */
/*Input: string config - path to configuration file                                        */  

CameraLane::CameraLane(string config): Lane(config){}


/*Funciton Name: CameraLane(string config)                                             */
/*Description: Constructor from camera model and  configuration file*/
/*Input: CameraModel* cm - pointer to camera model
                  string config - path to configuration file                                        */  

CameraLane::CameraLane(CameraModel* cm, string config): Lane(config){

    this-> cm = cm;

}


/*Funciton Name: UpdateLane()   					    	         	   	 			          */
/*Description: Update image representation of lane                              */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                          */  

void CameraLane::UpdateLane(){

    img_widths.clear();

    for(int i = 0; i<num_of_waypoints; i++){

        float distance = list_depth[i];
        int width_img = cm->Meter_Pixel(width, distance);
        vector<int> temp;
        int left = int((cm->OutWidth()-width_img)/2);
        int right = int((cm->OutWidth()+width_img)/2);
        temp.push_back(left);
        temp.push_back(right);
        img_widths.push_back(temp);
    }   

}


/*Funciton Name: Widths()   					    	                     	   	 			          */
/*Description: Return list of widths in image representation              */
/*Input: void                                                                                                             */        
/*Output: vector<vector<int>> img_widths                                                */  

vector<vector<int>> CameraLane::Widths(){

    return img_widths;

}


/*Funciton Name: Width(int n)				    	                     	   	 			          */
/*Description: Return the n-th width in image representation           */
/*Input: int n - index of way point                                                                   */        
/*Output: vector<int> width                                                                              */  

vector<int> CameraLane::Width(int n){

    if(n<num_of_waypoints) {
        return img_widths[n];
    }

    else{
        vector<int> temp;
        temp.push_back(0);
        temp.push_back(0);
        return temp;
    }

}


/*Funciton Name: PrintWidths()   					      	         	   	 			           */
/*Description: Print lane widths in image representation                   */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                         */  

 void CameraLane::PrintWidths(){

     cout<<"########## Camera Lane Widths ########## "<<endl;

    for(int i =0; i<num_of_waypoints; i++){
            // cout<<(i+1)*depth_step<<"m:\t";
            cout<<list_depth[i]<<"m:\t";
            cout<<img_widths[i][0]<<"\t"<<img_widths[i][1]<<"\t";
            cout<<img_widths[i][1]-img_widths[i][0]<<endl;
    }

    cout<<"#########################################"<<endl;

 }


/*Funciton Name: CamModel()   					      	            	   	 			           */
/*Description: Return pointer of camera model                                       */
/*Input: void                                                                                                             */        
/*Output: CameraModel* pointer of camera model                                */  

CameraModel* CameraLane::CamModel(){

    return cm;

}




/**********************************************************************
Class Name: Lane
Description: Define physical ROI, width and height projection to image
**********************************************************************/

/*Funciton Name: MonoLane()                          					         			          */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                                */   

MonoLane::MonoLane(){}


/*Funciton Name: MonoLane(MonoCameraModel* mcm, string config) */
/*Description: Constructor from mono camera model and  configuration file*/
/*Input: MonoCameraModel* mcm - pointer to mono camera model
                  string config - path to configuration file                                                      */  

MonoLane::MonoLane(MonoCameraModel* mcm, string config): CameraLane(config){

    this-> mcm = mcm;

    FileStorage fs(config, FileStorage::READ);

    fs["max_pos_from_ground"] >> max_pos_from_ground;

}


/*Funciton Name: UpdateLane()			    	              	   	 			                  */
/*Description: Update line representation of lane                                  */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                          */  

void MonoLane::UpdateLane(){
    
    lines.clear();
    img_widths.clear();

    for(int i = 0; i<num_of_waypoints; i++){
        // float distance = (i+1) * depth_step;
        float distance = list_depth[i];
        int width_img = mcm->Meter_Pixel(width, distance);
        int u_img = mcm->Distance_U(distance);
        int v_l = int((mcm->OutWidth()-width_img)/2);
        int v_r = int((mcm->OutWidth()+width_img)/2);

        // Update widths
        vector<int> temp_w;
        temp_w.push_back(v_l);
        temp_w.push_back(v_r);
        img_widths.push_back(temp_w);

        // Update lines
        Point2i left(v_l, u_img);
        Point2i right(v_r, u_img);
        vector<Point2i> temp;
        temp.push_back(left);
        temp.push_back(right);
        lines.push_back(temp);
    }

}


/*Funciton Name: VisAllLines()   					    	              	   	 			           */
/*Description: Return list of lines in image representation                  */
/*Input: void                                                                                                             */        
/*Output: vector<vector<Point2i>> lines                                                     */  

vector<vector<Point2i>> MonoLane::VisAllLines(){

    return lines;

}


/*Funciton Name: VisOneLine(int n)			    	              	   	 			            */
/*Description: Return n-th line in image representation                       */
/*Input: int n - index of way point                                                                   */        
/*Output: vector<Point2i> line                                                                         */  

vector<Point2i> MonoLane::VisOneLine(int n){
        
    if(n<num_of_waypoints) {
        return lines[n];
    }
    else{
        Point2i p1, p2;
        vector<Point2i> vec;
        vec.push_back(p1);
        vec.push_back(p2);
        return vec;
    }
}


/*Funciton Name: MonoCamModel()   					           	   	 			           */
/*Description: Return pointer of camera model                                       */
/*Input: void                                                                                                             */        
/*Output: MonoCameraModel* pointer of  monocamera model      */  

MonoCameraModel* MonoLane::CamModel(){

    return mcm;

}


/*Funciton Name: MaxPosFromGround()   					           	   		           */
/*Description: Return max distance from the ground plane               */
/*Input: void                                                                                                             */        
/*Output: float max_pos_from_ground                                                       */  

float MonoLane::MaxPosFromGround(){

    return max_pos_from_ground;

}