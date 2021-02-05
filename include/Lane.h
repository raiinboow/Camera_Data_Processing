/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-18
Description:        Lane, CameraLane, MonoLane
******************************************************************/

#pragma once

#include "CameraModel.h"

using namespace std;
using namespace cv;


/******************************************************************
Class Name: Lane
Description: Define physical ROI
******************************************************************/

class Lane{

    protected:

        float width;
        float min_depth;
        float depth;
        float depth_step;
        int num_of_waypoints;
        vector<float> list_depth;

    public:

        /*Funciton Name: Lane()                                          					         			      */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                                */   

        Lane();


        /*Funciton Name: Lane(string config)                                                             */
        /*Description: Constructor from configuration file                                    */
        /*Input: string config - path to configuration file                                        */  

        Lane(string config);

                
        /*Funciton Name: LaneDepth()   					    	         	   	 					          */
        /*Description: Return depth                                  					                           */
        /*Input: void                                                                                                               */        
        /*Output: float depth                                                                                              */   
        
        float LaneDepth();


        /*Funciton Name: LaneWidth()   					    	         	   	 					          */
        /*Description: Return width                                  					                           */
        /*Input: void                                                                                                               */        
        /*Output: float width                                                                                              */  

        float LaneWidth();


        /*Funciton Name: LaneDepthStep()   					    		   	 					        */
        /*Description: Return depth step                        					                          */
        /*Input: void                                                                                                               */        
        /*Output: float depth_step                                                                                   */     

        float LaneDepthStep();


        /*Funciton Name: LaneNumWP()   			              				   	 					      */
        /*Description: Return number of way points    					                          */
        /*Input: void                                                                                                               */        
        /*Output: int num_of_waypoints                                                                      */    

        int LaneNumWP();


        /*Funciton Name: ListDepth()   					    	         	   	 					          */
        /*Description: Return depth list                           					                           */
        /*Input: void                                                                                                             */        
        /*Output: vector<float> list_depth                                                                 */  

        vector<float> ListDepth();


        /*Funciton Name: LaneDepth()   					    	         	   	 					          */
        /*Description: Return depth                                  					                           */
        /*Input: void                                                                                                               */        
        /*Output: float min_depth                                                                                   */  

        float LaneMinDepth();
        
};


/******************************************************************
Class Name: CameraLane
Description: Define physical ROI, width projection to image
******************************************************************/

class CameraLane: public Lane{

    private:
        CameraModel* cm;
    
    protected:
        vector<vector<int>> img_widths; 
    
    public:

        /*Funciton Name: CameraLane()                          					         			        */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                                */        

        CameraLane();


        /*Funciton Name: CameraLane(string config)                                             */
        /*Description: Constructor from configuration file                                    */
        /*Input: string config - path to configuration file                                        */  

        CameraLane(string config);


        /*Funciton Name: CameraLane(string config)                                             */
        /*Description: Constructor from camera model and  configuration file*/
        /*Input: CameraModel* cm - pointer to camera model
                        string config - path to configuration file                                        */  

        CameraLane(CameraModel* cm, string config);


        /*Funciton Name: UpdateLane()   					    	         	   	 			          */
        /*Description: Update image representation of lane                              */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                          */  

        void UpdateLane();


        /*Funciton Name: Widths()   					    	                     	   	 			          */
        /*Description: Return list of widths in image representation              */
        /*Input: void                                                                                                             */        
        /*Output: vector<vector<int>> img_widths                                                */  

        vector<vector<int>> Widths();


        /*Funciton Name: Width(int n)				    	                     	   	 			          */
        /*Description: Return the n-th width in image representation           */
        /*Input: int n - index of way point                                                                   */        
        /*Output: vector<int> width                                                                              */  

        vector<int> Width(int n);


        /*Funciton Name: PrintWidths()   					      	         	   	 			           */
        /*Description: Print lane widths in image representation                   */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                         */  

        void PrintWidths();


        /*Funciton Name: CamModel()   					      	            	   	 			           */
        /*Description: Return pointer of camera model                                       */
        /*Input: void                                                                                                             */        
        /*Output: CameraModel* pointer of camera model                                */  

        CameraModel* CamModel();

};


/******************************************************************
Class Name: Lane
Description: Define physical ROI, width and height projection to image
******************************************************************/

class MonoLane: public CameraLane{

    private:
        float max_pos_from_ground;
        MonoCameraModel* mcm;
        vector<vector<Point2i>> lines;

    
    public:

        /*Funciton Name: MonoLane()                          					         			          */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                                */   

        MonoLane();


        /*Funciton Name: MonoLane(MonoCameraModel* mcm, string config) */
        /*Description: Constructor from mono camera model and  configuration file*/
        /*Input: MonoCameraModel* mcm - pointer to mono camera model
                        string config - path to configuration file                                                      */  

        MonoLane(MonoCameraModel* mcm, string config);


        /*Funciton Name: UpdateLane()			    	              	   	 			                  */
        /*Description: Update line representation of lane                                  */
        /*Input: void                                                                                                             */        
        /*Output: void                                                                                                          */  

        void UpdateLane();


        /*Funciton Name: VisAllLines()   					    	              	   	 			           */
        /*Description: Return list of lines in image representation                  */
        /*Input: void                                                                                                             */        
        /*Output: vector<vector<Point2i>> lines                                                     */  

        vector<vector<Point2i>> VisAllLines();


        /*Funciton Name: VisOneLine(int n)			    	              	   	 			            */
        /*Description: Return n-th line in image representation                       */
        /*Input: int n - index of way point                                                                   */        
        /*Output: vector<Point2i> line                                                                         */  

        vector<Point2i> VisOneLine(int n);


        /*Funciton Name: MonoCamModel()   					           	   	 			           */
        /*Description: Return pointer of camera model                                       */
        /*Input: void                                                                                                             */        
        /*Output: MonoCameraModel* pointer of  monocamera model      */  

        MonoCameraModel* CamModel();


        /*Funciton Name: MaxPosFromGround()   					           	   		           */
        /*Description: Return max distance from the ground plane               */
        /*Input: void                                                                                                             */        
        /*Output: float max_pos_from_ground                                                       */  

        float MaxPosFromGround();

};