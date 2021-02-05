/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-23
Description:        ObjectFilter
*****************************************************************/

#pragma once

#include <opencv2/opencv.hpp>

#include"Utility.h"
#include"CameraModel.h"
#include"Lane.h"

using namespace cv;
using namespace std;


/*****************************************************************
Class Name: ObjectFilter
Description: Parent class to filter object
*****************************************************************/

class ObjectFilter{

    public:

        /*Funciton Name: ObjectFilter()                                          						       */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */    

        ObjectFilter();


        /*Funciton Name:FilterObjectList()                                                                  */                            
        /*Description: virtual function to filter object                                              */
        /*Input: void                                                                                                               */    
        /*Output: void                                                                                                           */

        virtual void FilterObjectList() = 0;

};



/*****************************************************************
Class Name: ObjectFilter2D
Description: Parent class to filter 2D object
*****************************************************************/

class ObjectFilter2D: public ObjectFilter{

    protected:

    vector<Object_2D>* source_2D;

    public:

        /*Funciton Name: ObjectFilter2D()                                          						    */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */    

        ObjectFilter2D();


        /*Funciton Name: UpdateObjectSource(vector<Object_2D>* source)      */
        /*Description: Set pointer of the object list                                                            */
        /*Input: vector<Object_2D>* source - pointer to object list                            */  
        /*Output: void                                                                                                                      */

        void UpdateObjectSource(vector<Object_2D>* source);

};



/*****************************************************************
Class Name: NMSFilter
Description: Filter object using NMS algorithm
*****************************************************************/

class NMSFilter: public ObjectFilter2D{

    private:

        float NMS_threshold;
    
    public:

        /*Funciton Name: NMSFilter()                                          						            */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */  

        NMSFilter();


        /*Funciton Name: NMSFilter(string config)                  						            */
        /*Description: Constructor from configuration                                           */
        /*Input: string config - path to configuration file                                        */  

        NMSFilter(string config);


        /*Funciton Name:FilterObjectList()                                                                  */                            
        /*Description: Filter object using NMS                                                            */
        /*Input: void                                                                                                               */    
        /*Output: void                                                                                                           */        

        void FilterObjectList();


        /*Funciton Name:FilterObjectBox(vector<Rect> boxes)                         */                            
        /*Description: Filter list of boxes using NMS                                                */
        /*Input: vector<Rect> boxes - list of original boxes                                   */    
        /*Output: vector<Rect> filtered boxes                                                            */        

        vector<Rect> FilterObjectBox(vector<Rect> boxes);
        
};



/*****************************************************************
Class Name: ConfidenceFilter
Description: Filter object by confidence
*****************************************************************/

class ConfidenceFilter: public ObjectFilter2D{

    private:

        float confidence_threshold;
        int disparity_tolerance;

        /*Funciton Name: CalculateConfidence(Mat disparity, Rect box)   	 */
        /*Description: Calculate confidence from a box
                                        confidence = non-zero pixels/area_of_box                    */
        /*Input: Mat disparity - disparity map 
                        Rect box - area to be calculated                                                         */  
        /*Output: float confidence                                                                                     */

        float CalculateConfidence(Mat disparity, Rect box);

    public:

        /*Funciton Name: ConfidenceFilter()                                          				      */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */      

        ConfidenceFilter();


        /*Funciton Name: ConfidenceFilter(string config)                  					 */
        /*Description: Constructor from configuration                                           */
        /*Input: string config - path to configuration file                                        */         

        ConfidenceFilter(string config);


        /*Funciton Name: ConfidenceFit(Mat disparity, Rect box)   	                                                      */
        /*Description: Return whether confidence of the box is above confidence_threshold   */
        /*Input: Mat disparity - disparity map 
                        Rect box - area to be calculated                                                                                               */  
        /*Output: bool result - 1: above threshold                                                                                           */ 

        bool ConfidenceFit(Mat disparity, Rect box);


        /*Funciton Name: FilterObjectList()   	                                                            */
        /*Description: Filter object list by confidence, not implemented       */
        /*Input: void                                                                                                               */  
        /*Output: void                                                                                                           */

        void FilterObjectList(); 

};


/*****************************************************************
Class Name: SizeFilter
Description: Filter object by size
*****************************************************************/

class SizeFilter: public ObjectFilter2D{

    private:

        float min_width;
        float min_height;
        float max_width;
        float max_height;
        CameraModel* cm;
    
    public:

        /*Funciton Name: SizeFilter()                                          				                       */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */  

        SizeFilter();


        /*Funciton Name: SizeFilter(string config)                  					             */
        /*Description: Constructor from configuration                                           */
        /*Input: string config - path to configuration file                                        */          

        SizeFilter(string config);


        /*Funciton Name: SizeFilter(CameraModel* cm, string config)              */
        /*Description: Constructor from cameramodel and configuration       */
        /*Input: CameraModel* cm - pointer to a CameraModel
                        string config - path to configuration file                                         */          

        SizeFilter(CameraModel* cm, string config);

        /*Funciton Name:FilterObjectList()                                                                  */                            
        /*Description: Filter object by size                                                                    */
        /*Input: void                                                                                                               */    
        /*Output: void                                                                                                           */       

        void FilterObjectList();

 
        /*Funciton Name: SizeFit(Object_2D obj)   	                                                     */
        /*Description: Return whether size of 2D object is relevant                       */
        /*Input: Object_2D obj - 2D object                                                                        */  
        /*Output: bool result - 1: size is relevant                                                            */       

        bool SizeFit(Object_2D obj);

};



/*****************************************************************
Class Name: LaneFilter
Description: Filter object by lane(width, depth)
*****************************************************************/

class LaneFilter: public ObjectFilter2D{

    private:

        CameraLane* clane;
        
    public:

        /*Funciton Name: LaneFilter()                                          				                     */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */     

        LaneFilter();


        /*Funciton Name: LaneFilter(CameraLane* clane)         		                     */
        /*Description: Constructor with a CameraLane                                         */
        /*Input: CameraLane* clane - pointer to a CameraLane                        */        

        LaneFilter(CameraLane* clane);


        /*Funciton Name: IsOverlay(Object_2D obj, vector<int> line)                  */
        /*Description: Return whether a 2D overlays with a line(2 points)         */
        /*Input: Object_2D obj - 2D object
                        vector<int> line - lane representation                                               */  
        /*Output: bool result - 1: is overlay with line                                                    */

        bool IsOverlay(Object_2D obj, vector<int> line);


        /*Funciton Name:FilterObjectList()                                                                  */                            
        /*Description: Filter object by lane                                                                    */
        /*Input: void                                                                                                               */    
        /*Output: void                                                                                                           */

        void FilterObjectList();

};



/*****************************************************************
Class Name: MonoLaneFilter
Description: Filter object by lane(width, depth, height)
*****************************************************************/

class MonoLaneFilter: public LaneFilter{

    private:

        MonoLane* mlane;
        float max_pos_from_ground;

        /*Funciton Name: :IsOnGround(Object_2D obj, int ground_u)                                        */
        /*Description: Return whether a 2D object floats above max_pos_from_ground   */
        /*Input: Object_2D obj - 2D object
                        int ground_u - ground line at the depth of the object                                        */  
        /*Output: bool result - 1: is on the ground                                                                                 */

        bool IsOnGround(Object_2D obj, int ground_u);
    
    public:

        /*Funciton Name: MonoLaneFilter()                                          				         */
        /*Description: Default constructor                                                                    */
        /*Input: void                                                                                                               */     

        MonoLaneFilter();

        /*Funciton Name: MonoLaneFilter(MonoLane* mlane)                         */
        /*Description: Constructor with a MonoLane                                             */
        /*Input: MonoLane* mlane - pointer to a MonoLane                              */  

        MonoLaneFilter(MonoLane* mlane);


        /*Funciton Name:FilterObjectList()                                                                  */                            
        /*Description: Filter object by lane                                                                    */
        /*Input: void                                                                                                               */    
        /*Output: void                                                                                                           */
        
        void FilterObjectList();

};