/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-23
Description:        ObjectFilter
*****************************************************************/

#include"ObjectFilter.h"


/*****************************************************************
Class Name: ObjectFilter
Description: Parent class to filter object
*****************************************************************/

/*Funciton Name: ObjectFilter()                                          						       */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */    

ObjectFilter::ObjectFilter(){}



/*****************************************************************
Class Name: ObjectFilter2D
Description: Parent class to filter 2D object
*****************************************************************/

/*Funciton Name: ObjectFilter2D()                                          						    */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */    

ObjectFilter2D::ObjectFilter2D(){}


/*Funciton Name: UpdateObjectSource(vector<Object_2D>* source)      */
/*Description: Set pointer of the object list                                                            */
/*Input: vector<Object_2D>* source - pointer to object list                            */  
/*Output: void                                                                                                                      */

void ObjectFilter2D::UpdateObjectSource(vector<Object_2D>* source){

    source_2D = source;

}



/*****************************************************************
Class Name: NMSFilter
Description: Filter object using NMS algorithm
*****************************************************************/

/*Funciton Name: NMSFilter()                                          						            */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */  

NMSFilter::NMSFilter(){}


/*Funciton Name: NMSFilter(string config)                  						            */
/*Description: Constructor from configuration                                           */
/*Input: string config - path to configuration file                                        */  

NMSFilter::NMSFilter(string config){

    FileStorage fs(config, FileStorage::READ);
    fs["NMS_threshold"]>>NMS_threshold;    

}


/*Funciton Name:FilterObjectList()                                                                  */                            
/*Description: Filter object using NMS                                                            */
/*Input: void                                                                                                               */    
/*Output: void                                                                                                           */

void NMSFilter::FilterObjectList(){

    vector<float> confidences;
	vector<int> indices;
    vector<Rect> boxes;
    vector<Object_2D> temp;

    int size = (*source_2D).size();

	if (size!=0) {
        
        // Loop through original list, box of larger disparity has greater priority
		for (int i = 0; i<size; i++) {
            Object_2D obj = (*source_2D)[i];
			confidences.push_back(obj.disparity/1000.0);
            boxes.push_back(Rect(obj.tl, obj.br));
		}
        
        // NMS filter
		dnn::NMSBoxes(boxes, confidences, 0.0, NMS_threshold, indices);
		int range = int(indices.size());

        // Loop through filtered indices and copy obj to temp
		for (size_t i = 0; i < range; ++i){
			int idx = indices[i];
			temp.push_back((*source_2D)[idx]);
		}
        
        // Clear original list
        (*source_2D).clear();

        // Copy back to list
         if(temp.size()>0){

            for(int i=0; i<temp.size(); i++)    (*source_2D).push_back(temp[i]);

        }


	}

}


/*Funciton Name:FilterObjectBox(vector<Rect> boxes)                         */                            
/*Description: Filter list of boxes using NMS                                                */
/*Input: vector<Rect> boxes - list of original boxes                                   */    
/*Output: vector<Rect> filtered boxes                                                            */

vector<Rect> NMSFilter::FilterObjectBox(vector<Rect> boxes){

	vector<float> confidences;
	vector<int> indices;
	vector<Rect> filtered_boxes;

	if (boxes.size()!=0) {
		for (int i = 0; i<boxes.size(); i++) {
			confidences.push_back(boxes[i].area()/1000.0);
		}
		dnn::NMSBoxes(boxes, confidences, 0.0, NMS_threshold, indices);
		int range = int(indices.size());
		for (size_t i = 0; i < range; ++i){
			int idx = indices[i];
			filtered_boxes.push_back(boxes[idx]);
		}
	}
	return filtered_boxes;

}



/*****************************************************************
Class Name: ConfidenceFilter
Description: Filter object by confidence
*****************************************************************/

/*Funciton Name: ConfidenceFilter()                                          				      */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */  

ConfidenceFilter::ConfidenceFilter(){}


/*Funciton Name: ConfidenceFilter(string config)                  					 */
/*Description: Constructor from configuration                                           */
/*Input: string config - path to configuration file                                        */  

ConfidenceFilter::ConfidenceFilter(string config){

    FileStorage fs(config, FileStorage::READ);
    fs["confidence_threshold"]>>confidence_threshold;    

}


/*Funciton Name: CalculateConfidence(Mat disparity, Rect box)   	 */
/*Description: Calculate confidence from a box
                                confidence = non-zero pixels/area_of_box                    */
/*Input: Mat disparity - disparity map 
                 Rect box - area to be calculated                                                         */  
/*Output: float confidence                                                                                     */

float ConfidenceFilter:: CalculateConfidence(Mat disparity, Rect box){

	int count =0;
	int area = box.area();
	for (int r = box.tl().y; r <= box.br().y; r++) {
		for (int c = box.tl().x; c <= box.br().x; c++) {
			if (disparity.at<uint8_t>(r, c) >0) {
				count++;
			}
		}
	}
	float confidence = float(count)/area;
	return confidence;

}


/*Funciton Name: ConfidenceFit(Mat disparity, Rect box)   	                                                      */
/*Description: Return whether confidence of the box is above confidence_threshold   */
/*Input: Mat disparity - disparity map 
                 Rect box - area to be calculated                                                                                               */  
/*Output: bool result - 1: above threshold                                                                                           */

bool ConfidenceFilter::ConfidenceFit(Mat disparity, Rect box){

    float conf = CalculateConfidence(disparity, box);

    bool result = false;

    if(isgreater(conf, confidence_threshold)) result = true;

    return result;    

}


/*Funciton Name: FilterObjectList()   	                                                            */
/*Description: Filter object list by confidence, not implemented       */
/*Input: void                                                                                                               */  
/*Output: void                                                                                                           */

void ConfidenceFilter::FilterObjectList(){}



/*****************************************************************
Class Name: SizeFilter
Description: Filter object by size
*****************************************************************/

/*Funciton Name: SizeFilter()                                          				                       */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */  

SizeFilter::SizeFilter(){}


/*Funciton Name: SizeFilter(string config)                  					             */
/*Description: Constructor from configuration                                           */
/*Input: string config - path to configuration file                                        */  

SizeFilter::SizeFilter(string config){

    FileStorage fs(config, FileStorage::READ);
    fs["min_width"]>>min_width;
    fs["min_height"]>>min_height;
    fs["max_width"]>>max_width;
    fs["max_height"]>>max_height;

}


/*Funciton Name: SizeFilter(CameraModel* cm, string config)              */
/*Description: Constructor from cameramodel and configuration       */
/*Input: CameraModel* cm - pointer to a CameraModel
                  string config - path to configuration file                                         */  

SizeFilter::SizeFilter(CameraModel* cm, string config){

    this->cm = cm;

    FileStorage fs(config, FileStorage::READ);
    fs["min_width"]>>min_width;
    fs["min_height"]>>min_height;
    fs["max_width"]>>max_width;
    fs["max_height"]>>max_height;

}


/*Funciton Name:FilterObjectList()                                                                  */                            
/*Description: Filter object by size                                                                    */
/*Input: void                                                                                                               */    
/*Output: void                                                                                                           */

void SizeFilter::FilterObjectList(){

    int size = (*source_2D).size();
    vector<Object_2D> temp;

    if(size>0){

        // Loop through original list
        for(int i=0; i<size; i++){

            Object_2D obj = (*source_2D)[i];

            bool criterium = SizeFit(obj);

            // Save fit candidates to temp    
            if(criterium) temp.push_back(obj);

        }

        // Clear original list
        (*source_2D).clear();
        
        // Copy to original list
        if(temp.size()>0){

            for(int i=0; i<temp.size(); i++)    (*source_2D).push_back(temp[i]);

        }

    }

}


/*Funciton Name: SizeFit(Object_2D obj)   	                                                     */
/*Description: Return whether size of 2D object is relevant                       */
/*Input: Object_2D obj - 2D object                                                                        */  
/*Output: bool result - 1: size is relevant                                                            */

bool SizeFilter::SizeFit(Object_2D obj){

    int w_pix = abs(obj.br.x - obj.tl.x);
    int h_pix = abs(obj. br.y - obj.tl.y);
    int d = obj.disparity;

    float w_m = cm->Pixel_Meter(w_pix, d);
    float h_m = cm->Pixel_Meter(h_pix, d);

    bool criterium = isgreater(w_m, min_width) && (isgreater(h_m, min_height)) 
                                && isless(w_m, max_width) && isless(h_m, max_height);

    return criterium;

}



/*****************************************************************
Class Name: LaneFilter
Description: Filter object by lane(width, depth)
*****************************************************************/

/*Funciton Name: LaneFilter()                                          				                     */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */  

LaneFilter::LaneFilter(){}


/*Funciton Name: LaneFilter(CameraLane* clane)         		                     */
/*Description: Constructor with a CameraLane                                         */
/*Input: CameraLane* clane - pointer to a CameraLane                        */  

LaneFilter::LaneFilter(CameraLane* clane){

    this->clane = clane;

}


/*Funciton Name: IsOverlay(Object_2D obj, vector<int> line)                  */
/*Description: Return whether a 2D overlays with a line(2 points)         */
/*Input: Object_2D obj - 2D object
                  vector<int> line - lane representation                                               */  
/*Output: bool result - 1: is overlay with line                                                    */

bool LaneFilter::IsOverlay(Object_2D obj, vector<int> line){

    bool overlay = true;

    if(obj.br.x<line[0] || obj.tl.x>line[1]) overlay = false;

    return overlay;

}


/*Funciton Name:FilterObjectList()                                                                  */                            
/*Description: Filter object by lane                                                                    */
/*Input: void                                                                                                               */    
/*Output: void                                                                                                           */

void LaneFilter::FilterObjectList(){

    vector<Object_2D> temp; 
    int size = (*source_2D).size();

	// loop through objects
	for(int i=0; i<size; i++){
		Object_2D obj = (*source_2D)[i];
        float distance = (*clane->CamModel()).Disparity_Distance(obj.disparity);
	    int sec = int(distance/clane->LaneDepthStep());
		if(sec<=clane->LaneNumWP() && sec>=0){
            vector<int> line = clane->Width(sec);
			bool overlay = IsOverlay(obj, line);
			if (overlay) temp.push_back(obj);
		}
	}

	// Clear original list
	(*source_2D).clear();

    // Copy back to original list
	for(int i=0; i<temp.size(); i++){
	    (*source_2D).push_back(temp[i]);
	}

}



/*****************************************************************
Class Name: MonoLaneFilter
Description: Filter object by lane(width, depth, height)
*****************************************************************/

/*Funciton Name: MonoLaneFilter()                                          				         */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */  

MonoLaneFilter::MonoLaneFilter(){}


/*Funciton Name: MonoLaneFilter(MonoLane* mlane)                         */
/*Description: Constructor with a MonoLane                                             */
/*Input: MonoLane* mlane - pointer to a MonoLane                              */  

MonoLaneFilter::MonoLaneFilter(MonoLane* mlane){

    this->mlane = mlane;

}


/*Funciton Name: :IsOnGround(Object_2D obj, int ground_u)                                        */
/*Description: Return whether a 2D object floats above max_pos_from_ground   */
/*Input: Object_2D obj - 2D object
                  int ground_u - ground line at the depth of the object                                        */  
/*Output: bool result - 1: is on the ground                                                                                 */

bool MonoLaneFilter::IsOnGround(Object_2D obj, int ground_u){

    int disp = obj.disparity;
    int bottom = obj.br.y;
    int pos_pix = bottom - ground_u;

    bool on_ground = true;
    if(pos_pix>0){
        float to_ground = (*mlane->CamModel()).Pixel_Meter(pos_pix, disp);
        if(isgreater(to_ground, mlane->MaxPosFromGround())) on_ground = false;
    }

    return on_ground;

}


/*Funciton Name:FilterObjectList()                                                                  */                            
/*Description: Filter object by lane                                                                    */
/*Input: void                                                                                                               */    
/*Output: void                                                                                                           */

void MonoLaneFilter::FilterObjectList(){

    vector<Object_2D> temp; 
    int size = (*source_2D).size();

	// loop through objects
	for(int i=0; i<size; i++){
		Object_2D obj = (*source_2D)[i];
        
        float distance = (*mlane->CamModel()).Disparity_Distance(obj.disparity);
	    int sec = int(distance/mlane->LaneDepthStep());
		if(sec<=mlane->LaneNumWP() && sec>=0){
            vector<int> line = mlane->Width(sec);
            int ground_u = mlane->VisOneLine(sec)[0].y;
            bool on_ground = IsOnGround(obj, ground_u);
			bool overlay = IsOverlay(obj, line);
			if (overlay&&on_ground) temp.push_back(obj);
		}
	}

	// Clear original list
	(*source_2D).clear();

    // Copy back to original list
	for(int i=0; i<temp.size(); i++){
	    (*source_2D).push_back(temp[i]);
	}


}




