/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-23
Description:        ObjectDetector
*****************************************************************/

#include "Global.h"
#include "ObjectDetector.h"
#include "ObjectFilter.h"


/*****************************************************************
Class Name: ObjectDetector
Description: Parent class to detect objects from disparity map
*****************************************************************/

/*Funciton Name: ObjectDetector()                                          			               */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */   

ObjectDetector::ObjectDetector(){}


/*Funciton Name: ObjectDetector(CameraModel* cm)           	               */
/*Description: Constructor with camera source                                         */
/*Input: CameraModel* cm - pointer to a CameraModel                        */   

ObjectDetector::ObjectDetector(CameraModel* cm){

    this->cm = cm;
    out_cx = cm->OutCx();
    out_cy = cm->OutCy();

}


/*Funciton Name: List2D()                                          			                                  */
/*Description: Return current 2D object list                                                 */
/*Input: void                                                                                                               */   
/*Output: vector<Object_2D> list_2D                                                              */

vector<Object_2D> ObjectDetector::List2D(){

    return list_2D;

}


/*Funciton Name: CalculateDisparity(Mat img, Rect box)                      */
/*Description: Calculate the disparity from given box                             
                              DISPARITY_CALC_TYPE:  0 - median
                                                                                 1 - average
                                                                                 2 - max                                          */
/*Input: Mat img - disparity map
                  Rect box - calculated area                                                                   */   
/*Output: int disparity value                                                                                */

int ObjectDetector::CalculateDisparity(Mat img, Rect box){

    int disparity ;
    if (box.area() != 0) {
        std::vector<float> vec;
        for (int r = box.tl().y; r <= box.br().y; r++) {
            for (int c = box.tl().x; c <= box.br().x; c++) {
                if (img.at<uint8_t>(r, c) != 0)  vec.push_back(img.at<uint8_t>(r, c));
            }
        }
        // not FP
        if (vec.size() != 0) {
            if (DISPARITY_CALC_TYPE == 0) {
                std::nth_element(vec.begin(), vec.begin() + vec.size() / 2, vec.end());
                disparity = vec[vec.size() / 2];
            }
            else if (DISPARITY_CALC_TYPE == 1) {
                float sum = 0;
                for (int k = 0; k < int(vec.size()); k++) sum += vec[k];
                disparity = int(sum / vec.size());
            }
            else {
                int max = -1;
                for (int k = 0; k < int(vec.size()); k++) {
                    if (vec[k] > max) max = vec[k];
                }
                disparity = max;
            }
        }
    }

    return disparity;

}


/*Funciton Name: RotateObject2D(float angle)                                         */
/*Description: Rotate all 2D objects for an angle                                       */
/*Input: float angle - rotation in degree,                                                        */   
/*Output: void                                                                                                           */

void ObjectDetector::RotateObject2D(float angle){

    Point p = Point(out_cx,out_cy);

	Mat rot2d = getRotationMatrix2D(p, angle, 1);

	for(int i = 0; i<list_2D.size(); i++){
		Object_2D obj = list_2D[i];
		Mat tl = (Mat_<double>(3,1) << obj.tl.x, obj.tl.y, 1);
		Mat br =(Mat_<double>(3,1) << obj.br.x, obj.br.y, 1);
		Mat tl_dst = rot2d * tl;
		Mat br_dst = rot2d *br;
		list_2D[i].tl.x = tl_dst.at<double>(0,0);
		list_2D[i].tl.y = tl_dst.at<double>(1,0);
		list_2D[i].br.x = br_dst.at<double>(0,0);
		list_2D[i].br.y = br_dst.at<double>(1,0);
	}

}



/*****************************************************************
Class Name: ContourDetector
Description: Detect object from contour
*****************************************************************/

/*Funciton Name: ContourDetector()                                     			               */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */ 

ContourDetector::ContourDetector(){}


/*Funciton Name: ContourDetector(CameraModel* cm, string config)        */
/*Description: Constructor from CameraModel and configuration                 */
/*Input: CameraModel* cm - pointer to a CameraModel                        
                  string config - path to configuration file                                                    */

ContourDetector::ContourDetector(CameraModel* cm, string config): ObjectDetector(cm){

    this->config = config;
    FileStorage fs(config, FileStorage::READ);

    fs["min_depth"]>>depth_step;
    fs["depth_step"]>>depth_step;
    fs["num_of_waypoints"]>>num_of_waypoints;
    fs.release();

    for(int i=0; i<num_of_waypoints; i++)   list_depth.push_back(min_depth + i * depth_step);

    InitiateParameter(config);

}


/*Funciton Name: ContourDetector(CameraModel* cm, Lane* lane, string config)        */
/*Description: Constructor from CameraModel, Lane and configuration                             */
/*Input: CameraModel* cm - pointer to a CameraModel  
                  Lane* lane - pointer to a Lane                      
                  string config - path to configuration file                                                                            */

ContourDetector::ContourDetector(CameraModel* cm, Lane* lane, string config): ObjectDetector(cm){

    this->config = config;
    this->lane = lane;
    depth_step = lane->LaneDepthStep();
    num_of_waypoints = lane->LaneNumWP();
    min_depth = lane->LaneMinDepth();
    list_depth = lane->ListDepth();

    InitiateParameter(config);

}


/*Funciton Name: InitiateParameter(string config)                                  */
/*Description: Read parameter from configuration file                          */
/*Input: string config - path to configuration file                                       */   
/*Output: void                                                                                                           */

void ContourDetector::InitiateParameter(string config){

    FileStorage fs(config, FileStorage::READ);
    
    fs["DISPARITY_CALC_TYPE"]>>DISPARITY_CALC_TYPE; 
    fs["od_pre_median_kernel"]>>pre_median_kernel;  
    fs["median_kernel"]>>median_kernel;
    fs["canny_threshold"]>>canny_threshold;

}


/*Funciton Name: DetectObject2D(Mat disparity)	                                  */
/*Description: Detect object from dispariy                                                   */
/*Input: Mat disparity - disparity map                                                            */   
/*Output: vector<Object_2D> list_2D                                                             */   

vector<Object_2D> ContourDetector::DetectObject2D(Mat disparity){

    list_2D.clear();

    if(pre_median_kernel>0 && pre_median_kernel%2 == 1) medianBlur(disparity, disparity, pre_median_kernel);

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat masked_disparity, visual_masked_disparity;
    RNG rng(12345);

    SizeFilter sf  = SizeFilter(cm, config);
    ConfidenceFilter cf = ConfidenceFilter(config);
    NMSFilter nmsf = NMSFilter(config);

    int delta, delta1;

    // for(float d=depth_step; isless(d,depth_step*num_of_waypoints)&&delta1!=1; d=d+depth_step){	
    
    for(int k=0; k+1<num_of_waypoints && delta1!=1; k++){

        // Declear lists of bounding box
        vector<Rect>  bounding_box_1, bounding_box_2;

        // Mask disparity through distance step
        delta = cm->Distance_Disparity(list_depth[k]);
        delta1 = cm->Distance_Disparity(list_depth[k+1]);

        if(delta == delta1) delta1 = 1;	

        // Add constraints to avoid double detection
        if(delta!=delta1){

            std::cout<<"Delta range: "<<delta<<" to "<< delta1<<endl;
            inRange(disparity, delta1, delta , masked_disparity);
            threshold(masked_disparity, masked_disparity, delta1, 255, THRESH_BINARY);
            // medianBlur(masked_disparity, masked_disparity, median_kernel);
            erode(masked_disparity, masked_disparity, getStructuringElement(MORPH_RECT, Size(20,20)));
            dilate(masked_disparity, masked_disparity, getStructuringElement(MORPH_RECT, Size(30,30)));

            // Visualize masked disparity
            if(VISUAL_MASKED_DISPARITY)  cvtColor(masked_disparity, visual_masked_disparity, COLOR_GRAY2RGB);

            Canny( masked_disparity, canny_output, canny_threshold, canny_threshold*2 );
            findContours(canny_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
            
            if(contours.size()>0){
                vector<vector<Point> > contours_poly(contours.size());
                vector<Rect> boundRect(contours.size());

                // Find bounding boxes
                for (int i = 0; i < contours.size(); i++){
                    approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
                    boundRect[i] = boundingRect(Mat(contours_poly[i]));
                }

                // Loop through bounding boxes
                for (size_t i = 0; i < contours.size(); i++){

                    // Calculate confidence to eliminate FPs
                    if(cf.ConfidenceFit(masked_disparity, boundRect[i])) bounding_box_1.push_back(boundRect[i]);
                
                    // Visualize bounding box at each step
                    if(VISUAL_MASKED_DISPARITY){
                        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                        rectangle(visual_masked_disparity, boundRect[i].tl(), boundRect[i].br(), color, 1, 8, 0);
                    }           
                }
            }

            // Visualize masked disparity
            if(VISUAL_MASKED_DISPARITY) imshow(to_string(list_depth[k]), visual_masked_disparity);
            
        }

        // Filter and generate objects
        if(bounding_box_1.size()!=0) {
            bounding_box_2 = nmsf.FilterObjectBox(bounding_box_1);
            Generate2DList(disparity, bounding_box_2);
        }
    }

    sf.UpdateObjectSource(&list_2D);
    sf.FilterObjectList();
    cout<<"Detection after size filtering: "<<list_2D.size()<<endl;

    return list_2D;

}


/*Funciton Name: Generate2DList(Mat&src, vector<Rect> boxes)      */
/*Description: Generate 2D list from dispariy map and boxes              */
/*Input: Mat src - disparity map                                                            
                  vector<Rect> boxes - detected boxes                                            */   
/*Output: void                                                                                                           */  

void ContourDetector::Generate2DList(Mat&src, vector<Rect> boxes){

	for(int i =0; i<boxes.size(); i++){
		Rect box = boxes[i];
		int disparity = CalculateDisparity(src, box);
        Object_2D obj;
        obj.tl.x = box.tl().x;
        obj.tl.y = box.tl().y;
        obj.br.x = box.br().x;
        obj.br.y = box.br().y;
        obj.disparity = disparity;
        list_2D.push_back(obj);
    }

}



/*****************************************************************
Class Name: UVDisparityDetector
Description: Detect object from UV-Disparity (experimental, incomplete)
*****************************************************************/

/*Funciton Name: UVDisparityDetector()                             			                 */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */ 

UVDisparityDetector::UVDisparityDetector(){}


/*Funciton Name: UVDisparityDetector(string config)                              */
/*Description: Constructor from configuration                                            */
/*Input: string config - path to configuration file                                         */

UVDisparityDetector::UVDisparityDetector(string config){

    FileStorage fs(config, FileStorage::READ);

    fs["uvd_min_count"]>>uvd_min_count;

}


/*Funciton Name: DetectObject2D(Mat disparity)	                                  */
/*Description: Detect object from dispariy                                                   */
/*Input: Mat disparity - disparity map                                                            */   
/*Output: vector<Object_2D> list_2D                                                             */ 

vector<Object_2D> UVDisparityDetector::DetectObject2D(Mat disparity){

    vector<Object_2D>  output;

    v_disp = ogf.Vdisparity(disparity, 0, 255, uvd_min_count);
    u_disp = ogf.Udisparity(disparity, 0, 255, uvd_min_count);

    imshow("vd", v_disp);
    imshow("ud", u_disp);

    return output;

}