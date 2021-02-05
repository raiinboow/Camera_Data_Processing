/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-18
Description:        Detection
*****************************************************************/

#include "Global.h"
#include "Detection.h"


/*Funciton Name: Detection()                                          								     */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */     

Detection::Detection(){}


/*Funciton Name: Detection(CameraLane* lane, string config)           */
/*Description: Constructor from configuration file, stereo model      */
/*Input: CameraLane* lane - pointer to lane model
				  string config - path to configuration file                                        */  

Detection::Detection(CameraLane* lane, string config){

	this->lane = lane;
	this->cm = lane->CamModel();
	Q = cm->Q();
	offset_x = cm->OffsetX();
	offset_y = cm->OffsetY();
	this->mcm = NULL;
	this->mlane = NULL;

	ReadParameters(config);
	UpdateMinDisparity();
	Initialize(config);
    
}


/*Funciton Name: Detection(MonoLane* mlane, string config)           */
/*Description: Constructor from configuration file, mono model       */
/*Input: MonoLane* mlane - pointer to monolane model
				  string config - path to configuration file                                        */  

Detection::Detection(MonoLane* mlane, string config){

	this->config = config;

	this->mlane = mlane;
	this->mcm = mlane->CamModel();
	
	Q = mcm->Q();
	offset_x = mcm->OffsetX();
	offset_y = mcm->OffsetY();
	this->lane =NULL;
	this->cm = NULL;
	
	ReadParameters(config);
	UpdateMinDisparity();
	Initialize(config);

}


/*Funciton Name: ReadParameters(string config)                                     */
/*Description: Read parameters from configuration file                         */
/*Input: string config - path to configuration file                                        */        
/*Output: void                                                                                                           */   

void Detection::ReadParameters(string config){

	this->config = config;

    FileStorage fs(config, FileStorage::READ);

	fs["FILTER_GROUND"]>>FILTER_GROUND;
	fs["FILTER_BACKGROUND"]>>FILTER_BACKGROUND;
	fs["FILTER_LANE"]>>FILTER_LANE;
	fs["DISPARITY_TYPE"]>>DISPARITY_TYPE;
	fs["GROUND_FILTER_TYPE"]>>GROUND_FILTER_TYPE;
	fs["ONLINE_GROUND_FILTER_TYPE"]>>ONLINE_GROUND_FILTER_TYPE;
	fs["DETECTOR_TYPE"]>>DETECTOR_TYPE;
	fs["NUM_OF_RANSAC_FILTERING"]>>NUM_OF_RANSAC_FILTERING;

}


/*Funciton Name: Initialize(string config)                                                   */
/*Description: Initialize objects according to configuration                */
/*Input: string config - path to configuration file                                      */        
/*Output: void                                                                                                          */

void Detection::Initialize(string config){

	// Disparity
	if(DISPARITY_TYPE==0)	sgbm = DisparitySGBM(config);
	else if (DISPARITY_TYPE==1)	 bm = DisparityBM(config);
	else if (DISPARITY_TYPE==2) elas = DisparityELAS(config);

	// GroundFilter
	if(FILTER_GROUND){

		if(!mlane){
			if(ONLINE_GROUND_FILTER_TYPE == 0)	hgf = HoughGroundFilter(config);
			else if(ONLINE_GROUND_FILTER_TYPE == 1)	rgf = RANSACGroundFilter(config);
		}
		else{
			if(GROUND_FILTER_TYPE == 0)	mgf = MonoGroundFilter(mcm, config);
			else if (GROUND_FILTER_TYPE == 1 ){
				if(ONLINE_GROUND_FILTER_TYPE == 0)	hgf = HoughGroundFilter(config);
				else if(ONLINE_GROUND_FILTER_TYPE == 1)	rgf = RANSACGroundFilter(config);
			}	
		}
	}

	// ObjectDetector
	if(DETECTOR_TYPE == 0){

		if(lane)	cd = ContourDetector(cm, lane, config);
		else if (mlane)	cd = ContourDetector(mcm, mlane, config);	

	}
	else if (DETECTOR_TYPE == 1){
		uvd = UVDisparityDetector(config);
	}


	// NMSFilter
	nmsf = NMSFilter(config);

	// LaneFilter
	if(FILTER_LANE){

		if(lane) lf = LaneFilter(lane);
		else if (mlane)	mlf = MonoLaneFilter(mlane);

	}


}


/*Funciton Name: Update2D(Mat left, Mat right)                                     */
/*Description: Detect 2D objects from rectified image pair                 */
/*Input: Mat left - left rectified image                                      
                  Mat right - right rectified image														 */        
/*Output: void                                                                                                          */

void Detection::Update2D(Mat left, Mat right){

	// Calculate disparity
	Mat disparity;
	start_disp = clock();
	if(DISPARITY_TYPE==0) disparity = sgbm.CalculateDispMap(left, right);
	else if (DISPARITY_TYPE==1)	disparity = bm.CalculateDispMap(left, right);
	else if (DISPARITY_TYPE==2) disparity = elas.CalculateDispMap(left, right);
	stop_disp = clock();

	// Visualize disparity
	if(VISUAL_DISPARITY){
		Mat temp = Mat::zeros(disparity.rows, disparity.cols,CV_16SC1);
		Mat visual_disparity;
		applyColorMap(disparity, visual_disparity, COLORMAP_JET);
		imshow("Disparity", visual_disparity);
	}

	start_ground_filter = clock();
	// Filter ground and background
	if(GROUND_FILTER_TYPE==0 && mlane){

		mgf.UpdateGroundPlane();
		mgf.FilterGround(disparity);

		if(FILTER_BACKGROUND)	mgf.FilterBackground(disparity, min_disparity);

	}	
	else if(GROUND_FILTER_TYPE==1 || !mlane){

		if(ONLINE_GROUND_FILTER_TYPE==0){
			hgf.UpdateGroundPlane(disparity);
			hgf.FilterGround(disparity);
		}
		else if(ONLINE_GROUND_FILTER_TYPE==1){
			for(int i=0; i<NUM_OF_RANSAC_FILTERING; i++){
				rgf.UpdateGroundPlane(disparity);
				rgf.FilterGround(disparity);
			}
			// Mat temp = rgf.Vdisparity(disparity, 0, 255, 0);
			// // medianBlur(disparity, disparity, 13);
			// imshow("filtered v-disparity", temp);
		}
				
		if(FILTER_BACKGROUND)	mgf.FilterBackground(disparity, min_disparity);

	}
	
	stop_ground_filter = clock();
	
	// Visualize filtered disparity
	if(VISUAL_DISPARITY_NO_GROUND){
		Mat visual_disparity_no_ground;
		// ximgproc::getDisparityVis(disparity, visual_disparity_no_ground, 2.0);
		applyColorMap(disparity, visual_disparity_no_ground, COLORMAP_JET);
		imshow("Disparity no ground", visual_disparity_no_ground);
	} 

	start_detection = clock();
	// Detect objects
	if(DETECTOR_TYPE==0){
		list_2D = cd.DetectObject2D(disparity);
		nmsf.UpdateObjectSource(&list_2D);
		nmsf.FilterObjectList();
	}
	else if (DETECTOR_TYPE == 1){
		list_2D = uvd.DetectObject2D(disparity);
	}

	// Filter from lane
	if(FILTER_LANE){

		if(lane){
			lf.UpdateObjectSource(&list_2D);
			lf.FilterObjectList();
		}	
		else if (mlane){
			mlf.UpdateObjectSource(&list_2D);
			mlf.FilterObjectList();
		}

		cout<<"Detection after lane filtering: "<<list_2D.size()<<endl;

	}

	stop_detection = clock();

}


/*Funciton Name: Update3D()                                                                          */
/*Description: Project 2D objects to 3D											               */
/*Input: void																				                              */        
/*Output: void                                                                                                          */

void Detection::Update3D(){

	vector<Point3f> points,  points_cam;
	list_3D.clear();

	if(list_2D.size()!=0){

		// Get 2D points
		for(int i =0; i<list_2D.size(); i++){
			Object_2D obj = list_2D[i];
			points.push_back(Point3f(obj.tl.x+offset_x, obj.tl.y+offset_y, obj.disparity));
			points.push_back(Point3f(obj.br.x+offset_x, obj.br.y+offset_y, obj.disparity));
		}

		// Project to 3D
		perspectiveTransform(points, points_cam, Q);

		// Update 3D list
		if (points_cam.size()!=0){
			for(size_t i=0; i+1<points_cam.size(); i=i+2){
				Object_3D obj;
				obj.tl.x = float(points_cam[i].x);
				obj.tl.y = float(points_cam[i].y);
				obj.tl.z = float(points_cam[i].z);
				obj.br.x = float(points_cam[i +1].x);
				obj.br.y = float(points_cam[i +1].y);
				obj.br.z = float(points_cam[i +1].z);
				obj.distance = float(points_cam[i].z);
				list_3D.push_back(obj);
			}
		}
	}

}


/*Funciton Name: PrintList2D()						                                                   */
/*Description: Print 2D object list in the console                    		             */
/*Input: void                                                                                                               */        
/*Output: void                                                                                                           */         

void Detection::PrintList2D(){

    for (int i=0; i<list_2D.size(); i++){
		cout<<"Object"<<to_string(i+1)<<": \t";
		cout<<"u1:"<<list_2D[i].tl.x<<"\t";
		cout<<"v1:"<<list_2D[i].tl.y<<"\t";
		cout<<"d1:"<<list_2D[i].disparity<<endl;
		cout<<"\t\tu2:"<<list_2D[i].br.x<<"\t";
		cout<<"v2:"<<list_2D[i].br.y<<"\t";
		cout<<"d2:"<<list_2D[i].disparity<<endl;
	}
	cout<<"---------------------------------------------------------------"<<endl;
}


/*Funciton Name: PrintList2D(vector<Object_2D> list)					      */
/*Description: Print input 2D object list in the console                           */
/*Input: void                                                                                                               */        
/*Output: void                                                                                                           */     

void Detection::PrintList2D(vector<Object_2D> list){

    for (int i=0; i<list.size(); i++){
		cout<<"Object"<<to_string(i+1)<<": \t";
		cout<<"u1:"<<list[i].tl.x<<"\t";
		cout<<"v1:"<<list[i].tl.y<<"\t";
		cout<<"d1:"<<list[i].disparity<<endl;
		cout<<"\t\tu2:"<<list[i].br.x<<"\t";
		cout<<"v2:"<<list[i].br.y<<"\t";
		cout<<"d2:"<<list[i].disparity<<endl;
	}
	cout<<"---------------------------------------------------------------"<<endl;
}


/*Funciton Name: PrintList3D()						                                                   */
/*Description: Print 3D object list in the console                    		             */
/*Input: void                                                                                                               */        
/*Output: void                                                                                                           */        

void Detection::PrintList3D(){

	for (int i=0; i<list_3D.size(); i++){
		cout<<"Object"<<to_string(i+1)<<": \t";
		cout<<"X1:"<<setprecision(3)<<list_3D[i].tl.x<<"m\t";
		cout<<"Y1:"<<setprecision(3)<<list_3D[i].tl.y<<"m\t";
		cout<<"Z1:"<<setprecision(3)<<list_3D[i].tl.z<<"m"<<endl;
		cout<<"\t\tX2:"<<setprecision(3)<<list_3D[i].br.x<<"m\t";
		cout<<"Y2:"<<setprecision(3)<<list_3D[i].br.y<<"m\t";
		cout<<"Z2:"<<setprecision(3)<<list_3D[i].br.z<<"m"<<endl;
	}
	cout<<"---------------------------------------------------------------"<<endl;

}


/*Funciton Name: PrintList3D(vector<Object_3D> list)					      */
/*Description: Print input 3D object list in the console                           */
/*Input: void                                                                                                               */        
/*Output: void                                                                                                           */     

void Detection::PrintList3D(vector<Object_3D> list){

	for (int i=0; i<list.size(); i++){
		cout<<"Object"<<to_string(i+1)<<": \t";
		cout<<"X1:"<<setprecision(3)<<list[i].tl.x<<"m\t";
		cout<<"Y1:"<<setprecision(3)<<list[i].tl.y<<"m\t";
		cout<<"Z1:"<<setprecision(3)<<list[i].tl.z<<"m"<<endl;
		cout<<"\t\tX2:"<<setprecision(3)<<list[i].br.x<<"m\t";
		cout<<"Y2:"<<setprecision(3)<<list[i].br.y<<"m\t";
		cout<<"Z2:"<<setprecision(3)<<list[i].br.z<<"m"<<endl;
	}
	cout<<"---------------------------------------------------------------"<<endl;

}


/*Funciton Name: List2D()													   	 					        */
/*Description: Return current 2D object list     					                         */
/*Input: void                                                                                                               */        
/*Output: vector<Object_2D> list_2D                                                             */     

vector<Object_2D> Detection::List2D(){

    return list_2D;

}


/*Funciton Name: List3D()													   	 					        */
/*Description: Return current 3D object list     					                         */
/*Input: void                                                                                                               */        
/*Output: vector<Object_3D> list_3D                                                             */  

vector<Object_3D> Detection::List3D(){

    return list_3D;

}


/*Funciton Name: UpdateMinDisparity()												        */
/*Description: Update min_disparity according to lane depth            */
/*Input: void                                                                                                               */        
/*Output: void                                                                                                           */     

 void inline Detection::UpdateMinDisparity(){

    // Calculate min_disparity 
	if(cm)	min_disparity = cm->Distance_Disparity(lane->LaneDepth());
	else min_disparity = mcm->Distance_Disparity(mlane->LaneDepth());

 }