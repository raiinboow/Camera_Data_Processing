/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-24
Description:        Visualization
*****************************************************************/

#include"Visualization.h"


/*****************************************************************
Class Name: Visualization
Description: Class to visualize lane and object
*****************************************************************/

/*Funciton Name: Visualization()                                          							   */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */     

Visualization::Visualization(){

    lane_color = Scalar(153,255,153); // Green
    lane_thickness = 1;
    box_color = Scalar(102,255,255); // Yellow
    box_thickness = 2;

}


/*Funciton Name: DrawLane(Mat img, MonoLane lane)                        */
/*Description: Visualize lane on image                                                          */
/*Input: Mat img - source image
                  MonoLane lane - MonoLane                                                             */        
/*Output: Mat image with lanes                                                                       */

Mat Visualization::DrawLane(Mat img, MonoLane lane){

    vector<vector<Point2i>> lines = lane.VisAllLines();

	for(int i = 0; i<lines.size(); i++){

		line(img, lines[i][0], lines[i][1], lane_color, lane_thickness);
        int left_end = lines[i][0].x;
        if(left_end<0) left_end = 0;
		string text = format("%.2f", (i+1)*lane.LaneDepthStep()) + "m";
		putText(img, text,Point(left_end, lines[i][0].y),FONT_HERSHEY_PLAIN,  1, lane_color, 1);
        
	}

    return img;

}


/*Funciton Name: DrawBox(Mat img, vector<Object_2D> list_2D)    */
/*Description: Visualize objects on image                                                     */
/*Input: Mat img - source image
                  vector<Object_2D> list_2D - 2D object list                                 */        
/*Output: Mat image with objects                                                                    */

Mat Visualization::DrawBox(Mat img, vector<Object_2D> list_2D){

    if(list_2D.size()!=0){
        for(int i = 0; i<list_2D.size();i++){
            rectangle(img, list_2D[i].tl, list_2D[i].br, box_color, box_thickness, 8, 0);
        }
    }

    return img;

}


/*Funciton Name: SetLaneColor(Scalar lc)                                                 */
/*Description: Set lane color                                                                             */
/*Input: Scalar lc - BGR scalar                                                                           */        
/*Output: void                                                                                                          */

void Visualization::SetLaneColor(Scalar lc){

    lane_color = lc;

}


/*Funciton Name: SetLaneThickness(int lt)                                                 */
/*Description: Set lane thickness                                                                     */
/*Input: int lt - thickness                                                                                       */        
/*Output: void                                                                                                          */

void Visualization::SetLaneThickness(int lt){

    lane_thickness = lt;

}


/*Funciton Name: SetBoxColor(Scalar bc)                                                  */
/*Description: Set box color                                                                              */
/*Input: Scalar bc - BGR scalar                                                                           */        
/*Output: void                                                                                                          */

void Visualization::SetBoxColor(Scalar bc){

    box_color = bc;

}


/*Funciton Name: SetBoxThickness(int bt)                                                  */
/*Description: Set box thickness                                                                       */
/*Input: int bt - thickness                                                                                     */        
/*Output: void                                                                                                          */

void Visualization::SetBoxThickness(int bt){

    box_thickness = bt;

}