/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        GroundFilter
*****************************************************************/

#include "Global.h"
#include "GroundFilter.h"

using namespace std;
using namespace cv;


/*****************************************************************
Class Name: GroundFilter
Description: Parent class to filter ground
*****************************************************************/

/*Funciton Name: GroundFilter()                                          						     */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */    

GroundFilter::GroundFilter(){}


/*Funciton Name: FilterBackground(Mat& disparity, int min_disparity)    */                               						     
/*Description: Filter pixels smalle rthan min_disparity                                      */
/*Input: Mat& disparity - disparity map 
                 int min_disparity - minimal disparity                                                        */    
/*Output: void                                                                                                                       */

void GroundFilter::FilterBackground(Mat& disparity, int min_disparity){

    Mat temp;
    inRange(disparity, min_disparity, 255, temp);
    bitwise_and(disparity, temp, disparity);

}



/*****************************************************************
Class Name: VDisparityGroundFilter
Description: ground filter in v-disparity space
*****************************************************************/

/*Funciton Name: VDisparityGroundFilter()                                          		   */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */    

VDisparityGroundFilter::VDisparityGroundFilter(){}


/*Funciton Name:FilterGround(Mat& disparity)                                             */                            
/*Description: filter ground according to variables                                        */
/*Input: Mat& disparity - disparity map                                                               */    
/*Output: void                                                                                                                 */

void VDisparityGroundFilter::FilterGround(Mat& disparity){

    cout<<"Filtering ground... \tk: "<<ground_k<<"\tb: "<<ground_b<<endl;
    for (int r = 0; r < disparity.rows;r++) {
		for (int c = 0; c < disparity.cols;c++){
			int pixel = disparity.at<uint8_t>(r, c);
			if (r >int(ground_k*pixel+ground_b-ground_tolerance)) disparity.at<uint8_t>(r, c)=0;
		}
	}

}



/*****************************************************************
Class Name: MonoGroundFilter
Description: ground filter using mono-geometry
*****************************************************************/

/*Funciton Name: MonoGroundFilter()                                          		          */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */    

MonoGroundFilter::MonoGroundFilter(){}


/*Funciton Name: MonoGroundFilter(MonoCameraModel* mcm, string config)	   */
/*Description:  constructor with mono camera model and configuration file            */
/*Input: MonoCameraModel* mcm - pointer to a MonoCameraModel
                  string config - path to configuration file                                                                    */    

MonoGroundFilter::MonoGroundFilter(MonoCameraModel* mcm, string config){

    this->mcm = mcm;

    FileStorage fs(config, FileStorage::READ);

    fs["ground_tolerance"] >> ground_tolerance;

}


/*Funciton Name:UpdateGroundPlane()                                                            */                            
/*Description: upadate ground_k and ground_b                                             */
/*Input: void                                                                                                                     */    
/*Output: void                                                                                                                 */

void MonoGroundFilter::UpdateGroundPlane(){

    ground_k = mcm->CamHeight() / (mcm->Baseline()*cos(mcm->CamTheta()*M_PI/180)+0.0001);
	ground_b = mcm->OutCy() + mcm->F() * tan(mcm->CamTheta()*M_PI/180);

    cout<<"Mono\tk: "<<ground_k<<"\tb:"<<ground_b<<endl;

}


/*Funciton Name:Rotate(Mat& img, float angle)                                              */                            
/*Description: (not used) rotate disparity map for a given angle              */
/*Input: Mat& img - input/output image
                  float angle - angle in degree                                                                    */    
/*Output: void                                                                                                                 */

// void MonoGroundFilter::Rotate(Mat& img, float angle){
//     Size dst_sz(img.cols, img.rows);
//     cv::Point2f center(mcm->OutCx(), mcm->OutCy());
//     cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
//     cv::warpAffine(img, img, rot_mat, dst_sz, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
// }


/*Funciton Name:IsStable()                                                                                      */                            
/*Description: (not used) return whether rotation enabled                       */
/*Input: void                                                                                                                    */    
/*Output: bool                                                                                                                  */

// bool MonoGroundFilter::IsStable(){
//     return is_stable;
// }



/*****************************************************************
Class Name: OnlineGroundFilter
Description: ground plane is calculated from v-disparity online
*****************************************************************/

/*Funciton Name: OnlineGroundFilter()                                          		          */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */    

OnlineGroundFilter::OnlineGroundFilter(){}


/*Funciton Name: OnlineGroundFilter(string config)                 		          */
/*Description: constructor from configuration file                                     */
/*Input: string config - path to configuration file                                        */  

OnlineGroundFilter::OnlineGroundFilter(string config){

    FileStorage fs(config, FileStorage::READ);

    fs["ground_tolerance"] >> ground_tolerance;
    fs["min_count"] >> min_count;
    fs["max_disparity"] >> max_disparity;
    fs["min_disparity"] >> min_disparity;

}


/*Funciton Name: CalcUVdisp(Mat disparity, bool u, bool v, int start_row, int start_column)      */
/*Description: update u_disparity and v_disparity from disparity map,
                                a trapezoid area from (start_row, start_column)                                                              */
/*Input: Mat disparity - disparity map
                  bool u - calculate u-disparity
                  bool v - calculate v- disparity 
                  int start_row - row of the top-left point
                  int start_column - column of the top-left point                                                                                */  
/*Output: void                                                                                                                                                                    */

void OnlineGroundFilter::CalcUVdisp(Mat disparity, bool u, bool v, int start_row, int start_column){


    u_disparity= Mat:: zeros(Size(disparity.cols, max_disparity), CV_8UC1);
    v_disparity = Mat:: zeros(Size(max_disparity, disparity.rows), CV_8UC1);

    for (int r = start_row; r < disparity.rows;r++) {

        int c_left = int(start_column - start_column * (r-start_row)/(v_disparity.rows-start_row));
        int c_right = disparity.cols-c_left;

        for (int c = c_left; c < c_right;c++) {
            int pixel = disparity.at<uint8_t>(r, c);
            if (pixel > min_disparity){
                if(v)   v_disparity.at<uint8_t>(r, pixel)++;
                if(u)   u_disparity.at<uint8_t>(pixel, c)++;
            }
        }
    }

    // Filter weak counts
    if(v)   threshold(v_disparity, v_disparity, min_count, 255, THRESH_BINARY);
    if(u)   threshold(u_disparity, u_disparity, min_count, 255, THRESH_BINARY);

    // Visualize
    if(VISUAL_U_V_DISPARITY){
        if(u)   cv::imshow("u-disparity", u_disparity);
        if(v)   cv::imshow("v_disparity", v_disparity);
    }

}


/*Funciton Name: Udisparity(Mat disparity, int min_disp, int max_disp, int min_count)              */
/*Description: calculate and return u-disparity                                                                                                 */
/*Input: Mat disparity - disparity map
                  int min_disp - minimal disparity
                  int max_disp - maximal disparity
                  int min_count - minimal count                                                                                                                 */  
/*Output: Mat u_disparity                                                                                                                                             */

Mat OnlineGroundFilter::Udisparity(Mat disparity, int min_disp, int max_disp, int min_count){

    Mat temp = Mat:: zeros(Size(disparity.cols, max_disparity), CV_8UC1);
    
    // Calculate
    for (int r = 0; r < disparity.rows;r++) {
        for (int c = 0; c < disparity.cols;c++) {
            int pixel = disparity.at<uint8_t>(r, c);
            if (pixel > min_disparity){
                temp.at<uint8_t>(pixel, c)++;
            }
        }
    }

    // Filter weak counts
    threshold(temp, temp, min_count, 255, THRESH_BINARY);

    return temp;

}


/*Funciton Name: Vdisparity(Mat disparity, int min_disp, int max_disp, int min_count)              */
/*Description: calculate and return v-disparity                                                                                                 */
/*Input: Mat disparity - disparity map
                  int min_disp - minimal disparity
                  int max_disp - maximal disparity
                  int min_count - minimal count                                                                                                                 */  
/*Output: Mat v_disparity                                                                                                                                             */

Mat OnlineGroundFilter::Vdisparity(Mat disparity, int min_disp, int max_disp, int min_count){

    Mat temp = Mat:: zeros(Size(max_disparity, disparity.rows), CV_8UC1);

    // Calculate
    for (int r = 0; r < disparity.rows;r++) {
        for (int c = 0; c < disparity.cols;c++) {
            int pixel = disparity.at<uint8_t>(r, c);
            if (pixel > min_disparity){
                temp.at<uint8_t>(r, pixel)++;
            }
        }
    }

    // Filter weak counts
    threshold(temp, temp, min_count, 255, THRESH_BINARY);

    return temp;

}



/*****************************************************************
Class Name: HoughGroundFilter
Description: ground plane detected from Hough transform
*****************************************************************/

/*Funciton Name: HoughGroundFilter()                                         		         */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */   

HoughGroundFilter::HoughGroundFilter(){}


/*Funciton Name: HoughGroundFilter(string config)                 		         */
/*Description: constructor from configuration file                                     */
/*Input: string config - path to configuration file                                        */  

HoughGroundFilter::HoughGroundFilter(string config): OnlineGroundFilter(config){


    FileStorage fs(config, FileStorage::READ);

    fs["hough_min_count"] >> hough_min_count;
    fs["hough_min_length"] >> hough_min_length;
    fs["hough_max_gap"] >> hough_max_gap;

}


/*Funciton Name: UpdateGroundPlane(Mat disparity)           		              */
/*Description: update ground_k and ground_b from disparity map      */
/*Input: Mat disparity - diaprity map                                                                     */  
/*Output: void                                                                                                                  */

void HoughGroundFilter::UpdateGroundPlane(Mat disparity){

    CalcUVdisp(disparity, false, true);

    // Detect ground line in V-Disparity
    Mat dst, cdst, cdstP;
    // Edge detection
    Canny(v_disparity, dst, 50, 100, 3);
    // Copy edges to the images that will display the results in BGR
    cv::cvtColor(dst, cdstP, COLOR_GRAY2BGR);
    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI / 180, hough_min_count, hough_min_length, hough_max_gap); // runs the actual detection
    Vec4i line_ground;
    float maxLength;
    double slope_threshold = 0;

    if (linesP.size() > 0) {
        // Find line with max length
        for (size_t i = 0; i < linesP.size(); i++){
            Vec4i l = linesP[i];
            double length = sqrt(pow(l[3] - l[1], 2) + pow(l[2] - l[0], 2));
            if(length > maxLength) line_ground = l;
        }
    }
    // Update k and b
    ground_k = (line_ground[3] - line_ground[1]) / ((line_ground[2] - line_ground[0]) + 0.0001);
    ground_b = line_ground[3] - ground_k * line_ground[2];

    // Show results
    if(VISUAL_U_V_DISPARITY){
        line(cdstP, Point(line_ground[0], line_ground[1]), Point(line_ground[2], line_ground[3]), Scalar(0, 255, 255), 1, LINE_AA);
        imshow("Ground Line - Probabilistic Line Transform", cdstP);
    }

    cout<<"Online\tk: "<<ground_k<<"\tb:"<<ground_b<<endl;

}



/*****************************************************************
Class Name: RANSACGroundFilter
Description: ground plane detected from RANSAC algorithm
*****************************************************************/

/*Funciton Name: RANSACGroundFilter()                                            	       */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */   

RANSACGroundFilter::RANSACGroundFilter(){}


/*Funciton Name: HoughGroundFilter(string config)                 		         */
/*Description: constructor from configuration file                                     */
/*Input: string config - path to configuration file                                        */  

RANSACGroundFilter::RANSACGroundFilter(string config): OnlineGroundFilter(config){

    FileStorage fs(config, FileStorage::READ);

    fs["num_samples"] >> num_samples;
    fs["distance_threshold"] >> distance_threshold;
    fs["start_row"] >> start_row;
    fs["start_column"] >> start_column;
    fs["max_iteration"] >> max_iteration;
    fs["num_median_filter"] >> num_median_filter;

    mfk = MedianFilter(num_median_filter);
    mfb = MedianFilter(num_median_filter);

}


/*Funciton Name: UpdateGroundPlane(Mat disparity)           		              */
/*Description: update ground_k and ground_b from disparity map      */
/*Input: Mat disparity - diaprity map                                                                     */  
/*Output: void                                                                                                                  */

void RANSACGroundFilter::UpdateGroundPlane(Mat disparity){


    // Calculate V-Disparity
    CalcUVdisp(disparity, false, true, start_row, start_column);

    // Calculate point list
    UpdateList();

    // Run RANSAC if num_data>num_samples
    if(list_point.size()>num_samples){

        last_ground_k = ground_k;
        last_ground_b = ground_b;

        max_cost = 0;

        // Start iteration
        for(int i=0; i<max_iteration; i++){
            
            // Randomly sample the data
            GetRandomSamples();

            // Calculate line from sample
            LineModel temp = ComputeLine();
            
            // // Compute cost from sample
            int cost = ComputeCost(temp);

            // Update ground line if the sample brings more inliers
            if(cost>max_cost){
                max_cost = cost;
                ground_line = ComputeLine();
            }

        }

        // conver to k and b
        int temp_k = int(ground_line.my / (ground_line.mx + 1e-6));
        if(isless(abs(temp_k), 10000) ){
            int temp_b = int( v_disparity.rows - int(ground_line.sy - temp_k * ground_line.sx));
            if(isgreater(temp_b, 0)&&isless(temp_b, 100)){
                ground_k = mfk.Filter(abs(temp_k));
                ground_b = mfb.Filter(temp_b);
            }
            // ground_b =  v_disparity.rows - int(ground_line.sy - temp_k * ground_line.sx);
        }

        if(VISUAL_U_V_DISPARITY){
            Mat visual_v_disparity;
            cvtColor(v_disparity, visual_v_disparity, COLOR_GRAY2RGB);
            line(visual_v_disparity, Point(0, ground_b), Point(int((v_disparity.rows-ground_b)/ground_k), v_disparity.rows), Scalar(0, 255, 255), 1, LINE_AA);
            imshow("RANSAC line detection", visual_v_disparity);
        }

    }

}


/*Funciton Name: UpdateList()                                                             		              */
/*Description: update list_point from v_disparity                                            */
/*Input: void                                                                                                                      */  
/*Output: void                                                                                                                  */

void RANSACGroundFilter::UpdateList(){

    list_point.clear();

    for(int r=start_row; r<v_disparity.rows; r++){

        for(int c=0; c<v_disparity.cols; c++){
        // for(int c=c_left; c<c_right; c++){
            if(v_disparity.at<uint8_t>(r,c)>0){
                list_point.push_back(Point2f(c,v_disparity.rows-r));
            }
        }
    }

}


/*Funciton Name: GetRandomSamples()                                                                      */
/*Description: get samples from list_point and save to list_sample_point    */
/*Input: void                                                                                                                               */  
/*Output: void                                                                                                                           */

void RANSACGroundFilter::GetRandomSamples(){

    list_sample_point.clear();

    for(int i=0; i<num_samples; i++){
        int temp = rand() % list_point.size();
        list_sample_point.push_back(list_point[temp]);
    }

}


/*Funciton Name: ComputeLine()                                                                                     */
/*Description: compute a line from list_sample_point                                           */
/*Input: void                                                                                                                               */  
/*Output: LineModel line                                                                                                      */

LineModel RANSACGroundFilter::ComputeLine(){

    LineModel model;

    float sx = 0, sy = 0;
	float sxx = 0, syy = 0;
	float sxy = 0, sw = 0;

	for (int i = 0; i<num_samples; i++)
	{
        float x = list_sample_point[i].x;
        float y = list_sample_point[i].y;

		sx += x;
		sy += y;
		sxx += x*x;
		sxy += x*y;
		syy += y*y;
		sw += 1;
	}

	//variance;
	float vxx = (sxx - sx*sx / sw) / sw;
	float vxy = (sxy - sx*sy / sw) / sw;
	float vyy = (syy - sy*sy / sw) / sw;

	//principal axis
	float theta = atan2(2 * vxy, vxx - vyy) / 2;

	model.mx = cos(theta);
	model.my = sin(theta);

	//center of mass(xc, yc)
	model.sx = sx / sw;
	model.sy = sy / sw;

    return model;

}


/*Funciton Name: ComputeCost(LineModel line)                                                      */
/*Description: find inliers for a given line and count newly added inliers       */
/*Input: void                                                                                                                               */  
/*Output: LineModel line                                                                                                      */

int RANSACGroundFilter::ComputeCost(LineModel line){

    list_inliers.clear();
    num_inliers = 0;

    int cost = 0.;

    for(int i=0; i<list_point.size(); i++){

        float dist = ComputeDistance(line, list_point[i]);

        // Is inlier if dist less than threshold
        if(isless(dist, distance_threshold)){
            list_inliers.push_back(list_point[i]);
            num_inliers++;
            cost ++;
        }

    }

    return cost;

}


/*Funciton Name: ComputeDistance(LineModel line, Point p)                            */
/*Description: compute point to line distance                                                            */
/*Input: LineModel line - line
                 Point p - point                                                                                                           */  
/*Output: float distance                                                                                                        */

float RANSACGroundFilter::ComputeDistance(LineModel line, Point p){

    return fabs((p.x - line.sx)*line.my - (p.y - line.sy)*line.mx) / sqrt(line.mx*line.mx + line.my*line.my);

}
