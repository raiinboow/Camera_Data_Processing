/*******************************************************************************
Author:                 Jing Gong
Date:                      2020-12-18
Description:        CameraObjectDetection
*******************************************************************************/

#include "Global.h"
# include "CameraObjectDetection.h"

/*Funciton Name: CameraObjectDetection()                                               */
/*Description: Default constructor                                                                   */
/*Input: void                                                                                                               */  

CameraObjectDetection::CameraObjectDetection(){}


/*Funciton Name: CameraObjectDetection(string config)                     */
/*Description: Constructor from configuration file                                    */
/*Input: string config - path of configuration file                                        */ 

CameraObjectDetection::CameraObjectDetection(string config){

    FileStorage fs(config, FileStorage::READ);

    fs["SAMPLE_RATE"]>>SAMPLE_RATE;

    fs["READ_FROM_FILE"]>>READ_FROM_FILE;
    fs["PATH_FILE"]>>PATH_FILE;

    fs["RECORD"]>>RECORD;
    fs["RUN_ALGORITHM"]>>RUN_ALGORITHM;

	fs["SAVE_BAG"]>>SAVE_BAG;
    fs["SAVE_VIDEO"]>>SAVE_VIDEO;
    fs["PATH_RECORD"]>>PATH_RECORD;

    fs["IS_MONO"]>>IS_MONO;
	fs["PATH_STEREO_CALIBRATION"]>>PATH_STEREO_CALIBRATION;
	fs["PATH_DETECTION_CONFIG"]>>PATH_DETECTION_CONFIG;
	fs["PATH_VISUAL"]>>PATH_VISUAL;
    fs["PATH_MONTAGE"]>>PATH_MONTAGE;
    fs["PATH_LANE"]>>PATH_LANE;

     fs["MONO_DETECTION"]>>MONO_DETECTION;

    InitMotion();
    InitVisual();
    InitObjects();

}


/*Funciton Name: InitMotion()                                                                          */
/*Description: Initialize abs_motion according to configuration       */
/*Input: void                                                                                                              */
/*Output: void                                                                                                          */  

void CameraObjectDetection::InitMotion(){

    Point3f init_pos;

    FileStorage fs(PATH_MONTAGE, FileStorage::READ);

    fs["abs_init_x"]>>init_pos.x;
    fs["abs_init_y"]>>init_pos.y;
    fs["abs_init_z"]>>init_pos.z;

    abs_motion.InitPosition(init_pos);

}


/*Funciton Name: InitVisual()                                                                             */
/*Description: Initialize global visualization variables                            */
/*Input: void                                                                                                              */
/*Output: void                                                                                                           */    

void CameraObjectDetection::InitVisual(){

    FileStorage fs(PATH_VISUAL, FileStorage::READ);

    fs["VISUAL_ORIGINAL"]>>VISUAL_ORIGINAL;
    fs["VISUAL_RECTIFIED"]>>VISUAL_RECTIFIED;
    fs["VISUAL_DISPARITY"]>>VISUAL_DISPARITY;
    fs["VISUAL_DISPARITY_NO_GROUND"]>>VISUAL_DISPARITY_NO_GROUND;
    fs["VISUAL_DETECTION"]>>VISUAL_DETECTION;
    fs["VISUAL_MASKED_DISPARITY"]>>VISUAL_MASKED_DISPARITY;
    fs["VISUAL_LANE"]>>VISUAL_LANE;
    fs["VISUAL_U_V_DISPARITY"]>>VISUAL_U_V_DISPARITY;

}


/*Funciton Name: InitObjects()                                                                        */
/*Description: Initialize objects according to configuration                */
/*Input: void                                                                                                             */
/*Output: void                                                                                                          */

void CameraObjectDetection::InitObjects(){

    if(IS_MONO){
        mt265 = MonoCameraModel(PATH_STEREO_CALIBRATION);
        mt265.SetHeightSource(&abs_motion.position.y);
        mt265.SetThetaSource(&abs_motion.rotation.roll);
        if(RUN_ALGORITHM){
            mlane = MonoLane(&mt265, PATH_LANE);
            detector = Detection(&mlane, PATH_DETECTION_CONFIG);
        }
    }
    else{
        t265 = CameraModel(PATH_STEREO_CALIBRATION);
        if(RUN_ALGORITHM){
            lane = CameraLane(&t265, PATH_LANE);
            detector = Detection(&lane, PATH_DETECTION_CONFIG);
        }
    }
    rs_t265 = RScamera(PATH_STEREO_CALIBRATION, SAMPLE_RATE);
    if(RECORD) rs_t265.EnableRecord(PATH_RECORD, SAVE_BAG, SAVE_VIDEO);
    if(READ_FROM_FILE) rs_t265.EnableRead(PATH_FILE);
    cam_vehicle = Transform(PATH_MONTAGE);

    if(MONO_DETECTION)  md = MonoDetector();

}


/*Funciton Name: Run()                                                                                        */
/*Description: Run detection pipeline                                                            */
/*Input: void                                                                                                               */        
/*Output: void                                                                                                           */         

void CameraObjectDetection::Run(){

    // Start camera pipeline
    rs_t265.Start();

    // Variable to control the main loop
    bool run_loop = true;

    // Loop until 'q' or 'Q' is pressed
    while (run_loop){

        while (rs_t265.Running()){

            start_raw=clock();

            Mat left, right;

            // Update data from RScamera
            rs_t265.UpdateFrames();                                 // Frames
            left = rs_t265.RawLeft();                                   // Raw image left
            right = rs_t265.RawRight();                             // Raw image right
            
            stop_raw = clock();

            if(VISUAL_ORIGINAL){
                imshow("Original Left", left);
                imshow("Original Right", right);
            }

            if(RUN_ALGORITHM) RunAlgorithm(left, right);

            PrintProcessingTime();

            // Read key for interaction
            char key = char(cv::waitKey(1));
            if (key == ' ') {
                    rs_t265.Pause();
            }
            else if (key=='q' || key=='Q'){
                run_loop = false;
                rs_t265.Stop();
            }

        }

        char key = char(cv::waitKey(1));
        if (!rs_t265.Running() && key == ' ')   rs_t265.Start();
        else if (key == 'q' || key=='Q')    run_loop = false;

    }

    rs_t265.Stop();

    
}


/*Funciton Name: RunAlgorithm(Mat left, Mat right)                              */
/*Description: Run detection algorithm                                                        */
/*Input: Mat left - left rectified image                                                             */
/*              Mat right - right rectified image                                                       */        
/*Output: void                                                                                                           */    

void CameraObjectDetection::RunAlgorithm(Mat left, Mat right){

    start_raw = clock();

    if(IS_MONO) mlane.UpdateLane();                                         // MonoLane
    else lane.UpdateLane();                                             // Lane

    abs_motion.Update(rs_t265.IMU());           // Absolute motion

    stop_raw =clock();

    // Print motion
    rs_t265.IMU().PrintMotion("Ego");
    abs_motion.PrintMotion("Abs");

    // Rectification
    start_rectify = clock();

    if(IS_MONO) mt265.Rectify(left, right);
    else t265.Rectify(left, right);

    stop_rectify = clock();

    if(MONO_DETECTION){

        md.Detect(left);

    }
    else{

        // Detection            
        detector.Update2D(left, right);
        detector.Update3D();
        detector.PrintList3D();

        // Transform
        list_3D = cam_vehicle.PerspectiveTransform(detector.List3D());
        detector.PrintList3D(list_3D);

        // Visualize detection
        if(VISUAL_DETECTION){
            Mat  visual_left;
            cvtColor(left, visual_left, COLOR_GRAY2RGB);
            // if(disp.IsStable()) disp.Rotate(visual_left, abs_motion.rotation.yaw);
            vis.DrawBox(visual_left, detector.List2D());
            if(VISUAL_LANE&&IS_MONO) vis.DrawLane(visual_left, mlane);
            imshow("detection", visual_left);
        } 

    }
   

}


/*Funciton Name: PrintProcessingTime()                                                     */
/*Description: Print processing time in the console                                 */
/*Input: void                                                                                                               */        
/*Output: void                                                                                                           */         

void CameraObjectDetection::PrintProcessingTime(){

    int t_raw = stop_raw - start_raw;
    int t_rectify = stop_rectify - start_rectify;
    int t_disp = stop_disp - start_disp;
    int t_filter = stop_ground_filter - start_ground_filter;
    int t_detection = stop_detection - start_detection;

    int t_total = t_raw + t_rectify + t_disp + t_filter + t_detection;

    if(t_total >0){

        int p_raw =int(((t_raw+0.0) / t_total)*100);
        int p_rectify = (((t_rectify+0.0) / t_total)*100);
        int p_disp = (((t_disp+0.0) / t_total)*100);
        int p_filter = (((t_filter+0.0) / t_total)*100);
        int p_detection = (((t_detection+0.0) / t_total)*100);

        cout<<"###########################Timing###############################"<<endl;
        cout<<"Raw data:\t"<<(t_raw)/double(CLOCKS_PER_SEC)*1000<<"ms\t"<<p_raw<<"%"<<endl;
        cout<<"Rectification:\t"<<(t_rectify)/double(CLOCKS_PER_SEC)*1000<<"ms\t"<<p_rectify<<"%"<<endl;
        cout<<"Disparity:\t"<<(t_disp)/double(CLOCKS_PER_SEC)*1000<<"ms\t"<<p_disp<<"%"<<endl;
        cout<<"Ground fIlter:\t"<<(t_filter)/double(CLOCKS_PER_SEC)*1000<<"ms\t"<<p_filter<<"%"<<endl;
        cout<<"Detection:\t"<<(t_detection)/double(CLOCKS_PER_SEC)*1000<<"ms\t"<<p_detection<<"%"<<endl;
        cout<<"Total:\t\t"<<(t_total)/double(CLOCKS_PER_SEC)*1000<<"ms"<<endl;
        cout<<"################################################################"<<endl;

    }

}
