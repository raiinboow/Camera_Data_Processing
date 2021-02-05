/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-23
Description:        RScamera
*****************************************************************/

#include <ctime>

#include "RScamera.h"


/*****************************************************************
Class Name: RScamera
Description: Class to retrive rawdata from RealSense camera
*****************************************************************/

/*Funciton Name: RScamera()                                          								  */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */    

RScamera::RScamera(){}


/*Funciton Name: RScamera(string config, int sample_rate)                                                    */
/*Description: constructor from configuration and specified sample_rate                         */
/*Input: string config - path to configuration file
                  int sample_rate - (down) sample rate, actual_FPS = raw_FPS/sample_rate     */    

RScamera::RScamera(string config, int sample_rate){

    running = true;

    record = false;
    save_bag = false;
    save_video = false;
    read_from_file = false;

    FileStorage fs(config, FileStorage::READ);

    fs["raw_width"]>>raw_width;
    fs["raw_height"]>>raw_height;
    fs["raw_fps"]>>raw_fps;

    if(sample_rate>0) {
        this->sample_rate = sample_rate;
    }
    else{
        cout<<"Invalid sample rate"<<endl;
        this->sample_rate = 1;
    }
    fps = (raw_fps + 0.0) / sample_rate;

}


/*Funciton Name: Start()                                                                                    */
/*Description: Start pipeline                                                                              */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                          */

void RScamera::Start(){

    running = true;
    pipe.start(cfg);

}


/*Funciton Name: Stop()                                                                                    */
/*Description: Stop pipeline                                                                              */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                          */

void RScamera::Stop(){

    if(running){
        running = false;
        pipe.stop();
    }

    if(save_video){
        writer_left.release();
        writer_right.release();
    }
    else{
        int r1 = remove(file_left.c_str());
        int r2 = remove(file_right.c_str());
    }

}


/*Funciton Name: Pause()                                                                                    */
/*Description: Pause pipeline                                                                              */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                          */

void RScamera::Pause(){

    if(running){

        running = false;
        pipe.stop();

    }

}


/*Funciton Name: Running()                                                                            */
/*Description: Return whether pipeline is running                                 */
/*Input: void                                                                                                             */        
/*Output: bool running                                                                                       */

bool RScamera::Running(){

    return running;

}


/*Funciton Name: UpdateFrames()                                                                */
/*Description: Retrive frame from pipeline and update raw data     */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                          */

void RScamera::UpdateFrames(){

    for(int i = 0; i < sample_rate; i++){
        frames = pipe.wait_for_frames();
    }

    UpdateRawImage();
    UpdateIMU();

    if(save_video){
        writer_left.write(raw_l);
        writer_right.write(raw_r);
    }

}


/*Funciton Name: UpdateRawImage()                                                          */
/*Description: Update raw image                                                                    */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                          */

void RScamera::UpdateRawImage(){

    rs2::frame left_frame = frames.get_fisheye_frame(1);
    rs2::frame right_frame= frames.get_fisheye_frame(2);
    Mat left(Size(raw_width, raw_height), CV_8UC1, (void*)left_frame.get_data(), Mat::AUTO_STEP);
    Mat right(Size(raw_width, raw_height), CV_8UC1, (void*)right_frame.get_data(), Mat::AUTO_STEP);
    raw_l = left;
    raw_r = right;

}


/*Funciton Name: UpdateIMU()                                                                      */
/*Description: Update IMU data                                                                      */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                         */

void RScamera::UpdateIMU(){

    imu.position.x = frames.get_pose_frame().get_pose_data().translation.x;
    imu.position.y = frames.get_pose_frame().get_pose_data().translation.y;
    imu.position.z = frames.get_pose_frame().get_pose_data().translation.z;

    imu.velocity.x = frames.get_pose_frame().get_pose_data().velocity.x;
    imu.velocity.y = frames.get_pose_frame().get_pose_data().velocity.y;
    imu.velocity.z = frames.get_pose_frame().get_pose_data().velocity.z;

    EulerAngles a = Transform::quaternion_to_euler_grad(frames.get_pose_frame().get_pose_data().rotation);
    imu.rotation.roll = a.roll;
    imu.rotation.pitch = a.pitch;
    imu.rotation.yaw = a.yaw;

}


/*Funciton Name: EnableRecord(string path, bool save_bag, bool save_video)  */
/*Description: Enable recording                                                                                                 */
/*Input: string path - folder to save the recorded files
                 bool save_bag - save the raw data bag
                 bool save_video - save video in .avi                                                                          */        
/*Output: void                                                                                                                                      */

void RScamera::EnableRecord(string path, bool save_bag, bool save_video){

    DisableRead();

    record = true;
    this->save_bag = save_bag;
    this->save_video = save_video;
    this->path_record = path;

    time_t now = time(0);
    tm *ltm = localtime(&now);
    string time = int_to_string(ltm->tm_year+1900)+int_to_string(ltm->tm_mon+1)+int_to_string(ltm->tm_mday)+"_"+int_to_string(ltm->tm_hour)+"-"+int_to_string(ltm->tm_min)+"-"+int_to_string(ltm->tm_sec);
    
    if (save_bag) {
        file_bag = path_record+ time+"_raw.bag";
        cfg.enable_record_to_file(file_bag);
    }

    if(save_video){
        file_left = path_record+ time+"_img_left.avi";
        file_right = path_record + time+"_img_right.avi";
        writer_left = VideoWriter(file_left, VideoWriter::fourcc('M', 'J', 'P', 'G'),  int(fps), Size(raw_width, raw_height), false); 
        writer_right = VideoWriter(file_right, VideoWriter::fourcc('M', 'J', 'P', 'G'),  int(fps), Size(raw_width,raw_height), false); 
    }

}


/*Funciton Name: DisableRecord()                                                                */
/*Description: Disable recording                                                                     */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                         */

void RScamera::DisableRecord(){

    record = false;
    save_video = false;
    save_bag = false;
    cfg = config();

}


/*Funciton Name: EnableRead(string path)                                               */
/*Description: Enable reading from a .bag file                                          */
/*Input: string path - path to a .bag file                                                        */        
/*Output: void                                                                                                         */

void RScamera::EnableRead(string path){

    DisableRecord();

    path_file = path;
    read_from_file = true;

    cfg.enable_device_from_file(path);

}


/*Funciton Name: DisableRead()                                                                     */
/*Description: Disable reading                                                                          */
/*Input: void                                                                                                             */        
/*Output: void                                                                                                         */

void RScamera::DisableRead(){

    read_from_file = false;
    cfg = config();

}


/*Funciton Name: RawLeft()                                                                              */
/*Description: Get raw image left                                                                    */
/*Input: void                                                                                                             */        
/*Output: Mat raw_l                                                                                              */

Mat RScamera::RawLeft(){

    return raw_l;

}


/*Funciton Name: RawRight()                                                                           */
/*Description: Get raw image right                                                                 */
/*Input: void                                                                                                             */        
/*Output: Mat raw_r                                                                                              */

Mat RScamera::RawRight(){

    return raw_r;

}


/*Funciton Name: IMU()                                                                                      */
/*Description: Get IMU data                                                                               */
/*Input: void                                                                                                             */        
/*Output: Motion imu                                                                                           */

Motion RScamera::IMU(){

    return imu;

}


/*Funciton Name: int_to_string(int i)                                                            */
/*Description: convert integer to double digit string                              */
/*Input: int i - integer to convert                                                                      */        
/*Output: string resul                                                                                           */

string RScamera::int_to_string(int i){

    string result;
    if(i<10)  result = "0" + to_string(i);
    else result = to_string(i);

    return result;
}
