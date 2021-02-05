#include<fstream>

#include "Global.h"
#include "CameraModel.h"


// ### CameraModel ###

// Construct empty
CameraModel::CameraModel(){}

// Construct with a calibration file
CameraModel::CameraModel(string calibration){
    
    FileStorage fs(calibration, FileStorage::READ);

    fs["raw_width"] >> raw_width;
    fs["raw_height"] >> raw_height;
    fs["output_width"] >> output_width;
    fs["output_height"] >> output_height;
    fs["resize_factor"] >> resize_factor;
    fs["offset_x"] >> offset_x;
    fs["offset_y"] >> offset_y;
    fs["cx"] >> cx;
    fs["cy"] >> cy;
    fs["output_cx"] >> output_cx;
    fs["output_cy"] >> output_cy;
    fs["f"] >> f;
    fs["Q"] >> matrixQ;
    fs["baseline"] >> baseline;
    fs["R1"] >> R1;
    fs["P1"] >> P1;
    fs["K1"] >> K1;
    fs["D1"] >> D1;
    fs["R2"] >> R2;
    fs["P2"] >> P2;
    fs["K2"] >> K2;
    fs["D2"] >> D2;

    InitMap();

}

// Print camera information
void CameraModel::PrintParameters(){

    cout << "################# Camera Information #################"<<endl; 
    cout <<"Raw image: \t\t\t" << raw_width << "x" << raw_height << " pixel" << endl;
    cout << "Output image:\t\t\t " << output_width << "x" << output_height << " pixel" << endl;
    cout << "Output projection center: \t(" << output_cx << "," << output_cy << ")" <<endl;
    cout << "Focal length f: \t\t" << f << " pixel" << endl;
    cout << "Baseline b: \t\t\t" << baseline << " m" << endl;
    cout << "#########################################################"<<endl; 

}

// Test 4 transform functions
void CameraModel::TransformTest(){

    cout << "################# Transform Test #################"<<endl; 

    cout<<"Test with an object of size 0.3m at distance 1.0m"<<endl;

    int disp = this->Distance_Disparity(1.0);
    cout<<"Disparity of distance 1.0m: \t\t"<<disp<<" pixel"<<endl;
    cout<<"Transform disparity back to distance: \t"<< this->Disparity_Distance(disp)<<" m"<<endl;

    int size_p = this->Meter_Pixel(0.3, 1.0);
    cout<<"Size of the object in image:\t\t"<<size_p<<" pixel"<<endl;
    cout<<"Transform size back to actual size:\t"<<this->Pixel_Meter(size_p, disp)<<" m"<<endl;

    cout << "#########################################################"<<endl; 

}

// Mono and stereo, size_pixel is the image representation of an object of size_m and at a distance_m(disparity_pixel)
int CameraModel::Meter_Pixel(float size_m, float distance_m){

    int pixel;

    if (isgreater(distance_m,0))
        pixel = int(f * size_m / distance_m);
    else
        pixel = 0;
    
    return pixel;

}

float CameraModel::Pixel_Meter(int size_pixel, int disparity_pixel){

    float meter;

    if(size_pixel>0)
        meter = size_pixel * this->Disparity_Distance(disparity_pixel) / f;
    else
        meter = 0.0;
    
    return meter;

}

// Stereo, disparity_pixel in image is equivalent to distance_m in the world
int CameraModel::Distance_Disparity(float distance_m){

    int disparity_pixel;

    if (distance_m>0)
        disparity_pixel = int(f*baseline/distance_m);
    else
        disparity_pixel = 255;
    
    return disparity_pixel;

}

float CameraModel::Disparity_Distance(int disparity_pixel){

    float distance_m;

    if(disparity_pixel>0)
        distance_m = f * baseline / disparity_pixel;
    else
        distance_m = 0.0;
    
    return distance_m;

}

// Access variables
Mat CameraModel::Q(){

    return matrixQ;

}

int CameraModel::OffsetX(){

    return offset_x;

}

int CameraModel::OffsetY(){

    return offset_y;

}

float CameraModel::Baseline(){

    return baseline;

}

int CameraModel::F(){

    return f;

}

int CameraModel::OutWidth(){

    return output_width;

}

int CameraModel::OutHeight(){

    return output_height;

}

int CameraModel::RawWidth(){

    return raw_width;

}

int CameraModel::RawHeight(){

    return raw_height;

}

int CameraModel::OutCx(){

    return output_cx;

}

int CameraModel::OutCy(){

    return output_cy;

}

// Rectify and crop images
void CameraModel::Rectify(Mat& img_l, Mat& img_r){

    Mat img_l_rec, img_r_rec;
    
    // Remap
    remap(img_l, img_l_rec, rmap[0][0], rmap[0][1], INTER_LINEAR, BORDER_DEFAULT, Scalar());
    remap(img_r, img_r_rec, rmap[1][0], rmap[1][1], INTER_LINEAR, BORDER_DEFAULT, Scalar());

    // Crop
    img_l_rec(Rect(offset_x, offset_y, output_width, output_height)).copyTo(img_l);
    img_r_rec(Rect(offset_x, offset_y, output_width, output_height)).copyTo(img_r);

    if(VISUAL_RECTIFIED) {
        imshow("left, rectified", img_l);
        imshow("right, rectified", img_r);
    }    

}

// Initialize rectification map
void CameraModel::InitMap(){

    fisheye::initUndistortRectifyMap(K1, D1, R1, P1, Size(resize_factor*raw_width, resize_factor*raw_height), CV_16SC2, rmap[0][0], rmap[0][1]);
    fisheye::initUndistortRectifyMap(K2, D2, R2, P2, Size(resize_factor*raw_width, resize_factor*raw_height), CV_16SC2, rmap[1][0], rmap[1][1]);

}



// ### MonoCameraModel ###

MonoCameraModel::MonoCameraModel(){}

MonoCameraModel::MonoCameraModel(string config): CameraModel(config){}

// Mono, u_pixel is the image coordinate of the ground plane at distance_m
int MonoCameraModel::Distance_U(float distance_m){

    int u_pixel;

    float y = *cam_height;
    float roll = *cam_theta;

    if(isgreater(distance_m,0))
        u_pixel = int(f * tan(tanh(y/(distance_m+0.0001))+roll * M_PI/180)+output_cy);
    else 
        u_pixel = 0;

    return u_pixel;

}

float MonoCameraModel::U_Distance(int u_pixel){

    float distance_m;

    float y = *cam_height;
    float roll = *cam_theta;

    if (u_pixel>0) 
        // distance_m = abs_motion.position.y / (tan(tanh((u_pixel-output_cy)/f)-abs_motion.rotation.roll*M_PI/180) + 0.0001);
        distance_m = y / (tan(-1*roll*M_PI/180 + atan2((u_pixel-output_cy),f) ) + 0.0001);
    else 
        distance_m = 0.0;

	return distance_m;

}

void MonoCameraModel::SetHeightSource(float* cam_height){

    this->cam_height = cam_height;

}

void MonoCameraModel::SetThetaSource(float* cam_theta){

    this-> cam_theta = cam_theta;

}

void MonoCameraModel::TransformTest(){

    cout << "################# Transform Test #################"<<endl; 

    cout<<"Test with an object of size 0.3m at distance 1.0m"<<endl;

    int disp = this->Distance_Disparity(1.0);
    cout<<"Disparity of distance 1.0m: \t\t"<<disp<<" pixel"<<endl;
    cout<<"Transform disparity back to distance: \t"<< this->Disparity_Distance(disp)<<" m"<<endl;

    int size_p = this->Meter_Pixel(0.3, 1.0);
    cout<<"Size of the object in image:\t\t"<<size_p<<" pixel"<<endl;
    cout<<"Transform size back to actual size:\t"<<this->Pixel_Meter(size_p, disp)<<" m"<<endl;

    cout << "---------------------------------------------------------"<<endl; 
    cout<<"Following test is only valid when motion is up-to-date"<<endl;
    cout << "---------------------------------------------------------"<<endl; 
    int u = this->Distance_U(1.0);
    cout<<"Ground plane at 1.0m: \t\t\t"<<u<<" pixel"<<endl;
    cout<<"Transform back to actual ground: \t"<<this->U_Distance(u)<<" m"<<endl;

    cout << "#########################################################"<<endl; 

}

float MonoCameraModel::CamHeight(){

    return *cam_height;

}

float MonoCameraModel::CamTheta(){

    return *cam_theta;

}

