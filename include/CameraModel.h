#pragma once

#include<iostream>
#include <opencv2/opencv.hpp>

#include"Motion.h"

using namespace std;
using namespace cv;

class CameraModel{

    protected:

        string calibration_file;

        // Size
        int raw_width;
        int raw_height;
        int output_width;
        int output_height;
        int resize_factor;
        int offset_x;
        int offset_y;

        // Geometry
        int cx;
        int cy;
        int output_cx;
        int output_cy;
        int f;
        Mat matrixQ;
        float baseline;
        Mat R1, P1, K1, D1, R2, P2, K2, D2;

        Mat rmap[2][2];

        // Camera
        // int raw_fps;
        // int sample_rate;
        // float fps;

        // // RealSense API
        // bool running;

        // Initialize rectification map
        void InitMap();

        // Motion* abs_motion;


    public:

        // Construct
        CameraModel();
        CameraModel(string calibration);
        // CameraModel(string calibration, int sample_rate=1);


        // Set sources
        // void SetMotionSource(Motion* abs_motion);

        // Print
        void PrintParameters();
        void TransformTest();

        // Transform
        int Meter_Pixel(float size_m, float distance_m);
        float Pixel_Meter(int size_pixel, int disparity_pixel);
        int Distance_Disparity(float distance_m);
        float Disparity_Distance(int disparity_pixel);

        // Access
        Mat Q();
        // int SampleRate();
        int OutWidth();
        int OutHeight();
        int RawWidth();
        int RawHeight();
        int OutCx();
        int OutCy();
        int OffsetX();
        int OffsetY();
        float Baseline();
        int F();
        // float FPS();

        // Rectify
        void Rectify(Mat& img_l, Mat& img_r);      

};

class MonoCameraModel: public CameraModel{

    private: 
        float* cam_height;
        float* cam_theta;

    public:
        MonoCameraModel();
        MonoCameraModel(string config);
        void SetHeightSource(float* cam_height);
        void SetThetaSource(float* cam_theta);
        int Distance_U(float distance_m);
        float U_Distance(int u_pixel);
        void TransformTest();
        float CamHeight();
        float CamTheta();

};
