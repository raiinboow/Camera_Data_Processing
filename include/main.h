/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        main
*****************************************************************/

#pragma once

#include "Global.h"
#include "CameraObjectDetection.h"


// Global visual setting
bool VISUAL_ORIGINAL;
bool VISUAL_RECTIFIED;
bool VISUAL_DISPARITY;
bool VISUAL_DISPARITY_NO_GROUND;
bool VISUAL_DETECTION;
bool VISUAL_LANE;
bool VISUAL_MASKED_DISPARITY;
bool VISUAL_U_V_DISPARITY;

// Global ticking variables
int start_raw, stop_raw;
int start_rectify, stop_rectify;
int start_disp, stop_disp;
int start_ground_filter, stop_ground_filter;
int start_detection, stop_detection;