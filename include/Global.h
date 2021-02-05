/*******************************************************************************
Author:                 J. Gong
Date:                      2020-12-18
Description:        Declear global variables
*******************************************************************************/

#pragma once

// Visualization
extern bool VISUAL_ORIGINAL;
extern bool VISUAL_RECTIFIED;
extern bool VISUAL_DISPARITY;
extern bool VISUAL_DISPARITY_NO_GROUND;
extern bool VISUAL_DETECTION;
extern bool VISUAL_MASKED_DISPARITY;
extern bool VISUAL_LANE;
extern bool VISUAL_U_V_DISPARITY;


// Ticking
extern int start_raw, stop_raw;
extern int start_rectify, stop_rectify;
extern int start_disp, stop_disp;
extern int start_ground_filter, stop_ground_filter;
extern int start_detection, stop_detection;