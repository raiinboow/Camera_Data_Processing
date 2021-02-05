/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        main
*****************************************************************/

# include "main.h"


int main(int, char**){

    // Create detector object
    CameraObjectDetection cod = CameraObjectDetection("../config/config.yml");

    // Start pipeline
    cod.Run();
    
    return 0;

}
