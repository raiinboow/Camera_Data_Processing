/*****************************************************************
Author:                 J. Gong
Date:                      2020-12-22
Description:        Motion
*****************************************************************/

#include "Motion.h"


/*****************************************************************
Class Name: Motion
Description: Class to handle motion
*****************************************************************/

/*Funciton Name: Motion()                                          								            */
/*Description: Default constructor                                                                    */
/*Input: void                                                                                                               */   

// Motion::Motion(){}


/*Funciton Name: InitPosition(Point3f init_pos)                                        */
/*Description: Initialize motion with a 3D point                                         */
/*Input: Point3f init_pos - initial position                                                     */        
/*Output: void                                                                                                           */   

void Motion::InitPosition(Point3f init_pos){

    this->init_position.x = init_pos.x;
    this->init_position.y = init_pos.y;
    this->init_position.z = init_pos.z;

}


/*Funciton Name: Reset()                                                                                      */
/*Description: Reset motion back to initial position                                 */
/*Input: void                                                                                                                */        
/*Output: void                                                                                                           */   

void Motion::Reset(){

    position = init_position;
    velocity = Point3f(0,0,0);
    
}


/*Funciton Name: PrintMotion(string s)                                                         */
/*Description: Print motion                                                                                  */
/*Input: string s - additional information to be printed                          */        
/*Output: void                                                                                                           */  

void Motion::PrintMotion(string s){

        cout<<s<<" pos:\tX:"<<setprecision(3)<<this->position.x; 
        cout<<"\tY:"<<setprecision(3)<<this->position.y; 
        cout<<"\tZ:"<<setprecision(3)<<this->position.z<<endl; 
        cout<<s<<" vel:\tv_X:"<<setprecision(3)<<this->velocity.x; 
        cout<<"\tv_Y:"<<setprecision(3)<<this->velocity.y; 
        cout<<"\tv_Z:"<<setprecision(3)<<this->velocity.z<<endl; 
        cout<<s<<" rot:\tRoll:"<<setprecision(3)<<this->rotation.roll; 
        cout<<"\tPitch:"<<setprecision(3)<<this->rotation.pitch; 
        cout<<"\tYaw:"<<setprecision(3)<<this->rotation.yaw<<endl; 
        cout<<"---------------------------------------------------------------"<<endl; 

}


/*Funciton Name: Update(Motion rel_motion)                                           */
/*Description: Update motion with a relative motion                             */
/*Input: Motion rel_motion - relative motion                                              */        
/*Output: void                                                                                                           */  

void Motion::Update(Motion rel_motion){

    this->position.x = this->init_position.x  + rel_motion.position.x ;
    this->position.y = this->init_position.y + rel_motion.position.y;
    this->position.z = this->init_position.z + rel_motion.position.z;

    this->velocity.x = rel_motion.velocity.x;
    this->velocity.y = rel_motion.velocity.y;
    this->velocity.z = rel_motion.velocity.z;
    
    this->rotation.pitch = rel_motion.rotation.pitch;
    this->rotation.roll = rel_motion.rotation.roll;
    this->rotation.yaw = rel_motion.rotation.yaw;

}