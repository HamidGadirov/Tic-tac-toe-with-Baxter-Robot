#ifndef RAI_OPENCV
#define RAI_OPENCV 1
#endif
#ifndef RAI_ROS
#define RAI_ROS 1
#endif
#include <Perception/opencv.h>
#ifndef PERCEPTION_OPENCVCAMERA
#include <Perception/opencvCamera.h>
#define PERCEPTION_OPENCVCAMERA
#endif
#include <iostream>
#include <Gui/opengl.h>
#include <Kin/frame.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/rosCamera.h>
#include <RosCom/roscom.h>
#ifndef ROSCOM_BAXTER
#include <RosCom/baxter.h>
#define ROSCOM_BAXTER 1
#endif
#include "vision.h"
#include "board.h"
#include "kinematic.h"
#include "kinmodel.h"

#include <limits>

void printTakenCells(Vision vision){
    int color;
    for (int i = 0; i < 9; i++)
    {
        color = vision.detectToken(i); // blue 0, green 1, red 2
        switch (color)
        {
        case 0:
            cout<<"cell: "<<i<<"blue"<<endl;
            break;

        case 1:
            cout<<"cell: "<<i<<"green"<<endl;
            break;
            
        case 2:
            cout<<"cell: "<<i<<"red"<<endl;
            break;
        
        default:
            cout<<"cell: "<<i<<"error"<<endl;
            break;
        }
    }

}

void nextMove(Vision vision){

    if ( vision.detectNextMove() ) {
        printTakenCells(vision);
    }

}


int main(int argc, char **argv) {
    rai::initCmdLine(argc, argv);
    rai::KinematicWorld &K = KinModel::instance().K;
    RobotOperation* B = KinModel::instance().B;

    

    //KinModel::getK()->addFile("../rai-robotModels/baxter/baxter_new.g");
    //K.addFile("../rai-robotModels/baxter/baxter_new.g");
    arr q_real; //= KinModel::getOp()->getJointPositions();
    //cout << "q_real:" << q_real << endl;
    q_real = B->getJointPositions();
    cout << "Join Positions: " << q_real << endl;
    if (q_real.N == K.getJointStateDimension()){
        K.setJointState(q_real);
    }

    Vision vision;

    Kinematic kinematic;

    vision.threadLoop();

    while(true){
        int select;
        cout << "Joint Names:" << K.getJointNames() << endl;
        cout << "select cell to draw (1-9) and 0 for home" << endl;
        std::cin >> select;
        switch (select)
        {
        case 0:
            kinematic.goHome();
            break;

        case 10:
            kinematic.toggleVacuumGripper();
            break;

        case 11:
            printTakenCells(vision);
            break;

        case 12:
            nextMove(vision);
            break;
        
        default:
            arr pos1 = vision.getBoard().getCellPosition(select-1);
            K.getFrameByName("ball")->setPosition(pos1);
            arr new_q = kinematic.getJointState(pos1, K.getJointState());
            //kinematic.goToJointState(new_q);
            K.setJointState(new_q);
            K.watch(false);
            break;
        }

    }

    return 0;
}
