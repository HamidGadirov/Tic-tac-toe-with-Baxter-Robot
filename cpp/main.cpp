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
#include <Gui/opengl.h>
#include <Kin/frame.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/rosCamera.h>
#include <RosCom/roscom.h>
#include <iostream>
#ifndef ROSCOM_BAXTER
#include <RosCom/baxter.h>
#define ROSCOM_BAXTER 1
#endif
#include "controller.h"
#include "kinematic.h"
#include "kinmodel.h"

#include <limits>

void printTakenCells(Vision &vision) {
    cout << vision.detectTokens() << '\n';
}

int main(int argc, char **argv) {
    rai::initCmdLine(argc, argv);

    if (1) {
        Controller controller;
        controller.startController();
        return 0;
    }

    rai::KinematicWorld &K = KinModel::instance().K;
    RobotOperation &B = KinModel::instance().B;
    KinModel::instance().sync();

    Vision vision;

    Kinematic kinematic;

    vision.threadLoop();

    cout << "Joint State" << endl << B.getJointPositions() << endl;

    int select;
    while (true) {
        cout << "select cell to draw (1-9) and 0 for home" << endl;
        std::cin >> select;
        switch (select) {
        case 0:
            kinematic.goHome();
            break;

        case 10:
            kinematic.toggleVacuumGripper();
            break;

        case 99:
            printTakenCells(vision);
            break;

        case 11:
            kinematic.liftEndeffector(0.05, vision.getBoard().getY());
            break;

        default:
            arr pos2 = vision.getBoard().getCellPosition(select - 1);
            arr y = vision.getBoard().getY();
            arr pos1 = pos2;
            pos1(2) = pos2(2) + 0.05;
            cout << "pos1: " << pos1 << " , pos2: " << pos2 << endl;

            arr new_q = kinematic.getJointState(pos1, K.getJointState(), y);
            arr new_q2 = kinematic.getJointState(pos2, new_q, y);
            cout << "test " << endl;
            kinematic.goToJointState(new_q);
            kinematic.goToJointState(new_q2);
            K.setJointState(new_q);
            K.watch(false);
            break;
        }
    }

    return 0;
}
