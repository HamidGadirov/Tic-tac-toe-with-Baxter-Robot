#include "controller.h"

Controller::Controller() {}

void Controller::startController() {
    cout << "Starting Controller..." << endl;

    rai::KinematicWorld &K = KinModel::instance().K;

    KinModel::instance().sync();

    vision.threadLoop();

    while (vision.getBoard().updated <= 10) {
        cout << "detecting board..." << endl;
        rai::wait(1.);
    }

    // Check that board is empty:
    arr placed = vision.detectTokens(vision.getBoard());
    for (int i = 0; i < 9; i++) {
        if (placed(i) != 0.) {
            cout << "ERROR Board is not empty..." << endl;
            return;
        }
    }

    cout << "Board detected!" << endl;
    cout << "Starting game: Place a token (if you want to start) and then "
            "press any key."
         << endl;
    int select;
    std::cin >> select;

    // Saving current board status
    Board board = vision.getBoard();

    // Move to home position
    kinematic.goHome();

    while (true) {
        // Robot's Turn: ------------------------------------------------------

        // Grasp token:
        graspToken();

        // Detect current game board status
        placed = vision.detectTokens(board); // empty 0, red 1, blue 2
        cout << "placed:" << placed << endl;
        if (placed(0) == -1.) {
            cout << "ERROR while detecting tokens..." << endl;
            break;
        }

        int nextcell = gamelogic.getNextMove(placed);

        placed(nextcell) = 1.;
        if (nextcell < 0 || nextcell > 8) {
            cout << "ERROR wrong next cell..." << endl;
            break;
        }

        cout << "Place token in cell: " << nextcell << endl;

        // Drive to position:
        arr pos2 = vision.getBoard().getCellPosition(nextcell);
        arr y = vision.getBoard().getY();
        arr pos1 = pos2;
        pos1(2) = pos2(2) + 0.05;
        K.getFrameByName("ball")->setPosition(pos1);
        arr new_q = kinematic.getJointState(pos1, K.getJointState(), y);
        arr new_q2 = kinematic.getJointState(pos2, new_q, y);
        kinematic.goToJointState(new_q);
        kinematic.goToJointState(new_q2);
        K.setJointState(new_q);

        // Disable vacuum gripper:
        kinematic.toggleVacuumGripper();

        // Go vertical up:
        kinematic.liftEndeffector(0.05, vision.getBoard().getY());

        // Go home:
        kinematic.goHome();

        // Wait for human:
    }
}

void Controller::graspToken() {
    RobotOperation &B = KinModel::instance().B;

    while (B.getGripperOpened("left")) {
        cout << "start vacuum gripper in 2 sec." << endl;
        rai::wait(1.);
        cout << "start vacuum gripper in 1 sec." << endl;
        rai::wait(1.);
        kinematic.toggleVacuumGripper();
        rai::wait(1.);
    }
    cout << "Token gasped" << endl;
}
