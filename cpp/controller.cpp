#include "controller.h"

Controller::Controller() {}

void Controller::startController() {
    cout << "Starting Controller..." << endl;

    rai::KinematicWorld &K = KinModel::instance().K;

    KinModel::instance().sync();

    vision.threadLoop();

    int select;
    cout << "Press 0 for homing, other number to continue." << endl;

    std::cin >> select;

    if (select == 0) {
        // Move to home position
        kinematic.goHome();
    }


    bool board_empty = false;
    Grid placed;
    Grid placed_buff;
    while(board_empty == false){
        while (vision.getBoard().updated <= 10) {
            cout << "detecting board..." << endl;
            rai::wait(1.);
        }
        placed = vision.detectTokens();
        board_empty = true;
        for (Cell c : placed) {
            if (c != Cell::Empty) {
                board_empty = false;
                cout << "Board is not empty. Please Remove tokens!" << endl;
                rai::wait(1.);
                break;
            }
        }
    }


    cout << "Empty board detected!" << endl;
    // Check that board is empty:
    placed = vision.detectTokens();

    cout << "Starting game: Press.. " << endl;
    cout << "  1 - human starts" << endl;
    cout << "  2 - robot starts" << endl;

    
    std::cin >> select;

    if (select == 1) {
        cout << "Human starts..." << endl;
        do{
            vision.detectNextMove(placed);
            placed_buff = vision.detectTokens();

            if(gamelogic.detectCheating(placed,placed_buff,Cell::Empty) &&
                gamelogic.detectCheating(placed,placed_buff,Cell::Player)){
                    cout << "You cheater!" << endl;
                    kinematic.goCheat(0.5);
            }
        }while(gamelogic.detectCheating(placed,placed_buff,Cell::Player));
        placed = placed_buff;


        //vision.detectNextMove(placed);
        cout << "next move detected..." << endl;
        //placed = vision.detectTokens();
    }
    else{
        cout << "Robot starts" << endl;
    }

    GameState currentState;

    while (true) {
        // Robot's Turn: ------------------------------------------------------

        // Suspend vision task
        //vision.threadStop();

        // Grasp token:
        while(!graspToken()){
            kinematic.goHome(5.);
            vision.threadLoop();
            rai::wait(2.);
        }

        // Detect current game board status
        
        //placed = vision.detectTokens();



        int nextcell = gamelogic.getNextMove(placed);
        if (nextcell < 0 || nextcell > 8) {
            cout << "ERROR wrong next cell..." << endl;
            break;
        }

        cout << "Place token in cell: " << nextcell << endl;
        placed[nextcell] = Cell::Robot;

        // Drive to position:
        arr pos2 = vision.getBoard().getCellPosition(nextcell);
        arr ori = vision.getBoard().getOrientation();
        pos2 -= 0.01;
        arr pos1 = pos2;
        pos1(2) = pos2(2) + 0.05;

        arr new_q = kinematic.getJointState(pos1, K.getJointState(), ori);
        arr new_q2 = kinematic.getJointState(pos2, new_q, ori);
        kinematic.goToJointState(new_q,4.);
        kinematic.goToJointState(new_q2,3.);
        K.setJointState(new_q2);

        // Disable vacuum gripper:
        kinematic.toggleVacuumGripper();

        // Go vertical up:
        kinematic.liftEndeffector(0.05, arr(3,3,{0,0,0,0,0,0,0,0,0}),3.);

        currentState = gamelogic.getGameState(placed);

        if(GameState::RobotWon == currentState){
            kinematic.goCheer(5.);
            break;
        }


        // Go home:
        kinematic.goHome(4.);

        if(GameState::Draw == currentState){
            break;
        }

        // Restart vision thread
        vision.threadLoop();

        do{
            vision.detectNextMove(placed);
            placed_buff = vision.detectTokens();

            if(gamelogic.detectCheating(placed,placed_buff,Cell::Empty) &&
                gamelogic.detectCheating(placed,placed_buff,Cell::Player)){
                    cout << "You cheater!" << endl;
                    kinematic.goCheat(0.5);
            }
        }while(gamelogic.detectCheating(placed,placed_buff,Cell::Player));
        placed = placed_buff;

        currentState = gamelogic.getGameState(placed);
        if(GameState::Draw == currentState){
            break;
        }

    }
}

bool Controller::graspToken() {
    RobotOperation &B = KinModel::instance().B;
    rai::KinematicWorld &K = KinModel::instance().K;
    while(vision.getToken().valid < 0.7){
        cout << "No red token detected!" << endl;
        rai::wait(1.);
    }
    vision.threadStop();

    arr token_center = vision.getToken().center;
    arr ori = vision.getToken().getOrientation();
    token_center(2) -= 0.015;
    arr pos1 = token_center;

    pos1(2) = token_center(2) + 0.05;
    arr new_q = kinematic.getJointState(pos1, B.getJointPositions(),ori);
    arr new_q2 = kinematic.getJointState(token_center, new_q,ori);
    kinematic.goToJointState(new_q,4.);
    kinematic.goToJointState(new_q2,5.);
    K.setJointState(new_q2);
    int cnt = 0;
    while (B.getGripperOpened("left")) {
        // cout << "start vacuum gripper in 2 sec." << endl;
        // rai::wait(1.);
        cout << "start vacuum gripper in 0.5 sec." << endl;
        rai::wait(.5);
        kinematic.toggleVacuumGripper();
        rai::wait(1.);
        if(cnt == 2){
            arr buff = arr(3,3,{0,0,0,0,0,0,0,0,0});
            kinematic.liftEndeffector(0.05,buff,3.);
            return false;
        }
        cnt ++;
    }

    cout << "Token gasped" << endl;
    arr buff = arr(3,3,{0,0,0,0,0,0,0,0,0});
    kinematic.liftEndeffector(0.05,buff,3.);
    return true;
}
